#include "ltlAutomaton.h"

double notOperator(double ap)
{
    return ap == 0 ? 1 : 0;
}

ltl_Automaton::ltl_Automaton()
{
    this->actualState = 0;
    this->parser = new mu::Parser();
    this->parser->DefineFun("not", notOperator);
    this->atomic_propositions = new std::map<std::string, double&>();
    this->state_machine = new std::vector<std::vector<std::tuple<int, std::string>>*>();
}

void ltl_Automaton::create_automaton(std::string ltl_formula)
{
    //Check if ther is already a state machone defined
    if(!this->state_machine->empty())
        this->state_machine->clear();
    // Parse LTL formula
    spot::parsed_formula parsed_input = spot::parse_infix_psl(ltl_formula);
    // Translate formula to an Automatom
    spot::translator trans;
    trans.set_type(spot::postprocessor::Monitor);
    trans.set_pref(spot::postprocessor::Deterministic);
    spot::twa_graph_ptr aut = trans.run(parsed_input.f);
    //Fill atomic propositions vector
    const auto ap_dict = aut->get_dict();
    for (spot::formula ap: aut->ap())
    {
        double ap_value = 0;
        std::string ap_name = ap.ap_name();
        this->atomic_propositions->insert(std::pair<std::string, double&>(ap_name, ap_value));
        //this->parser->DefineVar(ap_name, &ap_value);
    }
    //Create matrix
    unsigned num_states = aut->num_states();
    for (unsigned state = 0; state < num_states; ++state)
    {
        std::cout << "State: " << state << std::endl;
        auto graph_edges = new std::vector<std::tuple<int, std::string>>();
        for (auto& t: aut->out(state))
        {
            int dest_state = (int)t.dst;
            std::ostringstream formula_stream;
            spot::bdd_print_formula(formula_stream, ap_dict, t.cond);
            std::string cond = formula_stream.str();
            this->replaceBooleanOperators("&", "&&", cond);
            this->replaceBooleanOperators("|", "||", cond);
            this->replaceNotOperators(cond);
            auto edge_tuple = std::make_tuple(dest_state, cond);
            graph_edges->push_back(edge_tuple);
            // std::cout << "  edge(" << t.src << " -> " << t.dst << ")\n    label = ";
            // std::cout << cond << '\n';
        }
        this->state_machine->push_back(graph_edges);
    }
    this->actualState = aut->get_init_state_number();

    // std::cout << "Init state " << aut->get_init_state_number() << std::endl;
    // std::cout << "Acceptance states " << aut->get_acceptance() << std::endl;
}

bool ltl_Automaton::evaluate_formula(std::vector<std::tuple<std::string, int>> *chain, std::string *outFailState)
{
    for(auto state = chain->begin(); state != chain->end(); state++)
    {
        auto atomic_proposition = std::get<0>(*state);
        std::vector<double*>test;
        for(auto ap_pair = this->atomic_propositions->begin(); ap_pair != this->atomic_propositions->end(); ap_pair++)
        {
            auto value = new double(0);
            std::string ap_name = ap_pair->first;
            if(atomic_proposition.compare(ap_name) == 0)
                *value = 1;
            else
                *value = 0;
            test.push_back(value);
            this->parser->DefineVar(ap_name, value);
        }
        // for(auto i = test.begin(); i != test.end(); i++)
        //     std::cout << *i << std::endl;
        auto actual_state_list = this->state_machine->at(this->actualState);
        for(auto edge = actual_state_list->begin(); edge != actual_state_list->end(); edge++)
        {
            std::string expression = std::get<1>(*edge);
            this->parser->SetExpr(expression);
            double result = this->parser->Eval();
            // std::cout << "Eval: " << result << std::endl;
            // std::cout << "Expression: " << expression << " AP: " << atomic_proposition << std::endl;
            if (result && (edge+1) == actual_state_list->end())
                break;
            if (result)
                this->actualState = std::get<0>(*edge);
            else
            {
                *outFailState = atomic_proposition;
                return false;
            }
        }
    }
    return true;
}

void ltl_Automaton::setAtomicPropositionsTable(std::string actual_ap)
{
    return;
}

void ltl_Automaton::replaceBooleanOperators(std::string from_operator, std::string to_operator, std::string &sentence)
{
    auto and_star_pos = sentence.find(from_operator);
    if(and_star_pos != std::string::npos)
        sentence.replace(and_star_pos, from_operator.length(), to_operator);
}

void ltl_Automaton::replaceNotOperators(std::string &sentence)
{
    auto and_star_pos = sentence.find("!");
    while(and_star_pos != std::string::npos)
    {
        sentence.insert(sentence.begin() + and_star_pos + 3, ')');
        sentence.replace(and_star_pos, 1, "not(");
        and_star_pos = sentence.find("!");
    }
}
