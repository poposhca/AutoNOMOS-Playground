#include "ltlAutomaton.h"

ltl_Automaton::ltl_Automaton()
{
    this->actualState = 0;
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
    //Create matrix
    unsigned num_states = aut->num_states();
    const auto ap_dict = aut->get_dict();
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
            auto edge_tuple = std::make_tuple(dest_state, cond);
            graph_edges->push_back(edge_tuple);

            std::cout << "  edge(" << t.src << " -> " << t.dst << ")\n    label = ";
            std::cout << cond << '\n';
        }
        this->state_machine->push_back(graph_edges);
    }
    this->actualState = aut->get_init_state_number();
    //Just for tests
    std::cout << "Init state " << aut->get_init_state_number() << std::endl;
    std::cout << "Acceptance states " << aut->get_acceptance() << std::endl;
}

bool ltl_Automaton::evaluate_formula(std::vector<std::tuple<std::string, int>> *chain)
{
    for(auto state = chain->begin(); state != chain->end(); state++)
    {
        auto atomic_proposition = std::get<0>(*state);
        auto actual_state_list = this->state_machine->at(this->actualState);
        for(auto edge = actual_state_list->begin(); edge != actual_state_list->end(); edge++)
        {
            std::cout << std::get<0>(*edge) << " " << std::get<1>(*edge) << std::endl;
        }
    }
    char control;
    std::cin >> control;
    return true;
}