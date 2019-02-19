#include "ltlAutomaton.h"

ltl_Automaton::ltl_Automaton()
{
    this->state_machine = new std::vector<std::map<int, std::string>*>();
}

void ltl_Automaton::create_automaton(std::string ltl_formula)
{
    // Parse LTL formula
    spot::parsed_formula parsed_input = spot::parse_infix_psl(ltl_formula);
    // Translate formula to an Automatom
    spot::translator trans;
    trans.set_type(spot::postprocessor::BA);
    trans.set_pref(spot::postprocessor::Deterministic);
    spot::twa_graph_ptr aut = trans.run(parsed_input.f);
    //Create matrix
    unsigned num_states = aut->num_states();
    const auto ap_dict = aut->get_dict();
    for (unsigned state = 0; state < num_states; ++state)
    {
        std::cout << "State: " << state << std::endl;
        auto graph_edges = new std::map<int, std::string>();
        for (auto& t: aut->out(state))
        {
            int dest_state = (int)t.dst;
            std::ostringstream formula_stream;
            spot::bdd_print_formula(formula_stream, ap_dict, t.cond);
            std::string cond = formula_stream.str();
            graph_edges->insert(std::pair<int, std::string>(t.dst, cond));
            std::cout << "  edge(" << t.src << " -> " << t.dst << ")\n    label = ";
            std::cout << "\n" << cond << '\n';
        }
        this->state_machine->push_back(graph_edges);
        std::cout << "Acceptance states " << aut->get_acceptance() << std::endl;
    }

    char control;
    std::cin >> control;
}

bool ltl_Automaton::evaluate_formula(std::vector<std::tuple<int, std::string>> *chain)
{
    
}