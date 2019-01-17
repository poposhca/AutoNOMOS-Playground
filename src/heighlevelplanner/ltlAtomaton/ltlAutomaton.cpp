#include "ltlAutomaton.h"

ltl_Automaton::ltl_Automaton()
{

}

void ltl_Automaton::create_automaton(std::string ltl_formula)
{
    spot::parsed_formula pf = spot::parse_infix_psl(ltl_formula);
    spot::translator trans;
    trans.set_type(spot::postprocessor::Monitor);
    trans.set_pref(spot::postprocessor::Deterministic);
    spot::twa_graph_ptr aut = trans.run(pf.f);
    print_hoa(std::cout, aut) << '\n';
    std::cout << "Just testing the node" << std::endl;
    char control;
    std::cin >> control;
}