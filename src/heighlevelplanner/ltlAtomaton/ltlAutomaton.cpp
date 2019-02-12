#include "ltlAutomaton.h"

ltl_Automaton::ltl_Automaton()
{

}

void ltl_Automaton::create_automaton(std::string ltl_formula)
{
    // Parse formula
    spot::parsed_formula parsed_input = spot::parse_infix_psl(ltl_formula);
    // Translate formula to Monitor Automatom
    spot::translator trans;
    trans.set_type(spot::postprocessor::BA);
    trans.set_pref(spot::postprocessor::Deterministic);
    spot::twa_graph_ptr aut = trans.run(parsed_input.f);
    // Get states dictionary
    const spot::bdd_dict_ptr& states_dict = aut->get_dict();
    // Print automata
    print_hoa(std::cout, aut) << '\n';

    // Print more info
    std::cout << aut->num_states() << std::endl;
    for (spot::formula ap: aut->ap())
      std::cout << ' ' << ap << " (=" << states_dict->varnum(ap) << ')';
    std::cout << '\n';
    std::cout << aut->get_init_state_number() << std::endl;

    

    unsigned n = aut->num_states();
    for (unsigned s = 0; s < n; ++s)
    {
        std::cout << "State: " << s << std::endl;
        for (auto& t: aut->out(s))
        {
            std::cout << "  edge(" << t.src << " -> " << t.dst << ")\n    label = ";
            spot::bdd_print_formula(std::cout, states_dict, t.cond);
            std::cout << "\n" << t.cond << '\n';
        }
    }

    //Prueba muParser
    double a = 1;
    double b = 0;
    mu::Parser parser;
    parser.DefineVar("a", &a);
    parser.DefineVar("b", &b);
    parser.SetExpr("a && b");
    std::cout << "Result: " << parser.Eval() << std::endl;

    char control;
    std::cin >> control;
}