#ifndef __LTLAUTOMATON_H__
#define __LTLAUTOMATON_H__

#include <iostream>
#include <string>
#include <spot/tl/parse.hh>
#include <spot/twaalgos/translate.hh>
#include <spot/twaalgos/hoa.hh>

class ltl_Automaton {

public:
    ltl_Automaton();
    void create_automaton(std::string ltl_formula);

};

#endif