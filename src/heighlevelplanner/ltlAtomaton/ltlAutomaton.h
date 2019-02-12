#ifndef __LTLAUTOMATON_H__
#define __LTLAUTOMATON_H__

#include <iostream>
#include <string>
#include <spot/tl/parse.hh>
#include <spot/parseaut/public.hh>
#include <spot/twaalgos/translate.hh>
#include <spot/twaalgos/hoa.hh>
#include <spot/twa/bddprint.hh>
#include "muParser.h"

class ltl_Automaton {

public:
    ltl_Automaton();
    void create_automaton(std::string ltl_formula);

};

#endif