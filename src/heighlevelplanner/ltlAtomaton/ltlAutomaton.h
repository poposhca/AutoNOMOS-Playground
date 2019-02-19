#ifndef __LTLAUTOMATON_H__
#define __LTLAUTOMATON_H__

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <spot/tl/parse.hh>
#include <spot/parseaut/public.hh>
#include <spot/twaalgos/translate.hh>
#include <spot/twaalgos/hoa.hh>
#include <spot/twa/bddprint.hh>
#include "muParser.h"

class ltl_Automaton {

private:
    std::vector<std::map<int, std::string>*> *state_machine;

public:
    ltl_Automaton();
    void create_automaton(std::string ltl_formula);
    bool evaluate_formula(std::vector<std::tuple<int, std::string>> *chain);

};

#endif