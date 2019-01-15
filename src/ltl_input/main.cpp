#include <ros/ros.h>
#include <iostream>
#include <string>
#include <spot/tl/parse.hh>
#include <spot/twaalgos/translate.hh>
#include <spot/twaalgos/hoa.hh>

#define RATE_HZ 5

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ltl_input");
    ros::NodeHandle nh;
    //ros::Rate loop_rate(RATE_HZ);
    string ltl_formula = "!F(red & X(yellow))";
    spot::parsed_formula pf = spot::parse_infix_psl(ltl_formula);
    spot::translator trans;
    trans.set_type(spot::postprocessor::Monitor);
    trans.set_pref(spot::postprocessor::Deterministic);
    spot::twa_graph_ptr aut = trans.run(pf.f);
    print_hoa(std::cout, aut) << '\n';
    cout << "Just testing the node" << endl;
    return 0;
}