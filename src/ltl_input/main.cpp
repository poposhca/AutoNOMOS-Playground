#include <ros/ros.h>
#include <iostream>
#include <string>
#include <spot/tl/parse.hh>
#include <spot/tl/print.hh>

#define RATE_HZ 5

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ltl_input");
    ros::NodeHandle nh;
    ros::Rate loop_rate(RATE_HZ);

    string ltl_formula = "!F(red & X(yellow))";
    spot::formula pf = spot::parse_formula(ltl_formula);
    print_lbt_ltl(std::cout, pf) << endl;
    return 0;
}