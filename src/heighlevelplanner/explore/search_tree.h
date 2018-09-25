#ifndef __SEARCH_THREE_H__
#define __SEARCH_THREE_H__

#include <map>

struct searchTreeNode
{
    int origin_state;
    int geal_state;
    int control_signal;
    float delta_time;
};

class searchTree
{
private:
    std::map<int, searchTreeNode> tree;
public:
    void insert(int origin_state, int geal_state, int control_signal, float delta_time);
    void invalidRoute(int origin_state);
};

#endif