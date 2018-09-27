#include "search_tree.h"

searchTree::searchTree()
{
    this->tree = new std::vector<searchTreeNode*>();
}

void searchTree::insert(int origin_state, int goal_state, int control_signal, float delta_time)
{
    auto newNode = new searchTreeNode;
    newNode->origin_state = origin_state;
    newNode->goal_state = goal_state;
    newNode->control_signal = control_signal;
    newNode->delta_time = delta_time;
    newNode->nextStates = new std::vector<int>();
    auto lastNode = this->tree->back();
    lastNode->nextStates->push_back(origin_state);
    this->tree->push_back(newNode);
}

void searchTree::invalidRoute(int origin_state)
{
    //TODO: NOT SURE IF NEEDED
}