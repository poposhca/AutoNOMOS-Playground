#ifndef __SEARCH_PLAN_H__
#define __SEARCH_PLAN_H__

#include <tuple>
#include <vector>
#include <algorithm>

class SearchedPlan
{
public:
    std::vector<int> *path;
    std::vector<int> *nextInvalidCells;
    std::vector<std::tuple<std::string, int>> *plann;
    bool isPlanSet;
    int startSearchCell ;
    int goalSearchCell;
    SearchedPlan();
    void setPlan(std::vector<int> *path, std::vector<std::tuple<std::string, int>> *plann);
    void pushPlan(std::vector<int> *path, std::vector<std::tuple<std::string, int>> *plann);
    void invalidPLanFromCell(std::string startCell, std::vector<int> *path, std::vector<std::tuple<std::string, int>> *plann);
    void getNextStep();
};

#endif