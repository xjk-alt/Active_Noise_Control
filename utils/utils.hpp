#pragma once
#include <vector>

template <typename T>
void getVectorStatesElement(const std::vector<T> &state_vector, unsigned int pos, std::vector<double> &res)
{
    if (0 != res.size())
    {
        res.clear();
    }
    for (const T &state : state_vector)
    {
        res.push_back(state(pos));
    }
}