#pragma once
#include <ct/core/core.h>
#include <vector>
#include <iostream>

template <typename T>
class TrajectoryLogger
{

public:
    TrajectoryLogger(std::string var_name, std::string log_path) : name(var_name), path(log_path) {}

    TrajectoryLogger(const TrajectoryLogger &other) : name(other.name), path(other.path), data(other.data) {}

    ~TrajectoryLogger() {}

    void log(const T &state)
    {
        data.push_back(state);
    }

    std::vector<T> data;

private:
    std::string name;
    std::string path;
};
