#ifndef PATHFIND_HPP_INCLUDED
#define PATHFIND_HPP_INCLUDED


#include <string>
#include <functional>


std::string pathFind
(
    const int xStart, const int yStart,
    const int xFinish, const int yFinish,
    std::function<bool(const int, const int)> collision
);

#endif // PATHFIND_HPP_INCLUDED
