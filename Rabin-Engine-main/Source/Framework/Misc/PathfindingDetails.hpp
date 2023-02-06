/******************************************************************************/
/*!
\file		PathfindingDetails.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Helper functions for displaying textual info and enumerations

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include "Misc/NiceTypes.h"
#include <list>

using WaypointList = std::list<Vec3>;

enum class Heuristic
{
    OCTILE,
    CHEBYSHEV,
    MANHATTAN,
    EUCLIDEAN,

    NUM_ENTRIES
};

const std::wstring &get_heuristic_wtext(Heuristic heuristic);
const std::string &get_heuristic_text(Heuristic heuristic);
Heuristic string_to_heuristic(const std::string &string);

enum class Method
{
    ASTAR,
    FLOYD_WARSHALL,
    JPS_PLUS,
    GOAL_BOUNDING,

    NUM_ENTRIES
};

const std::wstring &get_method_wtext(Method method);
const std::string &get_method_text(Method method);
Method string_to_method(const std::string &string);

struct PathRequest
{
    Vec3 start;
    Vec3 goal;
    WaypointList path;

    struct Settings
    {
        Method method;
        Heuristic heuristic;
        float weight;
        bool smoothing;
        bool rubberBanding;
        bool singleStep;
        bool debugColoring;
    } settings;

    bool newRequest;
};

enum class PathResult
{
    PROCESSING,
    COMPLETE,
    IMPOSSIBLE
};