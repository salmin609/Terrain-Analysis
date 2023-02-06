/******************************************************************************/
/*!
\file		PathingTestData.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	A single specific instance of a pathfinding test

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include "Misc/PathfindingDetails.hpp"
#include "Misc/NiceTypes.h"

class AStarAgent;
class Serialization;

class PathingTestData
{
    friend class Serialization;
public:
    PathingTestData();
    PathingTestData(const PathingTestData &other);

    PathingTestData &operator=(const PathingTestData &rhs);

    enum class Outcome
    {
        FAILED,
        PASSED,
        SCREEN,
        VALID,
        INVALID
    };

    Outcome operator()(AStarAgent *agent);

    Outcome operator==(const PathingTestData &rhs);

    const char *get_message_text() const;

    void bootstrap(AStarAgent *agent, const GridPos &st, const GridPos &gl, int map);

private:
    int map;
    GridPos start;
    GridPos goal;
    int distCard, distDiag;
    const char *message;
    bool hasSolution;
    bool requiresVisualConfirmation;

    bool calculate_distance(const WaypointList &path);
    bool calculate_distance(const GridPos &p0, const GridPos &p1);
};