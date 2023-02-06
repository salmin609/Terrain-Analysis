/******************************************************************************/
/*!
\file		PathingTestCase.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	A collection of pathfinding tests

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include <vector>
#include "PathingTestData.h"
#include "PathingTestResult.h"

class PathingTestCase
{
    friend class Serialization;
public:

    struct Outcome
    {
        bool complete;
        bool screenshot;
    };

    Outcome tick(AStarAgent *agent, PathingTestResult &results);

    void prep(AStarAgent *agent);

    const std::string &get_name() const;

    void bootstrap(AStarAgent *agent, const PathRequest::Settings &set, unsigned numTests, const std::string &name);

    const PathRequest::Settings &get_settings() const;

    size_t get_num_tests();
private:
    std::string name;
    PathRequest::Settings settings;
    std::vector<PathingTestData> expectedResults;
    std::queue<size_t> tickQueue;

    PathingTestData &pop_queue();
};