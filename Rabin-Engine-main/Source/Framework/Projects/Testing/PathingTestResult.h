/******************************************************************************/
/*!
\file		PathingTestResult.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	The results of pathfinding test

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include "PathingTestData.h"

class PathingTestResult
{
    friend class Serialization;
public:
    PathingTestResult();

    PathingTestResult(const std::string &name, const PathRequest::Settings &settings);

    void add_passing_test(const PathingTestData &test);
    void add_failing_test(const PathingTestData &test);
    void add_visual_test(const PathingTestData &test);

    size_t num_failing_tests() const;
    size_t num_passing_tests() const;
    size_t num_visual_tests() const;

    const std::vector<PathingTestData> &get_failed_tests() const;

    void clear();
private:
    std::string name;
    PathRequest::Settings settings;
    std::vector<PathingTestData> passed;
    std::vector<PathingTestData> failed;
    std::vector<PathingTestData> visual;
};