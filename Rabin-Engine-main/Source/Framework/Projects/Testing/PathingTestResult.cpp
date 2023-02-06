/******************************************************************************/
/*!
\file		PathingTestResult.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	The results of pathfinding test

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "PathingTestResult.h"

PathingTestResult::PathingTestResult() : name("Invalid")
{}

PathingTestResult::PathingTestResult(const std::string &name,
    const PathRequest::Settings &settings) :
    name(name), settings(settings)
{}

void PathingTestResult::add_passing_test(const PathingTestData &test)
{
    passed.emplace_back(test);
}

void PathingTestResult::add_failing_test(const PathingTestData &test)
{
    failed.emplace_back(test);
}

void PathingTestResult::add_visual_test(const PathingTestData &test)
{
    visual.emplace_back(test);
}

size_t PathingTestResult::num_failing_tests() const
{
    return failed.size();
}

size_t PathingTestResult::num_passing_tests() const
{
    return passed.size();
}

size_t PathingTestResult::num_visual_tests() const
{
    return visual.size();
}

const std::vector<PathingTestData> &PathingTestResult::get_failed_tests() const
{
    return failed;
}

void PathingTestResult::clear()
{
    passed.clear();
    failed.clear();
    visual.clear();
}
