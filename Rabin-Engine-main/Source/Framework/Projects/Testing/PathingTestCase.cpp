/******************************************************************************/
/*!
\file		PathingTestCase.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	A collection of pathfinding tests

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "PathingTestCase.h"
#include "Agent/AStarAgent.h"

PathingTestCase::Outcome PathingTestCase::tick(AStarAgent *agent, PathingTestResult &results)
{
    Outcome outcome { false, false };

    if (tickQueue.empty() == false)
    {
        // get the next test to run
        auto &test = pop_queue();

        // make a local copy of the test
        PathingTestData local(test);

        // run the test
        auto result = local(agent);
        bool failed = false;

        // check for any pre-mature failures
        switch (result)
        {
        case PathingTestData::Outcome::FAILED:
            results.add_failing_test(local);
            failed = true;
            break;
        case PathingTestData::Outcome::VALID:
            break;
        default:
            std::cout << name << " had an unexpected initial outcome of " <<
                static_cast<size_t>(result) << std::endl;
            outcome.screenshot = false;
            return outcome;
        }

        // compare the test to the expected results
        result = local == test;

        switch (result)
        {
        case PathingTestData::Outcome::INVALID:
        case PathingTestData::Outcome::FAILED:
            if (!failed)
            {
                results.add_failing_test(local);
            }
            break;
        case PathingTestData::Outcome::PASSED:
            results.add_passing_test(local);
            break;
        case PathingTestData::Outcome::SCREEN:
            results.add_visual_test(local);
            outcome.screenshot = true;
            break;
        default:
            std::cout << name << " had an unexpected validation outcome of " <<
                static_cast<size_t>(result) << std::endl;
            outcome.screenshot = false;
            break;
        }
    }
    else
    {
        outcome.complete = true;
    }

    return outcome;
}

void PathingTestCase::prep(AStarAgent * agent)
{
    agent->bulk_set_request_settings(settings);

    for (size_t i = 0; i < expectedResults.size(); ++i)
    {
        tickQueue.push(i);
    }
}

const std::string &PathingTestCase::get_name() const
{
    return name;
}

void PathingTestCase::bootstrap(AStarAgent *agent, const PathRequest::Settings &set, unsigned numTests, const std::string &n)
{
    name = n;
    settings = set;
    expectedResults.reserve(numTests);

    agent->bulk_set_request_settings(settings);

    const unsigned numMaps = static_cast<unsigned>(terrain->num_maps());
    expectedResults.reserve(numMaps * (numTests + 6));

    for (unsigned m = 0; m < numMaps; ++m)
    {
        terrain->goto_map(m);

        const int maxRow = terrain->get_map_height() - 1;
        const int maxCol = terrain->get_map_width() - 1;

        const GridPos c0 { 0, 0 };
        const GridPos c1 { maxRow, 0 };
        const GridPos c2 { maxRow, maxCol };
        const GridPos c3 { 0, maxCol };

        expectedResults.emplace_back();
        expectedResults.back().bootstrap(agent, c0, c1, m);

        expectedResults.emplace_back();
        expectedResults.back().bootstrap(agent, c1, c2, m);

        expectedResults.emplace_back();
        expectedResults.back().bootstrap(agent, c2, c3, m);

        expectedResults.emplace_back();
        expectedResults.back().bootstrap(agent, c3, c0, m);

        expectedResults.emplace_back();
        expectedResults.back().bootstrap(agent, c0, c2, m);

        expectedResults.emplace_back();
        expectedResults.back().bootstrap(agent, c1, c3, m);

        for (unsigned t = 0; t < numTests; ++t)
        {
            GridPos start;
            GridPos goal;

            while (true)
            {
                start.row = RNG::range(0, maxRow);
                start.col = RNG::range(0, maxCol);

                if (terrain->is_wall(start) == false)
                {
                    break;
                }
            }

            while (true)
            {
                goal.row = RNG::range(0, maxRow);
                goal.col = RNG::range(0, maxCol);

                if (goal != start && terrain->is_wall(goal) == false)
                {
                    break;
                }
            }

            expectedResults.emplace_back();
            expectedResults.back().bootstrap(agent, start, goal, m);
        }
    }
}

const PathRequest::Settings &PathingTestCase::get_settings() const
{
    return settings;
}

size_t PathingTestCase::get_num_tests()
{
    return expectedResults.size();
}

PathingTestData &PathingTestCase::pop_queue()
{
    auto &result = expectedResults[tickQueue.front()];
    tickQueue.pop();
    return result;
}
