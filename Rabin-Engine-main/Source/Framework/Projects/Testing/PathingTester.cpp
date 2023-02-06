/******************************************************************************/
/*!
\file		PathingTester.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	System for executing and recording test results

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "PathingTester.h"
#include "Core/Serialization.h"
#include "Agent/AStarAgent.h"
#include <sstream>
#include "Misc/Stopwatch.h"
#include <iomanip>
#include <fstream>

namespace fs = std::filesystem;

const std::wstring screenshots[] = { L"Diagonal_", L"Smooth_", L"Rubber_", L"Smooth_Rubber_" };

bool PathTester::initialize()
{
    currentTest = -1;

    needScreenshot = -1;
    testsProcessed = 0;

    clearGuard = false;

    const bool result = load_tests();

    if (result == true)
    {
        goto_next_test();
    }

    return result;
}

void PathTester::set_agent(AStarAgent *a)
{
    agent = a;
}

const std::wstring &PathTester::get_button_text()
{
    return buttonText;
}

const std::wstring &PathTester::get_status_text()
{
    return statusText;
}

const std::wstring &PathTester::get_failed_text()
{
    return failedText;
}

void PathTester::clear()
{
    if (clearGuard == false)
    {
        statusText = L"";
        currentFailed = 0;
        failedData.clear();
    }
}

void PathTester::goto_next_test()
{
    currentTest = (currentTest + 1) % tests.size();

    const auto &name = tests[currentTest].get_name();

    buttonText = std::wstring(name.begin(), name.end());
}

void PathTester::goto_next_failed()
{
    currentFailed = (currentFailed + 1) % failedData.size();
}

bool PathTester::has_multiple_failed_tests()
{
    return failedData.size() > 1;
}

void PathTester::execute_current_test()
{
    if (tickQueue.empty() == true)
    {
        on_test_start();

        auto &test = tests[currentTest];
        totalTests = test.get_num_tests();

        tickQueue.push({ static_cast<size_t>(currentTest), false });

        outputName = test.get_name();
    }
    else
    {
        std::cout << "Attemped to run test while another test is in progress, ignoring request" << std::endl;
    }
}

void PathTester::execute_all_tests()
{
    if (tickQueue.empty() == true)
    {
        on_test_start();

        for (size_t i = 0; i < tests.size(); ++i)
        {
            tickQueue.push({ i, false });
            totalTests += tests[i].get_num_tests();
        }

        outputName = "All_Tests";
    }
    else
    {
        std::cout << "Attemped to run test while another test is in progress, ignoring request" << std::endl;
    }
}

void PathTester::execute_speed_test()
{
    terrain->goto_map(1);

    agent->set_heuristic_type(Heuristic::OCTILE);
    agent->set_heuristic_weight(1.01f);
    agent->set_debug_coloring(false);
    agent->set_movement_type(Movement::NONE);
    agent->set_method_type(Method::ASTAR);
    agent->set_rubberbanding(false);
    agent->set_smoothing(false);
    agent->set_single_step(false);

    Stopwatch timer;

    const size_t numIter = 1000;

    using ms = std::chrono::microseconds;
    std::array<ms, numIter> results;
    ms::rep fastest = std::numeric_limits<ms::rep>().max();
    ms::rep total = 0;

    for (size_t i = 0; i < numIter; ++i)
    {
        timer.start();

        for (const auto &[start, goal] : speedPaths)
        {
            agent->set_position(terrain->get_world_position(start));
            agent->path_to(terrain->get_world_position(goal), false);
        }

        timer.stop();

        results[i] = timer.microseconds();

        if (results[i].count() < fastest)
        {
            fastest = results[i].count();
        }

        total += results[i].count();
    }

    std::stringstream filename;
    filename << "Output/SpeedTest_";
    Serialization::generate_time_stamp(filename);
    filename << ".txt";

    std::ofstream file(filename.str());

    if (file)
    {
        file << "Fastest: " << fastest << " microseconds" << std::endl;
        file << "Average: " << total / numIter << " microseconds" << std::endl << std::endl;

        const std::streamsize width = 10;

        file << std::left << std::setfill(' ');

        file << std::setw(width) << "Test #" << "Microseconds" << std::endl;
        std::string temp;

        for (size_t i = 0; i < numIter; ++i)
        {
            temp = std::to_string(i + 1) + ":";
            file << std::setw(width) << temp << results[i].count() << std::endl;
        }

        file.close();
    }
}

void PathTester::tick()
{
    // check if the previous test needed a screenshot, now that we have rendered it
    if (needScreenshot != -1)
    {
        renderer->output_screenshot(screenshots[needScreenshot]);
        needScreenshot = -1;
    }

    if (tickQueue.empty() == false)
    {
        auto &topQueue = tickQueue.front();

        auto &test = tests[topQueue.index];
        
        // do any prep work needed
        if (topQueue.prepped == false)
        {
            topQueue.prepped = true;
            test.prep(agent);
            results.emplace_back(test.get_name(), test.get_settings());
        }

        auto &result = results.back();

        // run the next test in the test case
        const auto outcome = test.tick(agent, result);

        // see if we need to generate a screenshot for this test, after we render
        if (outcome.screenshot == true)
        {
            const auto &settings = test.get_settings();

            if (settings.smoothing && settings.rubberBanding)
            {
                needScreenshot = 3;
            }
            else if (settings.rubberBanding == true)
            {
                needScreenshot = 2;
            }
            else if (settings.smoothing == true)
            {
                needScreenshot = 1;
            }
            else
            {
                needScreenshot = 0;
            }
        }

        // if this was the final test in this test case
        if (outcome.complete == true)
        {
            // record any failures
            if (result.num_failing_tests() > 0)
            {
                const auto &failures = result.get_failed_tests();
                failedData.insert(failedData.end(), failures.begin(), failures.end());
            }

            // pop the queue
            tickQueue.pop();
        }
        else
        {
            // update the status message to show we finished another test
            ++testsProcessed;
            build_status_message();
        }
    }
    else
    {
        on_test_end();
    }
}

void PathTester::bootstrap(const std::string &name, Method method, Heuristic heuristic, float weight)
{
    PathRequest::Settings settings;
    settings.debugColoring = true;
    settings.method = method;
    settings.heuristic = heuristic;
    settings.rubberBanding = false;
    settings.singleStep = false;
    settings.smoothing = false;
    settings.weight = weight;

    const auto smallName = name + "_Small";
    const auto largeName = name + "_Large";

    PathingTestCase smallTest;
    smallTest.bootstrap(agent, settings, 10, name);
    const std::string smallFilename = smallName + ".txt";
    const auto smallFilepath = Serialization::testsPath / smallFilename;
    Serialization::serialize(smallTest, smallFilepath);

    PathingTestCase largeTest;
    largeTest.bootstrap(agent, settings, 50, name + "++");
    const std::string largeFilename = largeName + ".txt";
    const auto largeFilepath = Serialization::testsPath / largeFilename;
    Serialization::serialize(largeTest, largeFilepath);
}

void PathTester::bootstrap_speed()
{
    terrain->goto_map(1);

    const int maxRow = terrain->get_map_height() - 1;
    const int maxCol = terrain->get_map_width() - 1;

    static constexpr unsigned numTests = 100;

    std::vector<std::tuple<GridPos, GridPos>> points;
    points.reserve(numTests);

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

        points.emplace_back(std::make_tuple(start, goal));
    }

    const std::string filename = "Speed.txt";
    const auto filepath = Serialization::testsPath / filename;
    Serialization::serialize(points, filepath);
}

void PathTester::bootstrap()
{
    const unsigned numHeuristics = static_cast<unsigned>(Heuristic::NUM_ENTRIES);

    for (unsigned h = 0; h < numHeuristics; ++h)
    {
        bootstrap(get_heuristic_text(static_cast<Heuristic>(h)), Method::ASTAR, static_cast<Heuristic>(h), 1.0f);
    }

    bootstrap("Dijkstra", Method::ASTAR, Heuristic::EUCLIDEAN, 0.0f);
    bootstrap("RFW", Method::FLOYD_WARSHALL, Heuristic::EUCLIDEAN, 0.0f);
    bootstrap("JPSPlus", Method::JPS_PLUS, Heuristic::EUCLIDEAN, 0.0f);
    bootstrap("Goalbounding", Method::GOAL_BOUNDING, Heuristic::EUCLIDEAN, 0.0f);
    bootstrap_speed();
}

void PathTester::on_test_start()
{
    clearGuard = true;
    testsProcessed = 0;
    needScreenshot = -1;
    agent->set_movement_type(Movement::NONE);
    agent->set_single_step(false);
    Messenger::send_message(Messages::PATH_TEST_BEGIN);

    results.clear();
    failedData.clear();
}

void PathTester::on_test_end()
{
    clearGuard = false;
    Messenger::send_message(Messages::PATH_TEST_END);
    build_status_message(results);

    std::stringstream filename;
    filename << outputName << "_";

    Serialization::generate_time_stamp(filename);
    filename << ".txt";

    const auto filepath = Serialization::outputPath / filename.str();

    Serialization::serialize(results, filepath);

    if (failedData.size() > 0)
    {
        currentFailed = 0;
        recreate_failed_scenario(failedData.front());
        build_status_message(failedData.front());
    }
}

bool PathTester::load_tests()
{
    std::cout << "    Initializing Path Testing System..." << std::endl;

    const fs::directory_iterator dir(Serialization::testsPath);

    for (auto && entry : dir)
    {
        if (fs::is_regular_file(entry) == true)
        {
            // TODO: Find some better way of handling this?
            const fs::path path(entry);
            if (path.filename() == "Speed.txt")
            {
                Serialization::deserialize(speedPaths, path);
            }
            else
            {
                PathingTestCase temp;
                if (Serialization::deserialize(temp, path) == true)
                {
                    tests.emplace_back(std::move(temp));
                }
            }
        }
    }

    Callback clearCB = std::bind(&PathTester::clear, this);
    Messenger::listen_for_message(Messages::PATH_REQUEST_BEGIN, clearCB);
    Messenger::listen_for_message(Messages::MAP_CHANGE, clearCB);

    results.reserve(tests.size());

    return tests.size() > 0;
}

void PathTester::build_status_message(const std::vector<PathingTestResult> &results)
{
    size_t numPassed = 0;
    size_t numFailed = 0;
    size_t numVisual = 0;

    // count the total number of tests run, and their state
    for (const auto & r : results)
    {
        const size_t f = r.num_failing_tests();

        numPassed += r.num_passing_tests();
        numFailed += f;
        numVisual += r.num_visual_tests();
    }

    const size_t total = numPassed + numFailed;

    std::wstringstream stream;

    stream << L"Out of " << total << L" tests: ";

    const unsigned elementCount = static_cast<unsigned>(numPassed > 0) +
        static_cast<unsigned>(numFailed > 0) +
        static_cast<unsigned>(numVisual > 0);
    
    if (numPassed > 0)
    {
        stream << numPassed << L" passed";

        if (elementCount == 3)
        {
            stream << L", ";
        }
        else if (elementCount == 2)
        {
            stream << L" and ";
        }
    }

    if (numFailed > 0)
    {
        stream << numFailed << L" failed";

        if (elementCount == 3)
        {
            stream << L", and ";
        }
        else if (elementCount == 2 && numPassed == 0)
        {
            stream << L" and ";
        }

        // set to -1, so the next click brings it to 0
        currentFailed = -1;
    }

    if (numVisual > 0)
    {
        stream << numVisual << L" additional screenshots were generated for validation";
    }

    statusText = stream.str();
}

void PathTester::build_status_message(const PathingTestData &failed)
{
    const std::string temp(failed.get_message_text());
    failedText = std::wstring(temp.begin(), temp.end());
}

void PathTester::build_status_message()
{
    statusText = std::to_wstring(testsProcessed) + L" of " + std::to_wstring(totalTests) + L" tests have been executed";
}

void PathTester::recreate_failed_scenario(const PathingTestData &failed)
{
    build_status_message(failed);

    // make a copy of the failed test
    PathingTestData local(failed);

    // prevent the map change from wiping our data
    clearGuard = true;

    // run the test and discard the results
    local(agent);

    clearGuard = false;
}

std::filesystem::path PathTester::build_filename(const std::string & title)
{
    std::stringstream filename;

    filename << title << "_";
    Serialization::generate_time_stamp(filename);
    filename << ".txt";

    return Serialization::outputPath / filename.str();
}
