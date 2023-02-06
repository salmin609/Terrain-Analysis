/******************************************************************************/
/*!
\file		PathingTester.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	System for executing and recording test results

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include "PathingTestCase.h"

class AStarAgent;

class PathTester
{
public:
    bool initialize();
    void set_agent(AStarAgent *agent);

    const std::wstring &get_button_text();
    const std::wstring &get_status_text();
    const std::wstring &get_failed_text();
    void clear();

    void goto_next_test();
    void goto_next_failed();

    bool has_multiple_failed_tests();

    void execute_current_test();
    void execute_all_tests();
    void execute_speed_test();

    void tick();

    void bootstrap();
private:
    std::vector<PathingTestCase> tests;
    std::vector<std::tuple<GridPos, GridPos>> speedPaths;
    std::vector<PathingTestData> failedData;

    struct QueueInfo
    {
        size_t index;
        bool prepped;
    };
    std::queue<QueueInfo> tickQueue;
    std::vector<PathingTestResult> results;
    std::wstring buttonText;
    std::wstring statusText;
    std::wstring failedText;
    std::string outputName;
    AStarAgent *agent;
    int currentTest;
    int currentFailed;
    int needScreenshot;
    size_t testsProcessed;
    size_t totalTests;
    bool clearGuard;

    void bootstrap(const std::string &name, Method method, Heuristic heuristic, float weight);
    void bootstrap_speed();
    void on_test_start();
    void on_test_end();
    bool load_tests();
    void build_status_message(const std::vector<PathingTestResult> &results);
    void build_status_message(const PathingTestData &failed);
    void build_status_message();
    void recreate_failed_scenario(const PathingTestData &failed);

    static std::filesystem::path build_filename(const std::string &title);
};