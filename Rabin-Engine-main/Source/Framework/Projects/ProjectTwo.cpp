/******************************************************************************/
/*!
\file		ProjectTwo.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Specification for project two - pathfinding

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "ProjectTwo.h"

#include "Terrain/TerrainAnalysis.h"
#include "Agent/CameraAgent.h"

#include "UI/Elements/Buttons/UIButton.h"
#include "UI/Elements/Sliders/UISlider.h"
#include "UI/Elements/Text/UIValueTextField.h"
#include "UI/Elements/Buttons/UIDynamicButton.h"
#include "UI/Elements/Buttons/UIToggleButton.h"
#include "UI/Elements/Text/UIDynamicBannerTextField.h"
#include "UI/Elements/Buttons/UIConditionalButton.h"

#include "Misc/TimeTracker.h"

TimeTracker<std::chrono::microseconds> pathingTimer;

std::wstring startPosText;
std::wstring goalPosText;

const std::wstring &start_pos_text_getter()
{
    return startPosText;
}

const std::wstring &goal_pos_text_getter()
{
    return goalPosText;
}

void grid_pos_to_text(const GridPos &pos, std::wstring &text)
{
    text = std::wstring(L"[" + std::to_wstring(pos.row) + L", " + std::to_wstring(pos.col) + L"]");
}

bool ProjectTwo::initialize()
{
    std::cout << "Initializing Project Two..." << std::endl;

    // create all the systems that project two requires
    terrain = std::make_unique<Terrain>();
    agents = std::make_unique<AgentOrganizer>();
    ui = std::make_unique<UICoordinator>();
    pather = std::make_unique<AStarPather>();

    return terrain->initialize() &&
        agents->initialize() &&
        ui->initialize() &&
        pather->initialize() &&
        tester.initialize();
}

bool ProjectTwo::finalize()
{
    agent = agents->create_pathing_agent();
    tester.set_agent(agent);    

    // initialize the position text
    grid_pos_to_text(GridPos { -1, -1 }, startPosText);
    grid_pos_to_text(GridPos { -1, -1 }, goalPosText);

    build_ui();

    terrain->goto_map(1);
    terrain->pathLayer.set_enabled(true);

    //tester.bootstrap();

    // set up a time tracker to listen for pathing messages
    Callback timerResetCB = std::bind(&decltype(pathingTimer)::reset, &pathingTimer);
    Messenger::listen_for_message(Messages::PATH_REQUEST_BEGIN, timerResetCB);

    Callback timerStartCB = std::bind(&decltype(pathingTimer)::start, &pathingTimer);
    Messenger::listen_for_message(Messages::PATH_REQUEST_TICK_START, timerStartCB);

    Callback timerStopCB = std::bind(&decltype(pathingTimer)::stop, &pathingTimer);
    Messenger::listen_for_message(Messages::PATH_REQUEST_TICK_FINISH, timerStopCB);

    Callback testBeginCB = std::bind(&ProjectTwo::on_test_begin, this);
    Messenger::listen_for_message(Messages::PATH_TEST_BEGIN, testBeginCB);

    Callback testEndCB = std::bind(&ProjectTwo::on_test_end, this);
    Messenger::listen_for_message(Messages::PATH_TEST_END, testEndCB);

    link_input();

    return true;
}

void ProjectTwo::shutdown()
{
    std::cout << "Shutting Down Project Two..." << std::endl;
    pather->shutdown();
    pather.reset();

    ui->shutdown();
    ui.reset();

    agents->shutdown();
    agents.reset();

    terrain->shutdown();
    terrain.reset();
}

void ProjectTwo::draw_meshes()
{
    terrain->draw();
    agents->draw();
}

void ProjectTwo::draw_sprites()
{
    ui->draw_sprites();
}

void ProjectTwo::draw_text()
{
    ui->draw_text();
}

void ProjectTwo::draw_debug()
{
    terrain->draw_debug();
    agents->draw_debug();
    ui->draw_debug();
}

void ProjectTwo::update()
{
    // have the ui coordinator determine its state
    ui->update();

    // if we currently aren't running any tests
    if (testRunning == false)
    {
        // have the input system update its current state and send out notifications
        InputHandler::update();

        agents->update(deltaTime);
    }
    else
    {
        tester.tick();
    }
}

void ProjectTwo::build_ui()
{
    // create the first button in the top right,
    // 90 is half button width + 16 (padding from side of screen) + 10 (padding so sliders aren't directly against the edge)
    // 32 is half button height + 16 (padding from top of screen)
    Callback nextMapCB = std::bind(&Terrain::goto_next_map, terrain.get());
    auto mapButton = ui->create_button(UIAnchor::TOP_RIGHT, -90, 32, nextMapCB, L"Next Map");

    // just below the map button, add a button that transitions through pathfinding algorithms
    Callback methodCB = std::bind(&AStarAgent::next_method_type, agent);
    TextGetter methodText = std::bind(&AStarAgent::get_method_text, agent);
    auto methodButton = ui->create_dynamic_button(UIAnchor::BOTTOM, mapButton,
        10, methodCB, methodText);

    // then a button for the heuristic to use
    Callback heuristicCB = std::bind(&AStarAgent::next_heuristic_type, agent);
    TextGetter heuristicText = std::bind(&AStarAgent::get_heuristic_type_text, agent);
    auto heuristicButton = ui->create_dynamic_button(UIAnchor::BOTTOM, methodButton,
        10, heuristicCB, heuristicText);

    // then a slider to control the weight
    Getter<float> weightGet = std::bind(&AStarAgent::get_heuristic_weight, agent);
    Setter<float> weightSet = std::bind(&AStarAgent::set_heuristic_weight, agent, std::placeholders::_1);
    TextGetter weightText = std::bind(&AStarAgent::get_heuristic_weight_text, agent);
    auto weightSlider = ui->create_slider<float>(UIAnchor::BOTTOM, heuristicButton,
        10, 0.0f, 2.0f, weightGet, weightSet, weightText, L"Weight:");

    // then a button for smoothing
    Callback smoothingCB = std::bind(&AStarAgent::toggle_smoothing, agent);
    Getter<bool> smoothingGet = std::bind(&AStarAgent::get_smoothing, agent);
    auto smoothingButton = ui->create_toggle_button(UIAnchor::BOTTOM, weightSlider,
        10, smoothingCB, L"Smoothing", smoothingGet);

    // then a button for rubberbanding
    Callback rubberCB = std::bind(&AStarAgent::toggle_rubberbanding, agent);
    Getter<bool> rubberGet = std::bind(&AStarAgent::get_rubberbanding, agent);
    auto rubberButton = ui->create_toggle_button(UIAnchor::BOTTOM, smoothingButton,
        10, rubberCB, L"Rubberbanding", rubberGet);

    // then a button for single step
    Callback singleCB = std::bind(&AStarAgent::toggle_single_step, agent);
    Getter<bool> singleGet = std::bind(&AStarAgent::get_single_step, agent);
    auto singleButton = ui->create_toggle_button(UIAnchor::BOTTOM, rubberButton,
        10, singleCB, L"Single Step", singleGet);

    // then a button for debug coloring
    Callback debugCB = std::bind(&AStarAgent::toggle_debug_coloring, agent);
    Getter<bool> debugGet = std::bind(&AStarAgent::get_debug_coloring, agent);
    auto debugButton = ui->create_toggle_button(UIAnchor::BOTTOM, singleButton,
        10, debugCB, L"Debug Coloring", debugGet);

    // then a button for movement
    Callback movementCB = std::bind(&AStarAgent::next_movement_type, agent);
    TextGetter movementGet = std::bind(&AStarAgent::get_movement_type_text, agent);
    auto movementButton = ui->create_dynamic_button(UIAnchor::BOTTOM, debugButton,
        10, movementCB, movementGet);


    // add some text on the left side for displaying fps
    TextGetter fpsGetter = std::bind(&Engine::get_fps_text, engine.get());
    auto fpsText = ui->create_value_text_field(UIAnchor::TOP_LEFT, 90, 32, L"FPS:", fpsGetter);

    // and the pathing time
    TextGetter timeGetter = std::bind(&decltype(pathingTimer)::get_text, &pathingTimer);
    auto pathingTimeText = ui->create_value_text_field(UIAnchor::BOTTOM, fpsText,
        10, L"Time:", timeGetter);

    // and the current path start position
    TextGetter startGetter = start_pos_text_getter;
    auto startText = ui->create_value_text_field(UIAnchor::BOTTOM, pathingTimeText,
        10, L"Start:", startGetter);

    // and the current path goal position
    TextGetter goalGetter = goal_pos_text_getter;
    auto goalText = ui->create_value_text_field(UIAnchor::BOTTOM, startText,
        10, L"Goal:", goalGetter);

    Callback testCB = std::bind(&PathTester::execute_current_test, &tester);
    TextGetter testText = std::bind(&PathTester::get_button_text, &tester);
    auto testButton = ui->create_dynamic_button(UIAnchor::BOTTOM, goalText, 10,
        testCB, testText);

    Callback nextTestCB = std::bind(&PathTester::goto_next_test, &tester);
    auto nextTestButton = ui->create_button(UIAnchor::BOTTOM, testButton, 10,
        nextTestCB, L"Next Test");

    Callback allTestsCB = std::bind(&PathTester::execute_all_tests, &tester);
    auto allTestsButton = ui->create_button(UIAnchor::BOTTOM, nextTestButton, 10,
        allTestsCB, L"Run All Tests");

    Callback failureCB = std::bind(&PathTester::goto_next_failed, &tester);
    Getter<bool> failureCond = std::bind(&PathTester::has_multiple_failed_tests, &tester);
    auto failureButton = ui->create_conditional_button(UIAnchor::BOTTOM, allTestsButton,
        10, failureCB, L"Next Failed Test", failureCond);

    Callback speedCB = std::bind(&PathTester::execute_speed_test, &tester);
    auto speedTestButton = ui->create_button(UIAnchor::BOTTOM, failureButton, 10,
        speedCB, L"Run Speed Test");



    // add a text field at the top for the project
    auto projectBanner = ui->create_banner_text_field(UIAnchor::TOP, 0, 32,
        UIAnchor::CENTER, L"Project Two");

    // add a banner to display overall testing results
    TextGetter testingText = std::bind(&PathTester::get_status_text, &tester);
    auto testingBanner = ui->create_dynamic_banner_text_field(UIAnchor::BOTTOM,
        0, -16, UIAnchor::CENTER, testingText);

    // and a banner above that to display test specific failure messages
    TextGetter failMSGText = std::bind(&PathTester::get_failed_text, &tester);
    auto failedBanner = ui->create_dynamic_banner_text_field(UIAnchor::TOP,
        testingBanner, 10, UIAnchor::CENTER, failMSGText);
}


void ProjectTwo::link_input()
{
    Callback escapeCB = std::bind(&Engine::stop_engine, engine.get());
    InputHandler::notify_when_key_pressed(KBKeys::ESCAPE, escapeCB);

    Callback leftMouseCB = std::bind(&ProjectTwo::on_left_mouse_click, this);
    InputHandler::notify_when_mouse_pressed(MouseButtons::LEFT, leftMouseCB);

    Callback f1CB = std::bind(&ProjectTwo::on_f1, this);
    InputHandler::notify_when_key_pressed(KBKeys::F1, f1CB);

    Callback f3CB = std::bind(&ProjectTwo::on_f3, this);
    InputHandler::notify_when_key_pressed(KBKeys::F3, f3CB);
}

void ProjectTwo::on_left_mouse_click()
{
    const auto &mousePos = InputHandler::get_mouse_position();

    // convert the mouse position to a point on the terrain plane
    const auto worldPos = renderer->screen_to_world(mousePos.x, mousePos.y, terrain->get_terrain_plane());

    // verify a valid point was determine
    if (worldPos.second == true)
    {
        // convert the world position to a terraing grid position
        const auto gridPos = terrain->get_grid_position(worldPos.first);

        // verify that the grid position is valid and not a wall
        if (terrain->is_valid_grid_position(gridPos) && !terrain->is_wall(gridPos))
        {
            const auto agentPos = terrain->get_grid_position(agent->get_position());

            grid_pos_to_text(gridPos, goalPosText);
            grid_pos_to_text(agentPos, startPosText);

            // have the agent path to the position
            agent->path_to(worldPos.first);
        }
    }
    
}

void ProjectTwo::on_f1()
{
    engine->change_projects(Project::Type::ONE);
}

void ProjectTwo::on_f3()
{
    engine->change_projects(Project::Type::THREE);
}

void ProjectTwo::on_test_begin()
{
    testRunning = true;
}

void ProjectTwo::on_test_end()
{
    testRunning = false;
}
