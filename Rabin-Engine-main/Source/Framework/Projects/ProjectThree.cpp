/******************************************************************************/
/*!
\file		ProjectThree.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Specification for project three - terrain analysis

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "ProjectThree.h"
#include "Misc/TimeTracker.h"
#include "Terrain/TerrainAnalysis.h"
#include "Agent/AStarAgent.h"
#include <sstream>
#include "Agent\EnemyAgent.h"
#include "UI/Elements/Buttons/UIButton.h"
#include "UI/Elements/Sliders/UISlider.h"
#include "UI/Elements/Text/UIValueTextField.h"
#include "UI/Elements/Buttons/UIDynamicButton.h"
#include "UI/Elements/Buttons/UIToggleButton.h"

TimeTracker<std::chrono::microseconds> analysisTimer;
bool enemyFoundPlayer = false;

bool ProjectThree::initialize()
{
    std::cout << "Initializing Project Three..." << std::endl;

    // create all the systems that project one requires
    terrain = std::make_unique<Terrain>();
    agents = std::make_unique<AgentOrganizer>();
    ui = std::make_unique<UICoordinator>();
    pather = std::make_unique<AStarPather>();

    set_analysis_frequency(15);

    return terrain->initialize() &&
        agents->initialize() &&
        ui->initialize() &&
        pather->initialize();
}

bool ProjectThree::finalize()
{
    player = agents->create_pathing_agent();
    player->set_debug_coloring(false);
    player->set_single_step(false);
    player->set_color(Vec3(0.0f, 0.0f, 0.8f));

    enemy = agents->create_enemy_agent();
    enemy->set_debug_coloring(false);
    enemy->set_single_step(false);
    enemy->set_color(Vec3(0.8f, 0.0f, 0.0f));
    enemy->set_player(player);

    set_propagation_growth(0.15f);
    set_propagation_decay(0.05f);

    build_ui();

    terrain->goto_map(0);
    terrain->pathLayer.set_enabled(true);

    // set up the time tracker to listen for analysis messages
    Callback timerResetCB = std::bind(&decltype(analysisTimer)::reset, &analysisTimer);
    Messenger::listen_for_message(Messages::ANALYSIS_BEGIN, timerResetCB);

    Callback timerStartCB = std::bind(&decltype(analysisTimer)::start, &analysisTimer);
    Messenger::listen_for_message(Messages::ANALYSIS_TICK_START, timerStartCB);

    Callback timerStopCB = std::bind(&decltype(analysisTimer)::stop, &analysisTimer);
    Messenger::listen_for_message(Messages::ANALYSIS_TICK_FINISH, timerStopCB);

    link_input();

    return true;
}

void ProjectThree::shutdown()
{
    std::cout << "Shutting Down Project Three..." << std::endl;

    ui->shutdown();
    ui.reset();

    agents->shutdown();
    agents.reset();

    terrain->shutdown();
    terrain.reset();
}

void ProjectThree::draw_meshes()
{
    terrain->draw();
    agents->draw();
}

void ProjectThree::draw_sprites()
{
    ui->draw_sprites();
}

void ProjectThree::draw_text()
{
    ui->draw_text();
}

void ProjectThree::draw_debug()
{
    terrain->draw_debug();
    agents->draw_debug();
    ui->draw_debug();
}

void ProjectThree::update()
{
    // have the ui coordinator determine its state
    ui->update();

    // have the input system update its current state and send out notifications
    InputHandler::update();

    agents->update(deltaTime);

    // use a frame offset to stagger the execution of various analysis functions
    auto frameDelay = engine->get_timer().GetFrameCount() % frequencyMod;

    if (playerVisibility == true && frameDelay == 0)
    {
        perform_player_visibility_analysis();
    }

    if (search == true)
    {
        auto decayOp = [](float &val)
        {
            val *= 0.999f;
        };
        terrain->agentVisionLayer.for_each(decayOp);

        perform_search_analysis();
    }

    if (propagation == true && frameDelay == (frequencyOffset * 2))
    {
        perform_propagation();
    }
    else if (propagationNormalized == true && frameDelay == (frequencyOffset * 2))
    {
        perform_propagation_normalized();
    }

    if (hideAndSeek == true)
    {
        if (frameDelay == (frequencyOffset * 3))
        {
            propagate_solo_occupancy(terrain->seekLayer, propagationDecay, propagationGrowth);
        }

        perform_hide_and_seek();
    }
}

unsigned ProjectThree::get_analysis_frequency()
{
    return analysisFrequency;
}

void ProjectThree::set_analysis_frequency(const unsigned &val)
{
    analysisFrequency = val;
    analysisFrequencyText = std::to_wstring(analysisFrequency);

    frequencyMod = 60 / analysisFrequency;
    frequencyOffset = frequencyMod / 4;
}

const std::wstring &ProjectThree::get_analysis_frequency_text()
{
    return analysisFrequencyText;
}

float ProjectThree::get_propagation_decay()
{
    return propagationDecay;
}

void ProjectThree::set_propagation_decay(const float &val)
{
    propagationDecay = val;
    std::wstringstream stream;
    stream.precision(3);
    stream << propagationDecay;
    propagationDecayText = stream.str();
}

const std::wstring &ProjectThree::get_propagation_decay_text()
{
    return propagationDecayText;
}

float ProjectThree::get_propagation_growth()
{
    return propagationGrowth;
}

void ProjectThree::set_propagation_growth(const float &val)
{
    propagationGrowth = val;
    std::wstringstream stream;
    stream.precision(3);
    stream << propagationGrowth;
    propagationGrowthText = stream.str();
}

const std::wstring &ProjectThree::get_propagation_growth_text()
{
    return propagationGrowthText;
}

void ProjectThree::build_ui()
{
    // add a text field at the top for the project
    auto projectText = ui->create_text_field(UIAnchor::TOP, 0, 32, UIAnchor::CENTER, L"Project Three");

    // create the first button in the top right,
    // 90 is half button width + 16 (padding from side of screen) + 10 (padding so sliders aren't directly against the edge)
    // 32 is half button height + 16 (padding from top of screen)
    Callback nextMapCB = std::bind(&Terrain::goto_next_map, terrain.get());
    auto mapButton = ui->create_button(UIAnchor::TOP_RIGHT, -90, 32, nextMapCB, L"Next Map");

    // setup the analysis buttons
    Callback opennessCB = std::bind(&ProjectThree::toggle_openness, this);
    Getter<bool> opennessGet = std::bind(&ProjectThree::get_openness_state, this);
    auto opennessButton = ui->create_toggle_button(UIAnchor::BOTTOM, mapButton,
        10, opennessCB, L"Openness", opennessGet);

    Callback totalVisCB = std::bind(&ProjectThree::toggle_total_visibility, this);
    Getter<bool> totalVisGet = std::bind(&ProjectThree::get_total_visibility_state, this);
    auto totalVisButton = ui->create_toggle_button(UIAnchor::BOTTOM, opennessButton,
        10, totalVisCB, L"Visibility", totalVisGet);

    Callback playerVisCB = std::bind(&ProjectThree::toggle_player_visibility, this);
    Getter<bool> playerVisGet = std::bind(&ProjectThree::get_player_visibility_state, this);
    auto playerVisButton = ui->create_toggle_button(UIAnchor::BOTTOM, totalVisButton,
        10, playerVisCB, L"Visible To Agent", playerVisGet);

    Callback searchCB = std::bind(&ProjectThree::toggle_search, this);
    Getter<bool> searchGet = std::bind(&ProjectThree::get_search_state, this);
    auto searchButton = ui->create_toggle_button(UIAnchor::BOTTOM, playerVisButton,
        10, searchCB, L"Search", searchGet);

    Callback propagationCB = std::bind(&ProjectThree::toggle_propagation, this);
    Getter<bool> propagationGet = std::bind(&ProjectThree::get_propagation_state, this);
    auto propagationButton = ui->create_toggle_button(UIAnchor::BOTTOM, searchButton,
        10, propagationCB, L"Propagation", propagationGet);

    Callback normalizedCB = std::bind(&ProjectThree::toggle_propagation_normalized, this);
    Getter<bool> normalizedGet = std::bind(&ProjectThree::get_propagation_normalized_state, this);
    auto normalizedButton = ui->create_toggle_button(UIAnchor::BOTTOM, propagationButton,
        10, normalizedCB, L"Normalized Prop.", normalizedGet);

    Callback dualCB = std::bind(&ProjectThree::toggle_propagation_dual, this);
    Getter<bool> dualGet = std::bind(&ProjectThree::get_propagation_dual_state, this);
    auto dualButton = ui->create_toggle_button(UIAnchor::BOTTOM, normalizedButton,
        10, dualCB, L"Dual Prop.", dualGet);

    Callback hideCB = std::bind(&ProjectThree::toggle_hide_and_seek, this);
    Getter<bool> hideGet = std::bind(&ProjectThree::get_hide_and_seek_state, this);
    auto hideButton = ui->create_toggle_button(UIAnchor::BOTTOM, dualButton,
        10, hideCB, L"Hide and Seek", hideGet);

    // add some text on the left side for displaying fps
    TextGetter fpsGetter = std::bind(&Engine::get_fps_text, engine.get());
    auto fpsText = ui->create_value_text_field(UIAnchor::TOP_LEFT, 90, 32, L"FPS:", fpsGetter);

    // and the pathing time
    TextGetter timeGetter = std::bind(&decltype(analysisTimer)::get_text, &analysisTimer);
    auto analysisTimeText = ui->create_value_text_field(UIAnchor::BOTTOM, fpsText, 10, L"Time:", timeGetter);

    // a slider to control how often analysis happens
    Getter<unsigned> frequencyGet = std::bind(&ProjectThree::get_analysis_frequency, this);
    Setter<unsigned> frequencySet = std::bind(&ProjectThree::set_analysis_frequency, this, std::placeholders::_1);
    TextGetter frequencyText = std::bind(&ProjectThree::get_analysis_frequency_text, this);
    auto frequencySlider = ui->create_slider<unsigned>(UIAnchor::BOTTOM, analysisTimeText, 10,
        1, 20, frequencyGet, frequencySet, frequencyText, L"Ticks Per Sec:");

    Getter<float> decayGet = std::bind(&ProjectThree::get_propagation_decay, this);
    Setter<float> decaySet = std::bind(&ProjectThree::set_propagation_decay, this, std::placeholders::_1);
    TextGetter decayText = std::bind(&ProjectThree::get_propagation_decay_text, this);
    auto decaySlider = ui->create_slider<float>(UIAnchor::BOTTOM, frequencySlider,
        10, 0.0f, 0.1f, decayGet, decaySet, decayText, L"Decay");

    Getter<float> growthGet = std::bind(&ProjectThree::get_propagation_growth, this);
    Setter<float> growthSet = std::bind(&ProjectThree::set_propagation_growth, this, std::placeholders::_1);
    TextGetter growthText = std::bind(&ProjectThree::get_propagation_growth_text, this);
    auto growthSlider = ui->create_slider<float>(UIAnchor::BOTTOM, decaySlider,
        10, 0.0f, 1.0f, growthGet, growthSet, growthText, L"Growth");

    Getter<float> fovGet = std::bind(&EnemyAgent::get_fov, enemy);
    Setter<float> fovSet = std::bind(&EnemyAgent::set_fov, enemy, std::placeholders::_1);
    TextGetter fovText = std::bind(&EnemyAgent::get_fov_text, enemy);
    auto fovSlider = ui->create_slider<float>(UIAnchor::BOTTOM, growthSlider,
        10, 10.0f, 180.0f, fovGet, fovSet, fovText, L"Enemy FOV");

    Getter<float> radiusGet = std::bind(&EnemyAgent::get_radius, enemy);
    Setter<float> radiusSet = std::bind(&EnemyAgent::set_radius, enemy, std::placeholders::_1);
    TextGetter radiusText = std::bind(&EnemyAgent::get_radius_text, enemy);
    auto radiusSlider = ui->create_slider<float>(UIAnchor::BOTTOM, fovSlider,
        10, 1.0f, 10.0f, radiusGet, radiusSet, radiusText, L"Enemy Dist.");
}

void ProjectThree::link_input()
{
    Callback f1CB = std::bind(&ProjectThree::on_f1, this);
    InputHandler::notify_when_key_pressed(KBKeys::F1, f1CB);

    Callback f2CB = std::bind(&ProjectThree::on_f2, this);
    InputHandler::notify_when_key_pressed(KBKeys::F2, f2CB);

    Callback lmbCB = std::bind(&ProjectThree::on_mouse_left_click, this);
    InputHandler::notify_when_mouse_pressed(MouseButtons::LEFT, lmbCB);

    Callback rmbCB = std::bind(&ProjectThree::on_mouse_right_click, this);
    InputHandler::notify_when_mouse_pressed(MouseButtons::RIGHT, rmbCB);
}

void ProjectThree::on_f2()
{
    engine->change_projects(Project::Type::TWO);
}

void ProjectThree::on_mouse_left_click()
{
    const auto &mousePos = InputHandler::get_mouse_position();

    // convert the mouse position to a point on the terrain plane
    const auto worldPos = renderer->screen_to_world(mousePos.x, mousePos.y, terrain->get_terrain_plane());

    // verify a valid point was determine
    if (worldPos.second == true)
    {
        // convert the world position to a terraing grid position
        const auto gridPos = terrain->get_grid_position(worldPos.first);

        // verify that the grid position is valid
        if (terrain->is_valid_grid_position(gridPos) == true && terrain->is_wall(gridPos) == false)
        {
            if (propagation || propagationNormalized)
            {
                terrain->occupancyLayer.set_value(gridPos, 1.0f);
            }
            else
            {
                // have the agent path to the position
                player->path_to(worldPos.first);
            }
        }
    }
}

void ProjectThree::on_mouse_right_click()
{
    if (!propagationDual)
    {
        return;
    }

    const auto &mousePos = InputHandler::get_mouse_position();

    // convert the mouse position to a point on the terrain plane
    const auto worldPos = renderer->screen_to_world(mousePos.x, mousePos.y, terrain->get_terrain_plane());

    // verify a valid point was determine
    if (worldPos.second == true)
    {
        // convert the world position to a terraing grid position
        const auto gridPos = terrain->get_grid_position(worldPos.first);

        // verify that the grid position is valid
        if (terrain->is_valid_grid_position(gridPos) == true && terrain->is_wall(gridPos) == false)
        {
            if (propagation || propagationNormalized)
            {
                terrain->occupancyLayer.set_value(gridPos, -1.0f);
            }
            else
            {
                // have the enemy path to the position
                enemy->path_to(worldPos.first);
            }
        }
    }
}

void ProjectThree::toggle_openness()
{
    openness = !openness;

    if (openness == true)
    {
        
        perform_openness_analysis();
    }

    terrain->opennessLayer.set_enabled(openness);
}

void ProjectThree::toggle_total_visibility()
{
    totalVisibility = !totalVisibility;

    if (totalVisibility == true)
    {
        perform_total_visibility_analysis();
    }

    terrain->totalVisibilityLayer.set_enabled(totalVisibility);
}

void ProjectThree::toggle_player_visibility()
{
    playerVisibility = !playerVisibility;

    terrain->cellVisibilityLayer.set_enabled(playerVisibility);
}

void ProjectThree::toggle_search()
{
    search = !search;

    terrain->agentVisionLayer.set_enabled(search);
}

void ProjectThree::toggle_propagation()
{
    propagation = !propagation;

    // propagation and normalized utilize the same layer
    if (propagation && propagationNormalized)
    {
        // don't clear, just disable the other
        propagationNormalized = false;
    }
    else
    {
        terrain->occupancyLayer.for_each([](float &v) { v = 0.0f; });
    }

    terrain->occupancyLayer.set_enabled(propagation);
}

void ProjectThree::toggle_propagation_normalized()
{
    propagationNormalized = !propagationNormalized;

    // propagation and normalized utilize the same layer
    if (propagation && propagationNormalized)
    {
        // don't clear, just disable the other
        propagation = false;
    }
    else
    {
        terrain->occupancyLayer.for_each([](float &v) { v = 0.0f; });
    }

    terrain->occupancyLayer.set_enabled(propagationNormalized);
}

void ProjectThree::toggle_propagation_dual()
{
    propagationDual = !propagationDual;
}

void ProjectThree::toggle_hide_and_seek()
{
    hideAndSeek = !hideAndSeek;

    if (hideAndSeek == true)
    {
        terrain->seekLayer.for_each([](float &v) { v = 0.0f; });
    }

    terrain->seekLayer.set_enabled(hideAndSeek);
}

void ProjectThree::perform_openness_analysis()
{
    Messenger::send_message(Messages::ANALYSIS_BEGIN);
    Messenger::send_message(Messages::ANALYSIS_TICK_START);

    analyze_openness(terrain->opennessLayer);

    Messenger::send_message(Messages::ANALYSIS_TICK_FINISH);
    Messenger::send_message(Messages::ANALYSIS_END);
}

void ProjectThree::perform_total_visibility_analysis()
{
    Messenger::send_message(Messages::ANALYSIS_BEGIN);
    Messenger::send_message(Messages::ANALYSIS_TICK_START);

    analyze_visibility(terrain->totalVisibilityLayer);

    Messenger::send_message(Messages::ANALYSIS_TICK_FINISH);
    Messenger::send_message(Messages::ANALYSIS_END);
}

void ProjectThree::perform_player_visibility_analysis()
{
    const auto pos = terrain->get_grid_position(player->get_position());

    if (terrain->is_valid_grid_position(pos) == true)
    {
        Messenger::send_message(Messages::ANALYSIS_BEGIN);
        Messenger::send_message(Messages::ANALYSIS_TICK_START);

        analyze_visible_to_cell(terrain->cellVisibilityLayer, pos.row, pos.col);

        Messenger::send_message(Messages::ANALYSIS_TICK_FINISH);
        Messenger::send_message(Messages::ANALYSIS_END);
    }
}

void ProjectThree::perform_search_analysis()
{
    Messenger::send_message(Messages::ANALYSIS_BEGIN);
    Messenger::send_message(Messages::ANALYSIS_TICK_START);

    analyze_agent_vision(terrain->agentVisionLayer, player);

    Messenger::send_message(Messages::ANALYSIS_TICK_FINISH);
    Messenger::send_message(Messages::ANALYSIS_END);
}

void ProjectThree::perform_propagation()
{
    Messenger::send_message(Messages::ANALYSIS_BEGIN);
    Messenger::send_message(Messages::ANALYSIS_TICK_START);

    if (propagationDual)
    {
        propagate_dual_occupancy(terrain->occupancyLayer, propagationDecay, propagationGrowth);
    }
    else
    {
        propagate_solo_occupancy(terrain->occupancyLayer, propagationDecay, propagationGrowth);
    }

    Messenger::send_message(Messages::ANALYSIS_TICK_FINISH);
    Messenger::send_message(Messages::ANALYSIS_END);
}

void ProjectThree::perform_propagation_normalized()
{
    Messenger::send_message(Messages::ANALYSIS_BEGIN);
    Messenger::send_message(Messages::ANALYSIS_TICK_START);

    if (propagationDual)
    {
        propagate_dual_occupancy(terrain->occupancyLayer, propagationDecay, propagationGrowth);
        normalize_dual_occupancy(terrain->occupancyLayer);
    }
    else
    {
        propagate_solo_occupancy(terrain->occupancyLayer, propagationDecay, propagationGrowth);
        normalize_solo_occupancy(terrain->occupancyLayer);
    }

    Messenger::send_message(Messages::ANALYSIS_TICK_FINISH);
    Messenger::send_message(Messages::ANALYSIS_END);
}

void ProjectThree::perform_hide_and_seek()
{
    Messenger::send_message(Messages::ANALYSIS_BEGIN);
    Messenger::send_message(Messages::ANALYSIS_TICK_START);

    if (enemy->logic_tick())
    {
      // run propagation a few times to make sure it expands before being removed by FOV
      for (int i = 0; i < 3; ++i)
        propagate_solo_occupancy(terrain->seekLayer, propagationDecay, propagationGrowth);
    }

    Messenger::send_message(Messages::ANALYSIS_TICK_FINISH);
    Messenger::send_message(Messages::ANALYSIS_END);
}

bool ProjectThree::get_openness_state()
{
    return openness;
}

bool ProjectThree::get_total_visibility_state()
{
    return totalVisibility;
}

bool ProjectThree::get_player_visibility_state()
{
    return playerVisibility;
}

bool ProjectThree::get_search_state()
{
    return search;
}

bool ProjectThree::get_propagation_state()
{
    return propagation;
}

bool ProjectThree::get_propagation_normalized_state()
{
    return propagationNormalized;
}

bool ProjectThree::get_propagation_dual_state()
{
    return propagationDual;
}

bool ProjectThree::get_hide_and_seek_state()
{
    return hideAndSeek;
}

void ProjectThree::on_f1()
{
    engine->change_projects(Project::Type::ONE);
}
