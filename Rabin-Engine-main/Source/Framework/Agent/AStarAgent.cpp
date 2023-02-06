/******************************************************************************/
/*!
\file		AStarAgent.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Agent that can pathfind implementation

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "AStarAgent.h"
#include <sstream>
#include "Projects/ProjectTwo.h"


const char *AStarAgent::patherTypeName("A* Agent");

std::set<Method> availableMethods;

const Color pathColor(1.0f, 0.0f, 0.0f, 1.0f);

AStarAgent::AStarAgent(size_t id) : Agent(patherTypeName, id), computingPath(false), movement(Movement::WALK)
{
    buffer.settings.heuristic = Heuristic::OCTILE;
    buffer.settings.weight = 1.0f;
    buffer.settings.smoothing = false;
    buffer.settings.rubberBanding = false;
    buffer.settings.singleStep = true;
    buffer.settings.debugColoring = true;
    buffer.settings.method = Method::ASTAR;

    set_heuristic_weight(1.0f);

    availableMethods.emplace(Method::ASTAR);

    if (ProjectTwo::implemented_floyd_warshall() == true)
    {
        availableMethods.emplace(Method::FLOYD_WARSHALL);
    }

    if (ProjectTwo::implemented_jps_plus() == true)
    {
        availableMethods.emplace(Method::JPS_PLUS);
    }

    if (ProjectTwo::implemented_goal_bounding() == true)
    {
        availableMethods.emplace(Method::GOAL_BOUNDING);
    }

    Callback mapCallback = std::bind(&AStarAgent::on_map_change, this);
    Messenger::listen_for_message(Messages::MAP_CHANGE, mapCallback);
}

void AStarAgent::update(float dt)
{   
    switch (movement)
    {
    case Movement::NONE:
      if (computingPath)
      {
        request.newRequest = false;

        process_request();
      }
      break;
    case Movement::WALK:
        if (computingPath == false)
        {
            if (request.path.size() > 0)
            {
                const auto &currPos = get_position();
                auto delta = (request.path.front() - currPos);

                if (delta.LengthSquared() <= 0.1f)
                {
                    request.path.pop_front();
                }
                else
                {
                    float lenSq = delta.LengthSquared();
                    float dist = get_movement_speed() * dt / static_cast<float>(terrain->get_map_width());
                    float distSq = dist * dist;
                    const auto movement = distSq > lenSq ? delta : dist * delta / std::sqrt(lenSq);

                    set_position(currPos + movement);

                    const float yaw = std::atan2(delta.x, delta.z);
                    set_yaw(yaw);
                }
            }
        }
        else
        {
            request.newRequest = false;
        
            process_request();
        }
        break;
    case Movement::TELEPORT:
        set_position(terrain->get_world_position(terrain->get_grid_position(request.goal)));
        break;
    }
}

void AStarAgent::path_to(const Vec3 &point, bool timed)
{
    const auto gridPos = terrain->get_grid_position(point);

    if (terrain->is_valid_grid_position(gridPos.row, gridPos.col) == true)
    {
        request.path.clear();
        
        request.start = get_position();
        request.goal = point;
        request.settings = buffer.settings;
        request.newRequest = true;

        if (movement != Movement::TELEPORT)
        {
            if (timed == true)
            {
                Messenger::send_message(Messages::PATH_REQUEST_BEGIN);
            }

            process_request();
        }
    }
}

void AStarAgent::next_heuristic_type()
{
    int next = (static_cast<int>(buffer.settings.heuristic) + 1) % static_cast<int>(Heuristic::NUM_ENTRIES);

    set_heuristic_type(static_cast<Heuristic>(next));
}

Heuristic AStarAgent::get_heuristic_type()
{
    return buffer.settings.heuristic;
}

void AStarAgent::set_heuristic_type(const Heuristic &type)
{
    buffer.settings.heuristic = type;
}

const std::wstring &AStarAgent::get_heuristic_type_text()
{
    return get_heuristic_wtext(buffer.settings.heuristic);
}

float AStarAgent::get_heuristic_weight()
{
    return buffer.settings.weight;
}

void AStarAgent::set_heuristic_weight(const float &weight)
{
    buffer.settings.weight = weight;
    std::wstringstream stream;
    stream.precision(3);
    stream << weight;
    heuristicWeightText = stream.str();
}

const std::wstring &AStarAgent::get_heuristic_weight_text()
{
    return heuristicWeightText;
}

void AStarAgent::toggle_smoothing()
{
    set_smoothing(!buffer.settings.smoothing);
}

bool AStarAgent::get_smoothing()
{
    return buffer.settings.smoothing;
}

void AStarAgent::set_smoothing(bool state)
{
    buffer.settings.smoothing = state;
}

void AStarAgent::toggle_rubberbanding()
{
    set_rubberbanding(!buffer.settings.rubberBanding);
}

bool AStarAgent::get_rubberbanding()
{
    return buffer.settings.rubberBanding;
}

void AStarAgent::set_rubberbanding(bool state)
{
    buffer.settings.rubberBanding = state;
}

void AStarAgent::toggle_single_step()
{
    set_single_step(!buffer.settings.singleStep);
}

bool AStarAgent::get_single_step()
{
    return buffer.settings.singleStep;
}

void AStarAgent::set_single_step(bool state)
{
    buffer.settings.singleStep = state;
}

void AStarAgent::next_movement_type()
{
  int next = (static_cast<int>(movement) + 1) % static_cast<int>(Movement::NUM_ENTRIES);

  set_movement_type(static_cast<Movement>(next));
}

Movement AStarAgent::get_movement_type()
{
    return movement;
}

void AStarAgent::set_movement_type(Movement type)
{
    movement = type;
}

const std::wstring &AStarAgent::get_movement_type_text()
{
  static const std::wstring text[] = { L"No Movement", L"Walk", L"Teleport" };

  return text[static_cast<size_t>(movement)];
}

void AStarAgent::next_method_type()
{
    const auto next = std::next(availableMethods.find(buffer.settings.method));

    set_method_type(
      *(next == availableMethods.end() ? availableMethods.begin() : next)
    );
}

Method AStarAgent::get_method_type()
{
    return buffer.settings.method;
}

void AStarAgent::set_method_type(Method type)
{
    buffer.settings.method = type;
}

const std::wstring &AStarAgent::get_method_text()
{
    return ::get_method_wtext(buffer.settings.method);
}

void AStarAgent::toggle_debug_coloring()
{
    set_debug_coloring(!buffer.settings.debugColoring);
}

bool AStarAgent::get_debug_coloring()
{
    return buffer.settings.debugColoring;
}

void AStarAgent::set_debug_coloring(bool state)
{
    buffer.settings.debugColoring = state;
}

void AStarAgent::draw_debug()
{
    if (request.path.size() >= 2)
    {
        auto p0 = request.path.begin();
        auto p1 = p0;
        ++p1;

        auto &dr = renderer->get_debug_renderer();

        while (p1 != request.path.end())
        {
            dr.draw_arrow(*p1, *p0, pathColor);

            p0 = p1;
            ++p1;
        }
    }
}

const PathRequest &AStarAgent::get_request_data() const
{
    return request;
}

void AStarAgent::bulk_set_request_settings(const PathRequest::Settings &settings)
{
    buffer.settings = settings;
}

void AStarAgent::process_request()
{
    Messenger::send_message(Messages::PATH_REQUEST_TICK_START);
    const auto result = pather->compute_path(request);
    Messenger::send_message(Messages::PATH_REQUEST_TICK_FINISH);

    switch (result)
    {
    case PathResult::COMPLETE:
    case PathResult::IMPOSSIBLE:
        computingPath = false;
        Messenger::send_message(Messages::PATH_REQUEST_END);
        break;
    case PathResult::PROCESSING:
        computingPath = true;
        break;
    }
}

void AStarAgent::on_map_change()
{
    computingPath = false;
    request.path.clear();
}
