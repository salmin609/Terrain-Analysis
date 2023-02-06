/******************************************************************************/
/*!
\file		EnemyAgent.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Agent capable of searching for another agent implementation

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "EnemyAgent.h"
#include "Terrain\TerrainAnalysis.h"
#include <sstream>

EnemyAgent::EnemyAgent(size_t id) : AStarAgent(id), state(State::IDLE), player(nullptr), playerPrevious{-1, -1}, timer(0.0f)
{
    set_fov(180.0f);
    set_radius(2.0f);
    set_movement_speed(movementSpeed);
}

void EnemyAgent::path_to(const Vec3 &point)
{
    AStarAgent::path_to(point, false);

    // remove the first point
    if (request.path.size() > 0)
    {
        request.path.pop_front();
    }
}

bool EnemyAgent::logic_tick()
{
    // update the detection area
    enemy_field_of_view(terrain->seekLayer, fov, radius, state == State::PATROL ? -0.25f : -0.5f, this);
    normalize_solo_occupancy(terrain->seekLayer);

    // see if the player is within the detection area
    if (enemy_find_player(terrain->seekLayer, this, player) == true)
    {
        if (state != State::CHASE)
        {
            state = State::CHASE;
            terrain->seekLayer.for_each([](float &v) { v = std::min(v, 0.0f); });
        }

        const auto playerWorld = player->get_position();
        const auto playerGrid = terrain->get_grid_position(playerWorld);

        if (playerGrid != playerPrevious)
        {
            set_movement_speed(movementSpeed * repathFactor);
            playerPrevious = playerGrid;

            // we know where the player currently is, remove all old possible search locations
            auto op = [](float &val)
            {
                if (val > 0.0f)
                {
                    val = 0.0f;
                }
            };
            terrain->seekLayer.for_each(op);

            // and mark the player's current position
            terrain->seekLayer.set_value(playerGrid, 1.0f);

            // and path there
            path_to(playerWorld);

            return true;
        }
    }

    switch (state)
    {
    case State::IDLE:
        set_movement_speed(movementSpeed);
        // see if there is a position it makes sense to look for the player
        if (enemy_seek_player(terrain->seekLayer, this) == true)
        {
            state = State::SEEK;
        }
        else
        {
            state = State::PATROL;
            update_timer(0.0f);
        }
        break;

    case State::SEEK:
        set_movement_speed(movementSpeed);
        [[fallthrough]];
    case State::CHASE:
        if (request.path.size() == 0)
        {
            state = State::IDLE;
        }

        break;

    case State::PATROL:
        if (update_timer(reactTimeIdle))
        {
            if (request.path.size() == 0)
            {
                choose_random_goal();
                update_timer(0.0f);
            }
        }
        break;
    }

    return false;
}

float EnemyAgent::get_fov()
{
    return fov;
}

void EnemyAgent::set_fov(const float &val)
{
    fov = val;
    std::wstringstream stream;
    stream.precision(3);
    stream << fov;
    fovText = stream.str();
}

const std::wstring &EnemyAgent::get_fov_text()
{
    return fovText;
}

float EnemyAgent::get_radius()
{
    return radius;
}

void EnemyAgent::set_radius(const float &val)
{
    radius = val;
    std::wstringstream stream;
    stream.precision(3);
    stream << radius;
    radiusText = stream.str();
}

const std::wstring &EnemyAgent::get_radius_text()
{
    return radiusText;
}

void EnemyAgent::set_player(AStarAgent *p)
{
    player = p;
}

void EnemyAgent::choose_random_goal()
{
    const auto worldPos = get_position();
    const auto gridPos = terrain->get_grid_position(worldPos);

    GridPos target;

    const int minRow = std::max(gridPos.row - 5, 0);
    const int maxRow = std::min(gridPos.row + 5, terrain->get_map_height());
    const int minCol = std::max(gridPos.col - 5, 0);
    const int maxCol = std::min(gridPos.col + 5, terrain->get_map_width());

    while (true)
    {
        target.row = RNG::range(minRow, maxRow);
        target.col = RNG::range(minCol, maxCol);

        if (terrain->is_valid_grid_position(target) == true && terrain->is_wall(target) == false)
        {
            break;
        }
    }

    const auto worldTarget = terrain->get_world_position(target);
    path_to(worldTarget);
}

bool EnemyAgent::update_timer(float time)
{
  if (timer > time)
  {
    timer = 0.0f;
    return true;
  }
  timer += deltaTime;
  return false;
}
