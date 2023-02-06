/******************************************************************************/
/*!
\file		BehaviorAgent.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Agent that uses behavior trees implementation

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "BehaviorAgent.h"

namespace
{
    const float movementEpsilon = 0.1f;
    const std::wstring debugTextDelimiter(L", ");
    const std::wstring debugNameSeparator(L"_");
}

BehaviorAgent::BehaviorAgent(const char *type, size_t id) : Agent(type, id)
{
    const std::string temp(type);
    debugName = std::wstring(temp.begin(), temp.end()) + debugNameSeparator + std::to_wstring(id);
    set_movement_speed(10.0f);
}

Blackboard &BehaviorAgent::get_blackboard()
{
    return blackboard;
}

BehaviorTree &BehaviorAgent::get_behavior_tree()
{
    return tree;
}

void BehaviorAgent::update(float dt)
{
    tree.update(dt);
}

bool BehaviorAgent::move_toward_point(const Vec3 &point, float dt)
{
    bool result = false;

    const auto currentPos = get_position();

    auto delta = point - currentPos;

    const float length = delta.Length();

    // see if we're close enough to the goal
    if (length <= movementEpsilon)
    {
        // no need to actually move
        result = true;
    }
    else
    {
        // determine how far to actually move
        float actualSpeed = get_movement_speed() * dt;
        
        // see if we even need to move the full distance
        if (length < actualSpeed)
        {
            // this move will put us within range of the target point
            result = true;
            actualSpeed = length;
        }

        delta.Normalize();
        delta *= actualSpeed;

        auto nextPos = currentPos + delta;

        nextPos.x = std::max(0.f, nextPos.x);
        nextPos.x = std::min(100.f, nextPos.x);
        nextPos.z = std::max(0.f, nextPos.z);
        nextPos.z = std::min(100.f, nextPos.z);
        nextPos.y = 0.f;

        set_position(nextPos);


        const float yaw = std::atan2(delta.x, delta.z);
        set_yaw(yaw);
    }

    return result;
}

const std::wstring &BehaviorAgent::get_debug_name() const
{
    return debugName;
}

std::wstringstream &BehaviorAgent::get_debug_text()
{
    return debugText;
}

void BehaviorAgent::add_debug_text(const std::wstring &nodeName)
{
    debugText << nodeName << debugTextDelimiter;
}
