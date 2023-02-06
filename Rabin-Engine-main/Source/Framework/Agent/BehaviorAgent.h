/******************************************************************************/
/*!
\file		BehaviorAgent.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Agent that uses behavior trees declarations

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include "Agent.h"
#include "../BehaviorTrees/BehaviorTree.h"
#include "../BehaviorTrees/Blackboard.h"
#include <sstream>

class BehaviorAgent final : public Agent
{
public:
    BehaviorAgent(const char *type, size_t id);

    Blackboard &get_blackboard();
    BehaviorTree &get_behavior_tree();

    virtual void update(float dt) override;

    // returns whether or not the point has been reached
    bool move_toward_point(const Vec3 &point, float dt);

    const std::wstring &get_debug_name() const;
    std::wstringstream &get_debug_text();
    void add_debug_text(const std::wstring &nodeName);

private:
    BehaviorTree tree;
    Blackboard blackboard;
    std::wstring debugName;
    std::wstringstream debugText;
};

