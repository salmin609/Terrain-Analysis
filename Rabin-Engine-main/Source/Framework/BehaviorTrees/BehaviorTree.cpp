/******************************************************************************/
/*!
\file		BehaviorTree.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Behavior tree implementation

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "BehaviorTree.h"
#include "BehaviorNode.h"

BehaviorTree::BehaviorTree() : rootNode(nullptr), treeName(nullptr)
{}

BehaviorTree::~BehaviorTree()
{
    delete rootNode;
}

void BehaviorTree::update(float dt)
{
    rootNode->tick(dt);

    // if the root node is complete in any way, restart it
    if (rootNode->is_running() == false)
    {
        rootNode->set_status(NodeStatus::READY);
    }
}

const char *BehaviorTree::get_tree_name() const
{
    return treeName;
}
