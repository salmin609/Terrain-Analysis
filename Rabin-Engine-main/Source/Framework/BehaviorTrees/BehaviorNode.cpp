/******************************************************************************/
/*!
\file		BehaviorNode.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Base behavior tree node implementation

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "BehaviorNode.h"
#include "Agent/BehaviorAgent.h"

void BehaviorNode::set_debug_info(const char *n, const char *s)
{
    name = n;
    summary = s;
    
    const std::string temp(n);
    wideName = std::wstring(temp.begin(), temp.end());
}

void BehaviorNode::add_child(BehaviorNode *child)
{
    children.emplace_back(child);
    child->parent = this;
    child->agent = agent;
}

BehaviorNode::BehaviorNode() : agent(nullptr), status(NodeStatus::READY), result(NodeResult::IN_PROGRESS), parent(nullptr)
{}

BehaviorNode::~BehaviorNode()
{
    for (auto && child : children)
    {
        delete child;
    }
}

bool BehaviorNode::is_ready() const
{
    return status == NodeStatus::READY;
}

bool BehaviorNode::succeeded() const
{
    return result == NodeResult::SUCCESS;
}

bool BehaviorNode::failed() const
{
    return result == NodeResult::FAILURE;
}

bool BehaviorNode::is_running() const
{
    return status == NodeStatus::RUNNING;
}

bool BehaviorNode::is_suspended() const
{
    return status == NodeStatus::SUSPENDED;
}

void BehaviorNode::set_status(NodeStatus newStatus)
{
    status = newStatus;
}

void BehaviorNode::set_status_all(NodeStatus newStatus)
{
    status = newStatus;

    for (auto && child : children)
    {
        child->set_status_all(newStatus);
    }
}

void BehaviorNode::set_status_children(NodeStatus newStatus)
{
    for (auto && child : children)
    {
        child->set_status(newStatus);
    }
}

void BehaviorNode::set_result(NodeResult r)
{
    result = r;
}

void BehaviorNode::set_result_children(NodeResult result)
{
    for (auto && child : children)
    {
        child->set_result(result);
    }
}

NodeStatus BehaviorNode::get_status() const
{
    return status;
}

NodeResult BehaviorNode::get_result() const
{
    return result;
}

void BehaviorNode::tick(float dt)
{
    // explicitly check the statuses in order, so a node can transition fully through its states in one frame if needed

    if (status == NodeStatus::READY)
    {
        on_enter();
    }

    if (status == NodeStatus::RUNNING)
    {
        on_update(dt);
    }

    if (status == NodeStatus::EXITING)
    {
        on_exit();
    }
}


const char *BehaviorNode::get_name() const
{
    return name;
}


const char *BehaviorNode::get_summary() const
{
    return summary;
}

void BehaviorNode::on_enter()
{
    // base logic is to mark as running
    set_status(NodeStatus::RUNNING);
    set_result(NodeResult::IN_PROGRESS);

    // and this node's children as ready to run
    set_status_children(NodeStatus::READY);
    set_result_children(NodeResult::IN_PROGRESS);
}

void BehaviorNode::on_leaf_enter()
{
	set_status(NodeStatus::RUNNING);
	set_result(NodeResult::IN_PROGRESS);
}


void BehaviorNode::on_update(float)
{
    // no base logic that makes sense for every node
}

void BehaviorNode::on_exit()
{
    // base logic is to mark the node as done executing
    set_status(NodeStatus::SUSPENDED);
}

void BehaviorNode::on_success()
{
    set_status(NodeStatus::EXITING);
    set_result(NodeResult::SUCCESS);
}

void BehaviorNode::on_failure()
{
    set_status(NodeStatus::EXITING);
    set_result(NodeResult::FAILURE);
}

void BehaviorNode::display_leaf_text()
{
    #ifdef _DEBUG
        agent->add_debug_text(wideName);
    #endif
}
