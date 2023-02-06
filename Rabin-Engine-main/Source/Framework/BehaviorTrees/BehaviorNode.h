/******************************************************************************/
/*!
\file		BehaviorNode.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Base behavior tree node declaration

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include <vector>

class BehaviorAgent;
class BehaviorTreePrototype;

enum class NodeStatus
{
    READY, // node is enterable
    RUNNING, // node is currently running
    EXITING, // node has succeeded or failed
    SUSPENDED // node won't exceute anything
};

enum class NodeResult
{
    IN_PROGRESS, // still being run 
    SUCCESS, // node succeeded
    FAILURE // node failed
};

class BehaviorNode
{
    friend class BehaviorTreePrototype;

public:
    BehaviorNode();
    ~BehaviorNode();

    // readability status getters
    bool is_ready() const;
    bool succeeded() const;
    bool failed() const;
    bool is_running() const;
    bool is_suspended() const;

    // set just this node's status
    void set_status(NodeStatus newStatus);

    // set this node and all childrens' status, recursively
    void set_status_all(NodeStatus newStatus);

    // set only the direct children's status
    void set_status_children(NodeStatus newStatus);

    void set_result(NodeResult result);

    void set_result_children(NodeResult result);

    NodeStatus get_status() const;

    NodeResult get_result() const;

    void tick(float dt);

    const char *get_name() const;
    const char *get_summary() const;

    virtual BehaviorNode *clone() = 0;

	

protected:
    BehaviorAgent *agent;
    NodeStatus status;
    NodeResult result;
    BehaviorNode *parent;
    std::vector<BehaviorNode *> children;
    std::wstring wideName;

	void on_leaf_enter();

    // override for any non-generic logic
    virtual void on_enter();
    virtual void on_update(float dt);
    virtual void on_exit();

    // convenience functions for setting status and result
    void on_success();
    void on_failure();

    void display_leaf_text();

private:
    const char *name;
    const char *summary;

    void set_debug_info(const char *name, const char *summary);
    void add_child(BehaviorNode *child);
};

// just a simple CRTP intermediary for cloning derived types
template <typename T>
class BaseNode : public BehaviorNode
{
public:
    virtual BehaviorNode *clone()
    {
        T &self = *static_cast<T *>(this);
        return new T(self);
    }
};