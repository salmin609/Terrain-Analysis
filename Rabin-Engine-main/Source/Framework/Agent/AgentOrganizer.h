/******************************************************************************/
/*!
\file		AgentOrganizer.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Actor manager declarations

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include <unordered_map>
#include "CameraAgent.h"
#include "AStarAgent.h"
#include "EnemyAgent.h"
#include "BehaviorAgent.h"

enum class BehaviorTreeTypes;
class UIBehaviorTreeTextField;

class AgentOrganizer
{
public:
    AgentOrganizer();

    bool initialize();
    void shutdown();

    bool acquire_rendering_resources();
    void release_rendering_resources();

    // agentType is for debug display and also looking up all agents of a specific type
    BehaviorAgent *create_behavior_agent(const char *agentType, BehaviorTreeTypes treeType);
    AStarAgent *create_pathing_agent();
    EnemyAgent *create_enemy_agent();
    void destroy_agent(Agent *agent);
    
    const std::vector<Agent *> &get_all_agents() const;
    const std::vector<Agent *> &get_all_agents_by_type(const char *type);
    CameraAgent *const get_camera_agent() const;

    void draw() const;
    void draw_debug() const;
    void update(float dt);
private:
    CameraAgent *cameraAgent;
    std::vector<Agent *> agentsAll;
    std::unordered_map<const char *, std::vector<Agent *>> agentsByType;
    std::unordered_map<const char *, size_t> agentIDCounts;
    std::vector<size_t> markedForDeletion;

    std::unordered_map<BehaviorAgent *, UIBehaviorTreeTextField *> inUseTextFields;
    std::vector<UIBehaviorTreeTextField *> freeTextFields;
    UIBehaviorTreeTextField *bottomTextField;

    void assign_text_field(BehaviorAgent *agent);
    void unassign_text_field(BehaviorAgent *agent);
};