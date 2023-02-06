/******************************************************************************/
/*!
\file		AgentOrganizer.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Actor manager implementation

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "AgentOrganizer.h"
#include "Projects/ProjectOne.h"
#include "UI/Elements/Text/UIBehaviorTreeTextField.h"

AgentOrganizer::AgentOrganizer() : cameraAgent(nullptr), bottomTextField(nullptr)
{}

bool AgentOrganizer::initialize()
{
    std::cout << "    Initializing Agent System..." << std::endl;
    cameraAgent = new CameraAgent;
    return true;
}

void AgentOrganizer::shutdown()
{
    std::cout << "    Shutting Down Agent System..." << std::endl;
    delete cameraAgent;

    for (auto && agent : agentsAll)
    {
        delete agent;
    }
}

bool AgentOrganizer::acquire_rendering_resources()
{
    return Agent::acquire_rendering_resources();
}

void AgentOrganizer::release_rendering_resources()
{
    Agent::release_rendering_resources();
}

BehaviorAgent *AgentOrganizer::create_behavior_agent(const char *type, BehaviorTreeTypes treeType)
{
    // make sure the tree builder is initialized
    if (treeBuilder)
    {
        auto &idCounter = agentIDCounts[type];
        const auto id = idCounter++;

        std::cout << "Creating agent " << type << id << std::endl;

        auto agent = new BehaviorAgent(type, id);

        // build a tree from the prototype
        treeBuilder->build_tree(treeType, agent);

        agentsAll.emplace_back(agent);
        agentsByType[type].emplace_back(agent);

        #ifdef _DEBUG
            assign_text_field(agent);
        #endif

        return agent;
    }

    std::cout << "Attempted to spawn behavior agent while not in project one" << std::endl;
    return nullptr;    
}

AStarAgent *AgentOrganizer::create_pathing_agent()
{
    auto &idCounter = agentIDCounts[AStarAgent::patherTypeName];
    const auto id = idCounter++;

    std::cout << "Creating pathing agent " << id << std::endl;

    auto agent = new AStarAgent(id);

    agentsAll.emplace_back(agent);
    agentsByType[AStarAgent::patherTypeName].emplace_back(agent);

    return agent;
}

EnemyAgent *AgentOrganizer::create_enemy_agent()
{
    auto &idCounter = agentIDCounts[AStarAgent::patherTypeName];
    const auto id = idCounter++;

    std::cout << "Creating enemy agent " << id << std::endl;

    auto agent = new EnemyAgent(id);

    agentsAll.emplace_back(agent);
    agentsByType[AStarAgent::patherTypeName].emplace_back(agent);

    return agent;
}

void AgentOrganizer::destroy_agent(Agent *agent)
{
    // prevent destroying the camera agent :)
    if (agent->get_type() == CameraAgent::cameraTypeName)
    {
        return;
    }

    for (size_t i = 0; i < agentsAll.size(); ++i)
    {
        if (agentsAll[i] == agent)
        {
            markedForDeletion.emplace_back(i);
            break;
        }
    }
}

const std::vector<Agent*> &AgentOrganizer::get_all_agents() const
{
    return agentsAll;
}

const std::vector<Agent*> &AgentOrganizer::get_all_agents_by_type(const char *type)
{
    return agentsByType[type];
}

CameraAgent *const AgentOrganizer::get_camera_agent() const
{
    return cameraAgent;
}

void AgentOrganizer::draw() const
{
    for (const auto & agent : agentsAll)
    {
        agent->draw_mesh();
    }
}

void AgentOrganizer::draw_debug() const
{
    for (const auto & agent : agentsAll)
    {
        agent->draw_debug();
    }
}

void AgentOrganizer::update(float dt)
{
    // avoid ranged for due to iterator invalidation from insertion
    for (size_t i = 0; i < agentsAll.size(); ++i)
    {
        agentsAll[i]->update(dt);
    }

    if (markedForDeletion.size() > 0)
    {
        std::sort(markedForDeletion.begin(), markedForDeletion.end());

        for (auto i = markedForDeletion.rbegin(); i != markedForDeletion.rend(); ++i)
        {
            auto agent = agentsAll[*i];

            auto type = agent->get_type();

            auto result = agentsByType.find(type);

            if (result != agentsByType.end())
            {
                for (auto j = result->second.begin(); j != result->second.end(); ++j)
                {
                    if (*j == agent)
                    {
                        result->second.erase(j);
                        break;
                    }
                }
            }

            #ifdef _DEBUG
                BehaviorAgent *bAgent = dynamic_cast<BehaviorAgent *>(agent);
                if (bAgent != nullptr)
                {
                    unassign_text_field(bAgent);
                }
            #endif

            delete agent;

            agentsAll.erase(agentsAll.begin() + *i);
        }

        markedForDeletion.clear();
    }
}

void AgentOrganizer::assign_text_field(BehaviorAgent *agent)
{
    if (freeTextFields.size() > 0)
    {
        auto textField = freeTextFields.back();
        freeTextFields.pop_back();

        textField->enable(agent);

        inUseTextFields.emplace(agent, textField);
    }
    else
    {
        UIBehaviorTreeTextField *textField;

        if (bottomTextField != nullptr)
        {
            textField = ui->create_behavior_tree_text_field(bottomTextField, agent); 
        }
        else
        {
            textField = ui->create_behavior_tree_text_field(UIAnchor::TOP, 26, 64, agent);
        }

        bottomTextField = textField;

        inUseTextFields.emplace(agent, textField);
    }
}

void AgentOrganizer::unassign_text_field(BehaviorAgent *agent)
{
    auto result = inUseTextFields.find(agent);

    if (result != inUseTextFields.end())
    {
        result->second->disable();
        freeTextFields.emplace_back(result->second);
        inUseTextFields.erase(result);
    }
}
