#include <pch.h>
#include "Projects/ProjectOne.h"
#include "Agent/CameraAgent.h"

void ProjectOne::setup()
{
    // Create your inital agents
    auto agent1 = agents->create_behavior_agent("Survivor", BehaviorTreeTypes::Survivor);
    agent1->set_position(Vec3(50, 0, 50));
    agent1->set_color(Vec3(1, 0.83, 0));
    agent1->set_movement_speed(20);
    agent1->set_scaling(Vec3(10, 2, 10));
    agent1->agentType = Agent::AgentType::Survivor;


    auto agent2 = agents->create_behavior_agent("Zombie", BehaviorTreeTypes::Zombie);
    agent2->set_position(Vec3(0, 0, 50));
    agent2->set_color(Vec3(1, 0, 0));
    agent2->set_movement_speed(10);
    agent2->set_scaling(Vec3(10, 2, 10));
    agent2->agentType = Agent::AgentType::Zombie;

    auto agent3 = agents->create_behavior_agent("Animal", BehaviorTreeTypes::Survivor);
    agent3->set_position(Vec3(70, 0, 50));
    agent3->set_color(Vec3(0, 1, 0));
    agent3->set_movement_speed(15);
    agent3->set_scaling(Vec3(10, 2, 10));
    agent3->agentType = Agent::AgentType::Animal;

    auto agent4 = agents->create_behavior_agent("Killer", BehaviorTreeTypes::Killer);
    agent4->set_position(Vec3(90, 0, 0));
    agent4->set_color(Vec3(0, 0, 1));
    agent4->set_movement_speed(15);
    agent4->set_scaling(Vec3(10, 2, 10));
    agent4->agentType = Agent::AgentType::Killer;

    auto agent5 = agents->create_behavior_agent("Zombie2", BehaviorTreeTypes::Zombie2);
    agent5->set_position(Vec3(0, 0, 70));
    agent5->set_color(Vec3(0.8, 0.8, 1));
    agent5->set_movement_speed(15);
    agent5->set_scaling(Vec3(10, 2, 10));
    agent5->agentType = Agent::AgentType::Zombie2;

    // you can technically load any map you want, even create your own map file,
    // but behavior agents won't actually avoid walls or anything special, unless you code that yourself
    // that's the realm of project 2 though
    terrain->goto_map(0);

    // you can also enable the pathing layer and set grid square colors as you see fit
    // works best with map 0, the completely blank map
    terrain->pathLayer.set_enabled(true);
    terrain->pathLayer.set_value(0, 0, Colors::Red);

    // camera position can be modified from this default as well
    auto camera = agents->get_camera_agent();
    camera->set_position(Vec3(-62.0f, 70.0f, terrain->mapSizeInWorld * 0.5f));
    camera->set_pitch(0.610865); // 35 degrees
}