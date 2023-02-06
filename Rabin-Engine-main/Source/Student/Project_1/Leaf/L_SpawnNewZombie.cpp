#include <pch.h>
#include "Projects/ProjectOne.h"
#include "L_SpawnNewZombie.h"

L_SpawnNewZombie::L_SpawnNewZombie()
{
}

void L_SpawnNewZombie::on_enter()
{
	const auto& allAgents = agents->get_all_agents();
	int zombieCount = 0;


	for(const auto& a : allAgents)
	{
		if(a->agentType == Agent::AgentType::Zombie)
		{
			zombieCount++;
		}
	}

	if(zombieCount < 3)
	{
		//Spawn new zombie
		set_status(NodeStatus::RUNNING);
	}
	else
	{
		set_status(NodeStatus::EXITING);
	}

}

void L_SpawnNewZombie::on_update(float dt)
{
	auto agent2 = agents->create_behavior_agent("Zombie", BehaviorTreeTypes::Zombie);
	agent2->set_position(Vec3(0, 0, 50));
	agent2->set_color(Vec3(1, 0, 0));
	agent2->set_movement_speed(10);
	agent2->set_scaling(Vec3(10, 2, 10));
	agent2->agentType = Agent::AgentType::Zombie;

	set_status(NodeStatus::EXITING);
}
