#include <pch.h>
#include "L_ZombieCollisionCheck.h"

L_ZombieCollisionCheck::L_ZombieCollisionCheck()
{
	
}

void L_ZombieCollisionCheck::on_enter()
{
	//BaseNode<L_ZombieCollisionCheck>::on_enter();
	collided = false;
	survivorAgent = nullptr;

	const auto& allAgents = agents->get_all_agents();

	const auto& currPos = agent->get_position();

	for (const auto& a : allAgents)
	{
		// make sure it's not our agent
		if (a != agent && 
			a->agentType == Agent::AgentType::Survivor)
		{
			const auto& agentPos = a->get_position();
			const float distance = Vec3::Distance(currPos, agentPos);

			if(distance < 5.f)
			{
				collided = true;
				survivorAgent = a;
			}

		}
	}

	if(collided == true)
	{
		set_status(NodeStatus::RUNNING);
	}
	else
	{
		set_status(NodeStatus::EXITING);
	}

}

void L_ZombieCollisionCheck::on_update(float dt)
{
	//BaseNode<L_ZombieCollisionCheck>::on_update(dt);

	if(survivorAgent != nullptr)
	{
		agents->destroy_agent(survivorAgent);

		set_status(NodeStatus::EXITING);
	}

}
