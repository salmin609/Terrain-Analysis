#include <pch.h>
#include "L_SurvivorCollisionCheck.h"

L_SurvivorCollisionCheck::L_SurvivorCollisionCheck()
{
	animalAgent = nullptr;
	ogTime = 1.f;
	timeCheck = ogTime;
}

void L_SurvivorCollisionCheck::on_enter()
{
	animalAgent = nullptr;

	SearchAnimalInRange();

	if(animalAgent != nullptr)
	{
		//Found!
		set_status(NodeStatus::RUNNING);
	}
	else
	{
		set_status(NodeStatus::EXITING);
	}
}

void L_SurvivorCollisionCheck::on_update(float dt)
{
	if(timeCheck < 0.f)
	{
		//need to consume
		const Vec3 scalingAmount = Vec3(0.2f, 0.2f, 0.2f);
		agent->set_scaling(agent->get_scaling() + scalingAmount);
		animalAgent->set_scaling(animalAgent->get_scaling() - scalingAmount);
		timeCheck = ogTime;
		set_status(NodeStatus::EXITING);
	}
	else
	{
		set_status(NodeStatus::RUNNING);
		timeCheck -= dt;
	}
}

void L_SurvivorCollisionCheck::SearchAnimalInRange()
{
	const auto& allAgents = agents->get_all_agents();
	Vec3 currPos = agent->get_position();

	for(const auto& a : allAgents)
	{
		if(a != agent &&
			a->agentType == Agent::AgentType::Animal)
		{
			Vec3 agentPos = a->get_position();
			const float distance = Vec3::Distance(currPos, agentPos);

			if(distance < 3.f)
			{
				//Collision!
				animalAgent = a;
				break;
			}
		}
	}
}
