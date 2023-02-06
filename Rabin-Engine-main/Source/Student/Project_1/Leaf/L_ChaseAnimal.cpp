#include <pch.h>
#include "L_ChaseAnimal.h"
#include "Projects/ProjectOne.h"

L_ChaseAnimal::L_ChaseAnimal()
{
	found = false;
	detectRange = 30.f;
}

void L_ChaseAnimal::on_enter()
{
	FindAnimal();

	if(found == true)
	{
		//BehaviorNode::on_leaf_enter();
		set_status(NodeStatus::RUNNING);
	}
	else
	{
		set_status(NodeStatus::EXITING);
	}

}

void L_ChaseAnimal::on_update(float dt)
{
	FindAnimal();

	Vec3 currPos = agent->get_position();
	Vec3 animalPos = animalAgent->get_position();

	Vec3 targetDir = animalPos - currPos;
	targetPoint = currPos + targetDir;

	agent->move_toward_point(targetPoint, dt);

	const float distance = Vec3::Distance(currPos, animalPos);

	if(distance < 5.f)
	{
		set_status(NodeStatus::EXITING);
	}

	display_leaf_text();
}

void L_ChaseAnimal::FindAnimal()
{
	const auto& allAgents = agents->get_all_agents();
	Vec3 currPos = agent->get_position();

	for (const auto& a : allAgents)
	{
		if (a != agent &&
			a->agentType == Agent::AgentType::Animal)
		{
			Vec3 agentPos = a->get_position();
			const float distance = Vec3::Distance(currPos, agentPos);

			if (distance < detectRange)
			{
				//Avoid
				animalAgent = a;
				found = true;
			}
		}
	}
}
