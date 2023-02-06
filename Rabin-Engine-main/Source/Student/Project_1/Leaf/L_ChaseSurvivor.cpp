#include <pch.h>
#include "L_ChaseSurvivor.h"

L_ChaseSurvivor::L_ChaseSurvivor()
{
}

void L_ChaseSurvivor::on_enter()
{
	shouldChase = false;

	const auto& allAgents = agents->get_all_agents();
	Vec3 currPos = agent->get_position();

	for (const auto& a : allAgents)
	{
		if (a != agent && a->agentType == Agent::AgentType::Survivor)
		{
			Vec3 agentPos = a->get_position();
			const float distance = Vec3::Distance(currPos, agentPos);

			if (distance < detectionDistance)
			{
				//Avoid
				survivorAgent = a;
				shouldChase = true;
			}
		}
	}

	if (shouldChase == true)
	{
		//BehaviorNode::on_leaf_enter();
		set_status(NodeStatus::RUNNING);
	}
	else
	{
		set_status(NodeStatus::EXITING);
	};
}

void L_ChaseSurvivor::on_update(float dt)
{
	Vec3 currPos = agent->get_position();
	Vec3 survivorPos = survivorAgent->get_position();
	Vec3 targetDir = survivorPos - currPos;
	targetPoint = currPos + targetDir;

	agent->move_toward_point(targetPoint, dt);

	const float distance = Vec3::Distance(currPos, survivorPos);

	if(distance < 1.f || distance > detectionDistance)
	{
		set_status(NodeStatus::EXITING);
	}

	display_leaf_text();
}
