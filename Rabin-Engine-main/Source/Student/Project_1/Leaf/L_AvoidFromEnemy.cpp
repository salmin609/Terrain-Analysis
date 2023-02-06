#include <pch.h>
#include "L_AvoidFromEnemy.h"

L_AvoidFromEnemy::L_AvoidFromEnemy()
{
	leastAvoidDistance = 50.f;
	detectionRange = 30.f;
}

void L_AvoidFromEnemy::on_enter()
{
	shouldAvoid = false;

	SearchClosestEnemy();

	if(shouldAvoid == true)
	{
		set_status(NodeStatus::RUNNING);
	}
	else
	{
		set_status(NodeStatus::EXITING);
	}


}

void L_AvoidFromEnemy::on_update(float dt)
{
	SearchClosestEnemy();

	Vec3 currPos = agent->get_position();
	Vec3 enemyPos = closestEnemy->get_position();

	Vec3 targetDir = currPos - enemyPos;
	targetPoint = currPos + targetDir;

	agent->move_toward_point(targetPoint, dt);

	const float distance = Vec3::Distance(currPos, enemyPos);

	if(distance > leastAvoidDistance)
	{
		//on_success();
		set_status(NodeStatus::EXITING);
	}

	display_leaf_text();
}

void L_AvoidFromEnemy::SearchClosestEnemy()
{
	const auto& allAgents = agents->get_all_agents();
	Vec3 currPos = agent->get_position();

	float shortestSoFar = 1000.f;

	for (const auto& a : allAgents)
	{
		if (a != agent &&
			(a->agentType == Agent::AgentType::Zombie ||
				a->agentType == Agent::AgentType::Zombie2 ||
				a->agentType == Agent::AgentType::Killer))
		{
			Vec3 agentPos = a->get_position();
			const float distance = Vec3::Distance(currPos, agentPos);

			if (distance < shortestSoFar)
			{
				//Avoid
				shortestSoFar = distance;
				closestEnemy = a;

			}
		}
	}

	if(shortestSoFar < detectionRange)
	{
		shouldAvoid = true;
	}
}
