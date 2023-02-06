#include <pch.h>
#include "L_ChaseZombie.h"

L_ChaseZombie::L_ChaseZombie()
{
}

void L_ChaseZombie::on_enter()
{
	FindClosestZombie();
	set_status(NodeStatus::RUNNING);
}

void L_ChaseZombie::on_update(float dt)
{
	Vec3 currPos = agent->get_position();
	Vec3 zombiePos = closestZombie->get_position();
	Vec3 targetDir = zombiePos - currPos;
	Vec3 targetPoint = currPos + targetDir;
	float originalSpeed = agent->get_movement_speed();

	agent->set_movement_speed(originalSpeed + 10.f);

	agent->move_toward_point(targetPoint, dt);

	agent->set_movement_speed(originalSpeed);

	const float distance = Vec3::Distance(currPos, zombiePos);

	if(distance < 5.f)
	{
		set_status(NodeStatus::EXITING);
		agents->destroy_agent(closestZombie);
		agent->killCount += 1;
	}

}

void L_ChaseZombie::FindClosestZombie()
{
	const auto& allAgents = agents->get_all_agents();
	Vec3 currPos = agent->get_position();
	float shortestSoFar = 1000.f;

	for(const auto& a : allAgents)
	{
		if(a != agent && a->agentType == Agent::AgentType::Zombie)
		{
			Vec3 agentPos = a->get_position();
			const float distance = Vec3::Distance(currPos, agentPos);

			if(distance < shortestSoFar)
			{
				shortestSoFar = distance;
				closestZombie = a;
			}
		}
	}

}
