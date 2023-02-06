#include <pch.h>
#include "L_Zombie2Patrol.h"

L_Zombie2Patrol::L_Zombie2Patrol()
{
	cornerIndex = 0;
	corners[0] = Vec3(0, 0, 0);
	corners[1] = Vec3(100, 0, 0);
	corners[2] = Vec3(100, 0, 100);
	corners[3] = Vec3(0, 0, 100);
}

void L_Zombie2Patrol::on_enter()
{
	set_status(NodeStatus::RUNNING);
	teleport2RandomIndex = RNG::range<int>(0, 3);
	move2RandomIndex = RNG::range<int>(0, 3);

	agent->set_position(corners[teleport2RandomIndex]);

}

void L_Zombie2Patrol::on_update(float dt)
{


	bool result = agent->move_toward_point(corners[move2RandomIndex], dt);

	if(result == true)
	{
		set_status(NodeStatus::EXITING);
	}

	display_leaf_text();
	
}
