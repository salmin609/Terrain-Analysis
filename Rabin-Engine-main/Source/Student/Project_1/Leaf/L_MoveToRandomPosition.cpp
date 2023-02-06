#include <pch.h>
#include "L_MoveToRandomPosition.h"
#include "Agent/BehaviorAgent.h"

void L_MoveToRandomPosition::on_enter()
{
    // set animation, speed, etc

    const Vec3 randomPoint = RNG::world_position();
    Vec3 randomDir = randomPoint - agent->get_position();
    //Vec3::Normalize(randomDir);
    randomDir.Normalize();
    randomDir *= 10.f;

    targetPoint = agent->get_position() + randomDir;
	BehaviorNode::on_leaf_enter();
}

void L_MoveToRandomPosition::on_update(float dt)
{
    const auto result = agent->move_toward_point(targetPoint, dt);

    if (result == true)
    {
        on_success();
    }

    display_leaf_text();
}
