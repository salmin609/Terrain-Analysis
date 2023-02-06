#include <pch.h>
#include "C_LeftFirst.h"

C_LeftFirst::C_LeftFirst()
{
}

void C_LeftFirst::on_enter()
{
	BehaviorNode::on_enter();
}

void C_LeftFirst::on_update(float dt)
{
	BehaviorNode* leftNode = children[0];
	BehaviorNode* rightNode = children[1];

	leftNode->tick(dt);

	if(leftNode->get_status() == NodeStatus::SUSPENDED)
	{
		//Means pass
		rightNode->tick(dt);

		set_status(rightNode->get_status());
	}
	else if(leftNode->get_status() == NodeStatus::RUNNING)
	{
		set_status(NodeStatus::RUNNING);
	}
}
