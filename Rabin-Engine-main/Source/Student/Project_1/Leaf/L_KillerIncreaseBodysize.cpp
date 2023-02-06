#include <pch.h>
#include "L_KillerIncreaseBodysize.h"

L_KillerIncreaseBodySize::L_KillerIncreaseBodySize()
{
}

void L_KillerIncreaseBodySize::on_enter()
{
	if(agent->killCount > 0)
	{
		//enter
		Vec3 originalScale = agent->get_scaling();
		agent->set_scaling(originalScale + Vec3(0.1f, 0.1f, 0.1f));
		agent->killCount -= 1;
	}

	set_status(NodeStatus::EXITING);

	
}

void L_KillerIncreaseBodySize::on_update(float dt)
{

}
