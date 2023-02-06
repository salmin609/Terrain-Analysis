#pragma once
#include "BehaviorNode.h"

class L_Zombie2Patrol : public BaseNode<L_Zombie2Patrol>
{
public:
	L_Zombie2Patrol();
protected:
	virtual void on_enter() override;
	virtual void on_update(float dt) override;
private:
	int teleport2RandomIndex, move2RandomIndex;
	int cornerIndex = 0;
	Vec3 corners[4];
};