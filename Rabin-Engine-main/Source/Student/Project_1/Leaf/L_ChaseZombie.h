#pragma once
#include "BehaviorNode.h"

class L_ChaseZombie : public BaseNode<L_ChaseZombie>
{
public:
	L_ChaseZombie();
protected:
	virtual void on_enter() override;
	virtual void on_update(float dt) override;
private:
	void FindClosestZombie();

	Agent* closestZombie;
	bool shouldChase;
	float detectionDistance = 30.f;
};