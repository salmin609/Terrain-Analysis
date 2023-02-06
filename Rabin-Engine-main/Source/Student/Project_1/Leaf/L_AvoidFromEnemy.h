#pragma once
#include "BehaviorNode.h"

class L_AvoidFromEnemy : public BaseNode<L_AvoidFromEnemy>
{
public:
	L_AvoidFromEnemy();
protected:
	virtual void on_enter() override;
	virtual void on_update(float dt) override;
private:
	void SearchClosestEnemy();

	bool shouldAvoid;
	Vec3 targetPoint;
	Agent* closestEnemy = nullptr;

	float leastAvoidDistance, detectionRange;

};