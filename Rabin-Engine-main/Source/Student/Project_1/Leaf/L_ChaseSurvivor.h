#pragma once
#include "BehaviorNode.h"

class L_ChaseSurvivor : public BaseNode<L_ChaseSurvivor>
{
public:
	L_ChaseSurvivor();
protected:
	virtual void on_enter() override;
	virtual void on_update(float dt) override;
private:
	bool shouldChase;
	Vec3 targetPoint;
	Agent* survivorAgent = nullptr;
	float detectionDistance = 30.f;
};