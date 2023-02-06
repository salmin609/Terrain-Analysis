#pragma once
#include "BehaviorNode.h"

class L_ChaseAnimal : public BaseNode<L_ChaseAnimal>
{
public:
	L_ChaseAnimal();

protected:
	virtual void on_enter() override;
	virtual void on_update(float dt) override;
	
private:
	void FindAnimal();

	bool found;
	float detectRange;
	Vec3 targetPoint;
	Agent* animalAgent;
};