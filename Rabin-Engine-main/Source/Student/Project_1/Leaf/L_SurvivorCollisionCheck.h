#pragma once
#include "BehaviorNode.h"

class L_SurvivorCollisionCheck : public BaseNode<L_SurvivorCollisionCheck>
{
public:
	L_SurvivorCollisionCheck();
protected:
	virtual void on_enter() override;
	virtual void on_update(float dt) override;
private:
	void SearchAnimalInRange();

	Agent* animalAgent;
	float timeCheck;
	float ogTime;
};