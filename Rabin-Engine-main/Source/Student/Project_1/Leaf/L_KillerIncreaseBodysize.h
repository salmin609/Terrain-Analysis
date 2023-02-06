#pragma once
#include "BehaviorNode.h"

class L_KillerIncreaseBodySize : public BaseNode<L_KillerIncreaseBodySize>
{
public:
	L_KillerIncreaseBodySize();
protected:
	virtual void on_enter() override;
	virtual void on_update(float dt) override;
private:
};