#pragma once
#include "BehaviorNode.h"

class C_CheckDistance : public BaseNode<C_CheckDistance>
{
public:
	C_CheckDistance();

protected:
	virtual void on_enter() override;

};