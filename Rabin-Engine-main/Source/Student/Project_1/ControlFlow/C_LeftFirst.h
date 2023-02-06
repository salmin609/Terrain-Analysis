#pragma once
#include "BehaviorNode.h"

class C_LeftFirst : public BaseNode<C_LeftFirst>
{
public:
    C_LeftFirst();

protected:
    size_t currentIndex;

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};