#pragma once
#include "BehaviorNode.h"

class L_SpawnNewZombie : public BaseNode<L_SpawnNewZombie>
{
public:
    L_SpawnNewZombie();
protected:

    virtual void on_enter() override;
    virtual void on_update(float dt) override;
    
};