#pragma once
#include "BehaviorNode.h"

class L_ZombieCollisionCheck : public BaseNode<L_ZombieCollisionCheck>
{
public:
    L_ZombieCollisionCheck();

protected:
    bool collided;
    Agent* survivorAgent;


    virtual void on_enter() override;
    virtual void on_update(float dt) override;
};