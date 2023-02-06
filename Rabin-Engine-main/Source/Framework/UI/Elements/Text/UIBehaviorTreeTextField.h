/******************************************************************************/
/*!
\file		UIBehaviorTreeTextField.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	UI text that displays behavior tree info

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include "UIBannerTextField.h"

class BehaviorAgent;

// a text field that displays a behavior agent name, and active nodes, can be disabled
// to avoid calling getters to destroyed agents
class UIBehaviorTreeTextField : public UIBannerTextField
{
public:
    UIBehaviorTreeTextField(UIAnchor anchor, const WindowPos &offsets, BehaviorAgent *agent);
    UIBehaviorTreeTextField(UIElement *other, BehaviorAgent *agent);

    virtual void draw_text() override;

    void enable(BehaviorAgent *newAgent);
    void disable();
private:
    BehaviorAgent *agent;
};