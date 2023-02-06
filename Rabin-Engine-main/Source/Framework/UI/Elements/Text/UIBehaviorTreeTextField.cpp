/******************************************************************************/
/*!
\file		UIBehaviorTreeTextField.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	UI text that displays behavior tree info

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "UIBehaviorTreeTextField.h"
#include "Agent/BehaviorAgent.h"

UIBehaviorTreeTextField::UIBehaviorTreeTextField(UIAnchor anchor,
    const WindowPos &offsets, BehaviorAgent *a) : UIBannerTextField(anchor, offsets,
        UIAnchor::LEFT, a->get_debug_name(), TextColor::BLACK), agent(a)
{}

UIBehaviorTreeTextField::UIBehaviorTreeTextField(UIElement *other, BehaviorAgent *a) : 
    UIBannerTextField(UIAnchor::BOTTOM, other, 10, UIAnchor::LEFT, a->get_debug_name(),
        TextColor::BLACK), agent(a)
{}

void UIBehaviorTreeTextField::draw_text()
{
    if (agent != nullptr)
    {
        const auto &name = agent->get_debug_name();
        auto &nodeText = agent->get_debug_text();

        std::wstringstream displayText;
        displayText << name << L": " << nodeText.str();

        std::wstringstream empty;
        nodeText.swap(empty);

        text.set_text(displayText.str());
        UITextField::draw_text();
    }
}

void UIBehaviorTreeTextField::enable(BehaviorAgent *newAgent)
{
    agent = newAgent;
}

void UIBehaviorTreeTextField::disable()
{
    agent = nullptr;
}
