/******************************************************************************/
/*!
\file		UIConditionalButton.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	UI Button that is only visible in a given condition

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "UIConditionalButton.h"

UIConditionalButton::UIConditionalButton(UIAnchor anchor, const WindowPos &offsets,
    Callback callback, const std::wstring &label, Getter<bool> condition) :
    UIButton(anchor, offsets, callback, label), condition(condition)
{}

UIConditionalButton::UIConditionalButton(UIAnchor anchor, UIElement *other, int padding,
    Callback callback, const std::wstring &label, Getter<bool> condition) :
    UIButton(anchor, other, padding, callback, label), condition(condition)
{}

void UIConditionalButton::draw_sprite()
{
    if (condition() == true)
    {
        UIButton::draw_sprite();
    }
}

void UIConditionalButton::draw_text()
{
    if (condition() == true)
    {
        UIButton::draw_text();
    }
}

bool UIConditionalButton::in_element_specific_space(const WindowPos &point) const
{
    // if we hit here the mouse is in the bounding box
    return condition();
}
