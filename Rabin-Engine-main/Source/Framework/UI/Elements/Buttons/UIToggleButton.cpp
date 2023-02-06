/******************************************************************************/
/*!
\file		UIToggleButton.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	UI Button that toggles a boolean value

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "UIToggleButton.h"

UIToggleButton::UIToggleButton(UIAnchor anchor, const WindowPos &offsets,
    Callback callback, const std::wstring &label, Getter<bool> state) :
    UIButton(anchor, offsets, callback, label), state(state)
{
    update_colors();
}

UIToggleButton::UIToggleButton(UIAnchor anchor, UIElement *other, int padding,
    Callback callback, const std::wstring &label, Getter<bool> state) :
    UIButton(anchor, other, padding, callback, label), state(state)
{
    update_colors();
}

void UIToggleButton::draw_sprite()
{
    update_colors();
    UIButton::draw_sprite();
}

void UIToggleButton::draw_text()
{
    if (state() == true)
    {
        label.draw(box, TextColor::WHITE);
    }
    else
    {
        label.draw(box, TextColor::BLACK);
    }
}

void UIToggleButton::update_colors()
{
    if (state() == true)
    {
        color.change_colors(enabledBaseColor, enabledHoverColor);
    }
    else
    {
        color.change_colors(disabledBaseColor, disabledHoverColor);
    }
}
