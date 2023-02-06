/******************************************************************************/
/*!
\file		UITextField.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Base UI text field

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "UITextField.h"

UITextField::UITextField(UIAnchor screenAnchor, const WindowPos &offsets,
    UIAnchor textAnchor, const std::wstring &text, TextColor color) :
    UIElement(screenAnchor, offsets, get_element_size()), text(textAnchor, text),
    color(color)
{}

UITextField::UITextField(UIAnchor screenAnchor, UIElement *other, int padding,
    UIAnchor textAnchor, const std::wstring &text, TextColor color) :
    UIElement(screenAnchor, other, padding, get_element_size()), text(textAnchor, text),
    color(color)
{}

bool UITextField::in_bounding_box(const WindowPos &pos) const
{
    // we never want the text field to be considered hovered
    return false;
}

void UITextField::draw_text()
{
    text.draw(box, color);
}

const WindowSize &UITextField::get_element_size() const
{
    static const WindowSize size { 128, 16 };
    return size;
}
