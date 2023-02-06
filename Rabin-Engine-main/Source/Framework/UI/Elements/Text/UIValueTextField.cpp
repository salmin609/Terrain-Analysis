/******************************************************************************/
/*!
\file		UIValueTextField.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	UI text for displaying a named value

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "UIValueTextField.h"

const std::wstring defaultText(L"...");

UIValueTextField::UIValueTextField(UIAnchor anchor, const WindowPos &offsets,
    const std::wstring &label, TextGetter getter, TextColor color) :
    UITextField(anchor, offsets, UIAnchor::LEFT, label, color),
    valueText(UIAnchor::RIGHT, getter()), getter(getter)
{}

UIValueTextField::UIValueTextField(UIAnchor anchor, UIElement *other, int padding,
    const std::wstring &label, TextGetter getter, TextColor color) :
    UITextField(anchor, other, padding, UIAnchor::LEFT, label, color),
    valueText(UIAnchor::RIGHT, getter()), getter(getter)
{}

void UIValueTextField::draw_text()
{
    valueText.set_text(getter());
    valueText.draw(box, color);

    UITextField::draw_text();
}
