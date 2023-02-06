/******************************************************************************/
/*!
\file		UIBannerTextField.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	UI text that stretches across the screen

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "UIBannerTextField.h"

UIBannerTextField::UIBannerTextField(UIAnchor screenAnchor, const WindowPos &offsets,
    UIAnchor textAnchor, const std::wstring &text, TextColor color) : 
    UITextField(screenAnchor, offsets, textAnchor, text, color)
{
    const auto &windowSize = renderer->get_resources().get_output_size();
    set_size(windowSize);
}

UIBannerTextField::UIBannerTextField(UIAnchor screenAnchor, UIElement *other,
    int padding, UIAnchor textAnchor, const std::wstring &text, TextColor color) :
    UITextField(screenAnchor, other, padding, textAnchor, text, color)
{
    const auto &windowSize = renderer->get_resources().get_output_size();
    set_size(windowSize);
}

void UIBannerTextField::update_position(const WindowSize &size)
{
    set_size(size);
}

void UIBannerTextField::set_size(const WindowSize &size)
{
    auto newSize = box.get_size();
    newSize.width = size.width;

    box = UIBoundingBox(box.center(), newSize);

    UIElement::update_position(size);
}
