/******************************************************************************/
/*!
\file		UIButton.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Base UI button

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "UIButton.h"
#include "UI/UIColors.h"

UIButton::UIButton(UIAnchor anchor, const WindowPos &offsets, Callback callback,
    const std::wstring &lbl) : UIElement(anchor, offsets, get_element_size()),
    callback(callback), label(UIAnchor::CENTER, lbl), color(interactableBaseColor, interactableHoverColor)
{}

UIButton::UIButton(UIAnchor anchor, UIElement *other, int padding, Callback callback,
    const std::wstring &lbl) : UIElement(anchor, other, padding, get_element_size()),
    callback(callback), label(UIAnchor::CENTER, lbl), color(interactableBaseColor, interactableHoverColor)
{}

void UIButton::on_hover_start(const WindowPos &pos)
{
    color.start_highlighting();

    ui->start_consuming_mouse_input();
}

void UIButton::on_hover_end(const WindowPos &pos)
{
    color.stop_highlighting();

    ui->stop_consuming_mouse_input();
}

void UIButton::on_mouse_pressed(MouseButtons button)
{
    callback();
}

void UIButton::draw_sprite()
{
    renderer->get_ui_sprite_renderer().draw_button(box, color.get_color());
}

void UIButton::draw_text()
{
    label.draw(box); // we'll use our bounding box so the text is on top of the button
}

const WindowSize &UIButton::get_element_size() const
{
    static const WindowSize size { 128, 32 };
    return size;
}
