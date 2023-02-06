/******************************************************************************/
/*!
\file		UIElement.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Base UI element

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "UIElement.h"

UIElement::UIElement(UIAnchor anchor, const WindowPos &offsets, const WindowSize &size) :
    anchor(anchor), offsets(offsets)
{
    const auto &windowSize = renderer->get_resources().get_output_size();
    const auto anchorPoint = get_screen_anchor_point(anchor, windowSize);
    
    box = UIBoundingBox(anchorPoint, size);
    
    box.offset(offsets);
}

UIElement::UIElement(UIAnchor anc, UIElement *other, int padding, const WindowSize &size)
{
    // we'll offset from the other element as our anchor, but save their
    // their anchor point so we adjust properly if the window resizes

    const auto &theirSize = other->get_element_size();
    
    const auto widthDist = (theirSize.width >> 1) + (size.width >> 1) + padding;
    const auto heightDist = (theirSize.height >> 1) + (size.height >> 1) + padding;

    anchor = other->anchor;

    WindowPos off { 0, 0 };
    switch (anc)
    {
    case UIAnchor::TOP_LEFT:
        off.x = -widthDist;
        off.y = -heightDist;
        break;
    case UIAnchor::TOP:
        off.y = -heightDist;
        break;
    case UIAnchor::TOP_RIGHT:
        off.x = widthDist;
        off.y = -heightDist;
        break;
    case UIAnchor::LEFT:
        off.x = -widthDist;
        break;
    case UIAnchor::CENTER:
        break;
    case UIAnchor::RIGHT:
        off.x = widthDist;
        break;
    case UIAnchor::BOTTOM_LEFT:
        off.x = -widthDist;
        off.y = heightDist;
        break;
    case UIAnchor::BOTTOM:
        off.y = heightDist;
        break;
    case UIAnchor::BOTTOM_RIGHT:
        off.x = widthDist;
        off.y = heightDist;
        break;
    }

    box = UIBoundingBox(other->box.center(), size);
    box.offset(off);
    offsets = other->offsets + off;
}

void UIElement::on_hover_start(const WindowPos &pos)
{
    // no op
}

void UIElement::on_hover(const WindowPos &pos)
{
    // no op
}

void UIElement::on_hover_end(const WindowPos &pos)
{
    // no op
}

void UIElement::on_mouse_pressed(MouseButtons button)
{
    // no op
}

void UIElement::on_mouse_released(MouseButtons button)
{
    // no op
}

void UIElement::on_key_pressed(KBKeys key)
{
    // no op
}

void UIElement::on_key_released(KBKeys key)
{
    // no op
}

void UIElement::on_focus(const WindowPos &pos)
{
    // no op
}

void UIElement::draw_sprite()
{
    // no op
}

void UIElement::draw_text()
{
    // no op
}

void UIElement::draw_debug()
{
    // no op
}

void UIElement::update_position(const WindowSize &size)
{
    const auto anchorPoint = get_screen_anchor_point(anchor, size);
    
    box.move_to(anchorPoint + offsets);
}

bool UIElement::in_bounding_box(const WindowPos &point) const
{
    return box.contains(point);
}

bool UIElement::in_element_specific_space(const WindowPos &point) const
{
    return true;
}

const UIBoundingBox &UIElement::get_bounding_box() const
{
    return box;
}

WindowPos UIElement::get_screen_anchor_point(UIAnchor anchor, const WindowSize &windowSize)
{
    WindowPos result { 0, 0 };

    switch (anchor)
    {
    case UIAnchor::TOP_LEFT:
        break; // already 0, 0
    case UIAnchor::TOP:
        result.x = windowSize.width / 2;
        break;

    case UIAnchor::TOP_RIGHT:
        result.x = windowSize.width;
        break;

    case UIAnchor::LEFT:
        result.y = windowSize.height / 2;
        break;

    case UIAnchor::CENTER:
        result.x = windowSize.width / 2;
        result.y = windowSize.height / 2;
        break;

    case UIAnchor::RIGHT:
        result.x = windowSize.width;
        result.y = windowSize.height / 2;
        break;

    case UIAnchor::BOTTOM_LEFT:
        result.y = windowSize.height;
        break;

    case UIAnchor::BOTTOM:
        result.x = windowSize.width / 2;
        result.y = windowSize.height;
        break;

    case UIAnchor::BOTTOM_RIGHT:
        result.x = windowSize.width;
        result.y = windowSize.height;
        break;
    }

    return result;
}
