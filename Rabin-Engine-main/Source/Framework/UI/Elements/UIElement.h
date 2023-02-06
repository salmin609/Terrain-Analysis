/******************************************************************************/
/*!
\file		UIElement.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Base UI element

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include "Misc/NiceTypes.h"
#include "UI/UIAnchor.h"
#include "UI/UIBoundingBox.h"
#include "Input/KeyboardKeys.h"
#include "Input/MouseButtons.h"

class UIElement
{
public:
    UIElement(UIAnchor anchor, const WindowPos &offsets, const WindowSize &size);
    UIElement(UIAnchor anchor, UIElement *other, int padding, const WindowSize &size);

    // called when the mouse begins hovering the element
    virtual void on_hover_start(const WindowPos &pos);

    // called every frame the mouse continues hovering the element
    virtual void on_hover(const WindowPos &pos);

    // called when the mouse stops hovering the element
    virtual void on_hover_end(const WindowPos &pos);

    // called when a mouse button is pressed while hovering the element, or if focused
    // IFF the element indicated that it wants to capture input
    virtual void on_mouse_pressed(MouseButtons button);

    // called when a mouse button is released while hovering the element, or if focused
    // IFF the element indicated that it wants to capture input
    virtual void on_mouse_released(MouseButtons button);

    // called when a keyboard key is pressed while hovering the element, or if focused
    // IFF the element indicated that it wants to capture input
    virtual void on_key_pressed(KBKeys key);

    // called when a keyboard key is released while hovering the element, or if focused
    // IFF the element indicated that it wants to capture input
    virtual void on_key_released(KBKeys key);

    // called every frame when the element is focused
    virtual void on_focus(const WindowPos &pos);

    virtual void draw_sprite();
    virtual void draw_text();
    virtual void draw_debug();

    virtual void update_position(const WindowSize &size);

    // generic bounding box check for every element
    virtual bool in_bounding_box(const WindowPos &point) const;

    virtual const WindowSize &get_element_size() const = 0;

    // any element specific logic that is more fine grained than bounding box
    virtual bool in_element_specific_space(const WindowPos &point) const;

    const UIBoundingBox &get_bounding_box() const;
protected:
    // keeping things in window pixel coords to allow easy checks vs mouse
    // recalculating only happens on window resize, so very rarely
    UIAnchor anchor;
    WindowPos offsets; // how many pixels from the anchor
    UIBoundingBox box;

    static WindowPos get_screen_anchor_point(UIAnchor anchor, const WindowSize &windowSize);
};