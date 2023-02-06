/******************************************************************************/
/*!
\file		UIButton.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Base UI button

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include "UI/Elements/UIElement.h"
#include "UI/UITextObject.h"
#include "UI/UIColorHighlighter.h"

class UIButton : public UIElement
{
public:
    UIButton(UIAnchor anchor, const WindowPos &offsets, Callback callback, const std::wstring &label);
    UIButton(UIAnchor anchor, UIElement *other, int padding, Callback callback, const std::wstring &label);

    virtual void on_hover_start(const WindowPos &pos) override;
    virtual void on_hover_end(const WindowPos &pos) override;
    virtual void on_mouse_pressed(MouseButtons button) override;

    virtual void draw_sprite() override;
    virtual void draw_text() override;

    virtual const WindowSize &get_element_size() const override;
protected:
    Callback callback;
    UITextObject label;
    UIColorHighlighter color;
};