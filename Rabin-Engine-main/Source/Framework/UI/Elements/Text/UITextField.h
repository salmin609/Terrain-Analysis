/******************************************************************************/
/*!
\file		UITextField.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Base UI text field

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include "UI/Elements/UIElement.h"
#include "UI/UITextObject.h"

class UITextField : public UIElement
{
public:
    UITextField(UIAnchor screenAnchor, const WindowPos &offsets, UIAnchor textAnchor,
        const std::wstring &text, TextColor color);
    UITextField(UIAnchor screenAnchor, UIElement *other, int padding, UIAnchor textAnchor,
        const std::wstring &text, TextColor color);

    virtual bool in_bounding_box(const WindowPos &pos) const override;

    virtual void draw_text() override;

    virtual const WindowSize &get_element_size() const override;

protected:
    UITextObject text;
    TextColor color;
};