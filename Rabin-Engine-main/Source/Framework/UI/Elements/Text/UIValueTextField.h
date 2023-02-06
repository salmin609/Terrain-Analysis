/******************************************************************************/
/*!
\file		UIValueTextField.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	UI text for displaying a named value

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include "UITextField.h"

// a text field that displays a label and a value that is queried every frame
class UIValueTextField : public UITextField
{
public:
    UIValueTextField(UIAnchor anchor, const WindowPos &offsets, const std::wstring &label, 
        TextGetter getter, TextColor color);
    UIValueTextField(UIAnchor anchor, UIElement *other, int padding,
        const std::wstring &label, TextGetter getter, TextColor color);

    virtual void draw_text() override;
protected:
    UITextObject valueText;
    TextGetter getter;
};