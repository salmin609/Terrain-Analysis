/******************************************************************************/
/*!
\file		UIDynamicTextField.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	UI text that updates its text

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include "UITextField.h"

// a text field that queries its text every frame
class UIDynamicTextField : public UITextField
{
public:
    UIDynamicTextField(UIAnchor screenAnchor, const WindowPos &offsets,
        UIAnchor textAnchor, TextGetter getter, TextColor color);
    UIDynamicTextField(UIAnchor screenAnchor, UIElement *other, int padding,
        UIAnchor textAnchor, TextGetter getter, TextColor color);

    virtual void draw_text() override;

protected:
    TextGetter getter;
};