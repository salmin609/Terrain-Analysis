/******************************************************************************/
/*!
\file		UIDynamicBannerTextField.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	UI banner text that updates its text

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include "UIBannerTextField.h"

// a text field that automatically sets its width to the screen size, and queries the text every frame
class UIDynamicBannerTextField : public UIBannerTextField
{
public:
    UIDynamicBannerTextField(UIAnchor screenAnchor, const WindowPos &offsets,
        UIAnchor textAnchor, TextGetter getter, TextColor color);
    UIDynamicBannerTextField(UIAnchor screenAnchor, UIElement *other, int padding,
        UIAnchor textAnchor, TextGetter getter, TextColor color);

    virtual void draw_text() override;

protected:
    TextGetter getter;
};