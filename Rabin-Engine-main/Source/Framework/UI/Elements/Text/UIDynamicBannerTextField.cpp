/******************************************************************************/
/*!
\file		UIDynamicBannerTextField.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	UI banner text that updates its text

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "UIDynamicBannerTextField.h"

UIDynamicBannerTextField::UIDynamicBannerTextField(UIAnchor screenAnchor,
    const WindowPos &offsets, UIAnchor textAnchor, TextGetter getter, TextColor color) :
    UIBannerTextField(screenAnchor, offsets, textAnchor, getter(), color), getter(getter)
{}

UIDynamicBannerTextField::UIDynamicBannerTextField(UIAnchor screenAnchor, 
    UIElement *other, int padding, UIAnchor textAnchor, TextGetter getter, TextColor color) :
    UIBannerTextField(screenAnchor, other, padding, textAnchor, getter(), color), getter(getter)
{}

void UIDynamicBannerTextField::draw_text()
{
    text.set_text(getter());
    UITextField::draw_text();
}
