/******************************************************************************/
/*!
\file		UIDynamicButton.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	UI Button that updates its display text

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "UIDynamicButton.h"

UIDynamicButton::UIDynamicButton(UIAnchor anchor, const WindowPos &offsets,
    Callback callback, TextGetter getter) :
    UIButton(anchor, offsets, callback, getter()), getter(getter)
{}

UIDynamicButton::UIDynamicButton(UIAnchor anchor, UIElement *other,
    int padding, Callback callback, TextGetter getter) :
    UIButton(anchor, other, padding, callback, getter()), getter(getter)
{}

void UIDynamicButton::draw_text()
{
    label.set_text(getter());

    UIButton::draw_text();
}
