/******************************************************************************/
/*!
\file		UIDynamicButton.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	UI Button that updates its display text

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include "UIButton.h"

// a button that has dynamic text
class UIDynamicButton : public UIButton
{
public:
    UIDynamicButton(UIAnchor anchor, const WindowPos &offsets, Callback callback, TextGetter getter);
    UIDynamicButton(UIAnchor anchor, UIElement *other, int padding, Callback callback, TextGetter getter);

    virtual void draw_text() override;

protected:
    TextGetter getter;
};