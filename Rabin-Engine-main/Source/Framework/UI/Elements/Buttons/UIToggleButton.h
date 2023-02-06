/******************************************************************************/
/*!
\file		UIToggleButton.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	UI Button that toggles a boolean value

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include "UIButton.h"

class UIToggleButton : public UIButton
{
public:
    UIToggleButton(UIAnchor anchor, const WindowPos &offsets, Callback callback,
        const std::wstring &label, Getter<bool> state);
    UIToggleButton(UIAnchor anchor, UIElement *other, int padding, Callback callback,
        const std::wstring &label, Getter<bool> state);

    virtual void draw_sprite() override;
    virtual void draw_text() override;

protected:
    Getter<bool> state;

    void update_colors();
};