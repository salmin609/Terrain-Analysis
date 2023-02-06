/******************************************************************************/
/*!
\file		MouseButtons.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Mouse enumeration and helper function

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "MouseButtons.h"

MouseButtons convert_mouse_from_windows(WPARAM wParam)
{
    switch (wParam)
    {
    case 0x01:
        return MouseButtons::LEFT;
    case 0x02:
        return MouseButtons::RIGHT;
    case 0x10:
        return MouseButtons::MIDDLE;
    case 0x20:
        return MouseButtons::BACK;
    case 0x40:
        return MouseButtons::FORWARD;
    default:
        return MouseButtons::NUM_ENTRIES;
    }
}