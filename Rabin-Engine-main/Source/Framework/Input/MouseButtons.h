/******************************************************************************/
/*!
\file		MouseButtons.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Mouse enumeration and helper function

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once

enum class MouseButtons
{
    LEFT,
    RIGHT,
    MIDDLE,
    BACK,
    FORWARD,

    NUM_ENTRIES
};

MouseButtons convert_mouse_from_windows(WPARAM wParam);