/******************************************************************************/
/*!
\file		KeyboardKeys.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Keyboard enumeration and helper function

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once

enum class KBKeys
{
    ESCAPE, F1, F2, F3, F4, F5, F6, F7, F8, F9, F10, F11, F12,
    GRAVE, ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE, ZERO, MINUS, EQUAL, BACKSPACE,
    TAB, Q, W, E, R, T, Y, U, I, O, P, OPEN_BRACKET, CLOSE_BRACKET, BACK_SLASH,
    CAPSLOCK, A, S, D, F, G, H, J, K, L, SEMI_COLON, APOSTROPHE, ENTER,
    SHIFT, Z, X, C, V, B, N, M, COMMA, PERIOD, FORWARD_SLASH,
    CONTROL, ALT, SPACEBAR, LEFT, RIGHT, UP, DOWN,
    INSERT, HOME, PAGE_UP, DEL, END, PAGE_DOWN,
    NP_ZERO, NP_ONE, NP_TWO, NP_THREE, NP_FOUR, NP_FIVE, NP_SIX, NP_SEVEN, NP_EIGHT, NP_NINE,

    NUM_ENTRIES
};

KBKeys convert_key_from_windows(WPARAM wParam);