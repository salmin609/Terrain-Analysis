/******************************************************************************/
/*!
\file		KeyboardKeys.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Keyboard enumeration and helper function

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "KeyboardKeys.h"

KBKeys convert_key_from_windows(WPARAM wParam)
{
    auto param = GET_KEYSTATE_WPARAM(wParam);
    switch (param)
    {
    case 0x08:
        return KBKeys::BACKSPACE;
    case 0x09:
        return KBKeys::TAB;
    case 0x0D:
        return KBKeys::ENTER;
    case 0x10:
        return KBKeys::SHIFT;
    case 0x11:
        return KBKeys::CONTROL;
    case 0x12:
        return KBKeys::ALT;
    case 0x14:
        return KBKeys::CAPSLOCK;
    case 0x1B:
        return KBKeys::ESCAPE;
    case 0x20:
        return KBKeys::SPACEBAR;
    case 0x21:
        return KBKeys::PAGE_UP;
    case 0x22:
        return KBKeys::PAGE_DOWN;
    case 0x23:
        return KBKeys::END;
    case 0x24:
        return KBKeys::HOME;
    case 0x25:
        return KBKeys::LEFT;
    case 0x26:
        return KBKeys::UP;
    case 0x27:
        return KBKeys::RIGHT;
    case 0x28:
        return KBKeys::DOWN;
    case 0x2D:
        return KBKeys::INSERT;
    case 0x2E:
        return KBKeys::DEL;
    case 0x30:
        return KBKeys::ZERO;
    case 0x31:
        return KBKeys::ONE;
    case 0x32:
        return KBKeys::TWO;
    case 0x33:
        return KBKeys::THREE;
    case 0x34:
        return KBKeys::FOUR;
    case 0x35:
        return KBKeys::FIVE;
    case 0x36:
        return KBKeys::SIX;
    case 0x37:
        return KBKeys::SEVEN;
    case 0x38:
        return KBKeys::EIGHT;
    case 0x39:
        return KBKeys::NINE;
    case 0x41:
        return KBKeys::A;
    case 0x42:
        return KBKeys::B;
    case 0x43:
        return KBKeys::C;
    case 0x44:
        return KBKeys::D;
    case 0x45:
        return KBKeys::E;
    case 0x46:
        return KBKeys::F;
    case 0x47:
        return KBKeys::G;
    case 0x48:
        return KBKeys::H;
    case 0x49:
        return KBKeys::I;
    case 0x4A:
        return KBKeys::J;
    case 0x4B:
        return KBKeys::K;
    case 0x4C:
        return KBKeys::L;
    case 0x4D:
        return KBKeys::M;
    case 0x4E:
        return KBKeys::N;
    case 0x4F:
        return KBKeys::O;
    case 0x50:
        return KBKeys::P;
    case 0x51:
        return KBKeys::Q;
    case 0x52:
        return KBKeys::R;
    case 0x53:
        return KBKeys::S;
    case 0x54:
        return KBKeys::T;
    case 0x55:
        return KBKeys::U;
    case 0x56:
        return KBKeys::V;
    case 0x57:
        return KBKeys::W;
    case 0x58:
        return KBKeys::X;
    case 0x59:
        return KBKeys::Y;
    case 0x5A:
        return KBKeys::Z;
    case 0x60:
        return KBKeys::NP_ZERO;
    case 0x61:
        return KBKeys::NP_ONE;
    case 0x62:
        return KBKeys::NP_TWO;
    case 0x63:
        return KBKeys::NP_THREE;
    case 0x64:
        return KBKeys::NP_FOUR;
    case 0x65:
        return KBKeys::NP_FIVE;
    case 0x66:
        return KBKeys::NP_SIX;
    case 0x67:
        return KBKeys::NP_SEVEN;
    case 0x68:
        return KBKeys::NP_EIGHT;
    case 0x69:
        return KBKeys::NP_NINE;
    case 0x70:
        return KBKeys::F1;
    case 0x71:
        return KBKeys::F2;
    case 0x72:
        return KBKeys::F3;
    case 0x73:
        return KBKeys::F4;
    case 0x74:
        return KBKeys::F5;
    case 0x75:
        return KBKeys::F6;
    case 0x76:
        return KBKeys::F7;
    case 0x77:
        return KBKeys::F8;
    case 0x78:
        return KBKeys::F9;
    case 0x79:
        return KBKeys::F10;
    case 0x7A:
        return KBKeys::F11;
    case 0x7B:
        return KBKeys::F12;
    case 0xBA:
        return KBKeys::SEMI_COLON;
    case 0xBB:
        return KBKeys::EQUAL;
    case 0xBC:
        return KBKeys::COMMA;
    case 0xBD:
        return KBKeys::MINUS;
    case 0xBE:
        return KBKeys::PERIOD;
    case 0xBF:
        return KBKeys::FORWARD_SLASH;
    case 0xC0:
        return KBKeys::GRAVE;
    case 0xDB:
        return KBKeys::OPEN_BRACKET;
    case 0xDC:
        return KBKeys::BACK_SLASH;
    case 0xDD:
        return KBKeys::CLOSE_BRACKET;
    case 0xDE:
        return KBKeys::APOSTROPHE;
    default:
        return KBKeys::NUM_ENTRIES;
    }
}
