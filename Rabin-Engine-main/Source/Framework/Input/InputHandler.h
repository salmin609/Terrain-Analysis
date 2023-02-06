/******************************************************************************/
/*!
\file		Input.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Keyboard and mouse input management

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include <queue>
#include "KeyboardKeys.h"
#include "MouseButtons.h"
#include "Misc/NiceTypes.h"

class InputHandler
{
public:
    enum class InputState
    {
        RELEASED,
        PRESSED
    };

    static void notify_when_key_pressed(KBKeys key, Callback callback);
    static void notify_when_key_released(KBKeys key, Callback callback);

    static void notify_when_mouse_pressed(MouseButtons button, Callback callback);
    static void notify_when_mouse_released(MouseButtons button, Callback callback);
   
    static const WindowPos &get_mouse_position();
    static InputState get_current_state(KBKeys key);
    static InputState get_current_state(MouseButtons button);

    static void update();
    static void reset_states();
    static void forced_reset();

    static void set_focus(bool value);
    static void process_key_message(UINT message, WPARAM wParam, LPARAM lParam);

private:
    struct KBStateChange
    {
        KBKeys key;
        InputState state;
    };

    struct MouseStateChange
    {
        MouseButtons button;
        InputState state;
    };

    static bool hasFocus;
    static std::queue<KBStateChange> keyboardQueue;
    static std::queue<MouseStateChange> mouseQueue;
    static std::array<InputState, static_cast<size_t>(KBKeys::NUM_ENTRIES)> keyboardState;
    static std::array<InputState, static_cast<size_t>(MouseButtons::NUM_ENTRIES)> mouseState;

    static std::array<std::vector<Callback>, static_cast<size_t>(KBKeys::NUM_ENTRIES)> keyboardPressedCallbacks;
    static std::array<std::vector<Callback>, static_cast<size_t>(KBKeys::NUM_ENTRIES)> keyboardReleasedCallbacks;

    static std::array<std::vector<Callback>, static_cast<size_t>(MouseButtons::NUM_ENTRIES)> mousePressedCallbacks;
    static std::array<std::vector<Callback>, static_cast<size_t>(MouseButtons::NUM_ENTRIES)> mouseReleasedCallbacks;
    static WindowPos mousePos;

    static void process_keyboard_queue();
    static void process_mouse_queue();

    static void send_key_pressed(KBKeys key);
    static void send_key_released(KBKeys key);

    static void send_mouse_pressed(MouseButtons button);
    static void send_mouse_released(MouseButtons button);
};