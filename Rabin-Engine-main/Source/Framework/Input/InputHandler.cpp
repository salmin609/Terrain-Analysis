/******************************************************************************/
/*!
\file		Input.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Keyboard and mouse input management

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "InputHandler.h"
#include <windowsx.h>

bool InputHandler::hasFocus = false;
std::queue<InputHandler::KBStateChange> InputHandler::keyboardQueue;
std::queue<InputHandler::MouseStateChange> InputHandler::mouseQueue;
std::array<InputHandler::InputState, static_cast<size_t>(KBKeys::NUM_ENTRIES)> InputHandler::keyboardState;
std::array<InputHandler::InputState, static_cast<size_t>(MouseButtons::NUM_ENTRIES)> InputHandler::mouseState;

std::array<std::vector<Callback>, static_cast<size_t>(KBKeys::NUM_ENTRIES)> InputHandler::keyboardPressedCallbacks;
std::array<std::vector<Callback>, static_cast<size_t>(KBKeys::NUM_ENTRIES)> InputHandler::keyboardReleasedCallbacks;

std::array<std::vector<Callback>, static_cast<size_t>(MouseButtons::NUM_ENTRIES)> InputHandler::mousePressedCallbacks;
std::array<std::vector<Callback>, static_cast<size_t>(MouseButtons::NUM_ENTRIES)> InputHandler::mouseReleasedCallbacks;
WindowPos InputHandler::mousePos;

void InputHandler::notify_when_key_pressed(KBKeys key, Callback callback)
{
    keyboardPressedCallbacks[static_cast<size_t>(key)].emplace_back(callback);
}

void InputHandler::notify_when_key_released(KBKeys key, Callback callback)
{
    keyboardReleasedCallbacks[static_cast<size_t>(key)].emplace_back(callback);
}

void InputHandler::notify_when_mouse_pressed(MouseButtons button, Callback callback)
{
    mousePressedCallbacks[static_cast<size_t>(button)].emplace_back(callback);
}

void InputHandler::notify_when_mouse_released(MouseButtons button, Callback callback)
{
    mouseReleasedCallbacks[static_cast<size_t>(button)].emplace_back(callback);
}

const WindowPos &InputHandler::get_mouse_position()
{
    return mousePos;
}

InputHandler::InputState InputHandler::get_current_state(KBKeys key)
{
    return keyboardState[static_cast<size_t>(key)];
}

InputHandler::InputState InputHandler::get_current_state(MouseButtons button)
{
    return mouseState[static_cast<size_t>(button)];
}

void InputHandler::update()
{
    process_keyboard_queue();
    process_mouse_queue();
}

void InputHandler::reset_states()
{
    // quick clear of the queues
    std::queue<KBStateChange> kbTemp;
    std::queue<MouseStateChange> mTemp;

    keyboardQueue.swap(kbTemp);
    mouseQueue.swap(mTemp);

    // for every key
    for (size_t i = 0; i < static_cast<size_t>(KBKeys::NUM_ENTRIES); ++i)
    {
        auto &state = keyboardState[i];

        // if it is currently down
        if (state == InputState::PRESSED)
        {
            // notify all listeners that we are reseting its state
            send_key_released(static_cast<KBKeys>(i));
        }

        // and reset the key
        state = InputState::RELEASED;
    }

    // do the same for the mouse buttons
    for (size_t i = 0; i < static_cast<size_t>(MouseButtons::NUM_ENTRIES); ++i)
    {
        auto &state = mouseState[i];

        if (state == InputState::PRESSED)
        {
            send_mouse_released(static_cast<MouseButtons>(i));
        }

        state = InputState::RELEASED;
    }
}

void InputHandler::forced_reset()
{
    // quick clear of the queues
    std::queue<KBStateChange> kbTemp;
    std::queue<MouseStateChange> mTemp;

    keyboardQueue.swap(kbTemp);
    mouseQueue.swap(mTemp);

    // for every key
    for (size_t i = 0; i < static_cast<size_t>(KBKeys::NUM_ENTRIES); ++i)
    {
        auto &state = keyboardState[i];

        // and reset the key
        state = InputState::RELEASED;
    }

    // do the same for the mouse buttons
    for (size_t i = 0; i < static_cast<size_t>(MouseButtons::NUM_ENTRIES); ++i)
    {
        auto &state = mouseState[i];

        state = InputState::RELEASED;
    }

    for (auto && cbv : keyboardPressedCallbacks)
    {
        cbv.clear();
    }

    for (auto && cbv : keyboardReleasedCallbacks)
    {
        cbv.clear();
    }

    for (auto && cbv : mousePressedCallbacks)
    {
        cbv.clear();
    }

    for (auto && cbv : mouseReleasedCallbacks)
    {
        cbv.clear();
    }
}

void InputHandler::set_focus(bool value)
{
    hasFocus = value;
}

void InputHandler::process_key_message(UINT message, WPARAM wParam, LPARAM lParam)
{
    KBStateChange change;
    change.key = convert_key_from_windows(wParam);
    change.state = (message == WM_KEYDOWN || message == WM_SYSKEYDOWN) ? InputState::PRESSED : InputState::RELEASED;
    
    if (change.key != KBKeys::NUM_ENTRIES)
    {
        keyboardQueue.emplace(change);
    }
}

void InputHandler::process_keyboard_queue()
{
    while (keyboardQueue.empty() == false)
    {
        auto change = keyboardQueue.front();
        keyboardQueue.pop();

        auto &currState = keyboardState[static_cast<size_t>(change.key)];

        // if the key was pressed
        if (change.state == InputState::PRESSED)
        {
            if (currState == InputState::RELEASED)
            {
                send_key_pressed(change.key);
            }
        }
        else // key was released
        {
            if (currState == InputState::PRESSED)
            {
                send_key_released(change.key);
            }
        }
    }
}

void InputHandler::process_mouse_queue()
{
    constexpr size_t NUM_BUTTONS = 5;
    constexpr std::array<DWORD, static_cast<size_t>(MouseButtons::NUM_ENTRIES)> BUTTON_ID = {
        VK_LBUTTON,
        VK_RBUTTON,
        VK_MBUTTON,
        VK_XBUTTON1,
        VK_XBUTTON2
    };
    static std::array<bool, static_cast<size_t>(MouseButtons::NUM_ENTRIES)> s_buttons_prev;
    static std::array<bool, static_cast<size_t>(MouseButtons::NUM_ENTRIES)> s_buttons;

    for (size_t idx = 0; idx < NUM_BUTTONS; ++idx)
    {
        s_buttons_prev[idx] = s_buttons[idx];
        s_buttons[idx] = hasFocus ? GetAsyncKeyState(BUTTON_ID[idx]) : 0;
        if (s_buttons[idx] != s_buttons_prev[idx])
        {
            mouseQueue.emplace(MouseStateChange{
                static_cast<MouseButtons>(idx), s_buttons[idx] ? InputState::PRESSED : InputState::RELEASED
            });
        }
    }

    POINT pt;
    GetCursorPos(&pt);
    mousePos = renderer->screen_to_window(pt);

    while (mouseQueue.empty() == false)
    {
        auto change = mouseQueue.front();
        mouseQueue.pop();

        auto &currState = mouseState[static_cast<size_t>(change.button)];

        // if the button was pressed
        if (change.state == InputState::PRESSED)
        {
            if (currState == InputState::RELEASED)
            {
                send_mouse_pressed(change.button);
            }
        }
        else // button was released
        {
            if (currState == InputState::PRESSED)
            {
                send_mouse_released(change.button);
            }
        }
    }
}

void InputHandler::send_key_pressed(KBKeys key)
{
    if (ui->is_consuming_keyboard_input() == true)
    {
        ui->on_keyboard_pressed(key);
    }
    else
    {
        auto &callbacks = keyboardPressedCallbacks[static_cast<size_t>(key)];
        for (auto && cb : callbacks)
        {
            cb();
        }
    }

    keyboardState[static_cast<size_t>(key)] = InputState::PRESSED;
}

void InputHandler::send_key_released(KBKeys key)
{
    if (ui->is_consuming_keyboard_input() == true)
    {
        ui->on_keyboard_released(key);
    }
    else
    {
        auto &callbacks = keyboardReleasedCallbacks[static_cast<size_t>(key)];
        for (auto && cb : callbacks)
        {
            cb();
        }
    }

    keyboardState[static_cast<size_t>(key)] = InputState::RELEASED;
}

void InputHandler::send_mouse_pressed(MouseButtons button)
{
    if (ui->is_consuming_mouse_input() == true)
    {
        ui->on_mouse_pressed(button);
    }
    else
    {
        auto &callbacks = mousePressedCallbacks[static_cast<size_t>(button)];
        for (auto && cb : callbacks)
        {
            cb();
        }
    }

    mouseState[static_cast<size_t>(button)] = InputState::PRESSED;
}

void InputHandler::send_mouse_released(MouseButtons button)
{
    if (ui->is_consuming_mouse_input() == true)
    {
        ui->on_mouse_released(button);
    }
    else
    {
        auto &callbacks = mouseReleasedCallbacks[static_cast<size_t>(button)];
        for (auto && cb : callbacks)
        {
            cb();
        }
    }

    mouseState[static_cast<size_t>(button)] = InputState::RELEASED;
}
