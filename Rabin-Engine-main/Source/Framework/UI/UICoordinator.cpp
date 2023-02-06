/******************************************************************************/
/*!
\file		UICoordinator.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	UI management

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "UICoordinator.h"
#include "Elements/Buttons/UIButton.h"
#include "Elements/Buttons/UIDynamicButton.h"
#include "Elements/Buttons/UIToggleButton.h"
#include "Elements/Text/UITextField.h"
#include "Elements/Text/UIDynamicTextField.h"
#include "Elements/Text/UIValueTextField.h"
#include "Elements/Buttons/UIConditionalButton.h"
#include "Elements/Text/UIBannerTextField.h"
#include "Elements/Text/UIDynamicBannerTextField.h"
#include "Elements/Text/UIBehaviorTreeTextField.h"

UICoordinator::UICoordinator() : hoveredElement(nullptr), focusedElement(nullptr),
    consumingMouse(false), consumingKeyboard(false)
{}

bool UICoordinator::is_consuming_mouse_input() const
{
    return consumingMouse;
}

bool UICoordinator::is_consuming_keyboard_input() const
{
    return consumingKeyboard;
}

bool UICoordinator::has_hovered_element() const
{
    return hoveredElement != nullptr;
}

bool UICoordinator::has_focused_element() const
{
    return focusedElement != nullptr;
}

UIButton *UICoordinator::create_button(UIAnchor anchor, int horizontal,
    int vertical, Callback clickCallback, const std::wstring &label)
{
    auto button = new UIButton(anchor, WindowPos { horizontal, vertical }, clickCallback, label);
    elements.emplace_back(button);
    return button;
}

UIDynamicButton *UICoordinator::create_dynamic_button(UIAnchor anchor, int horizontal,
    int vertical, Callback clickCallback, TextGetter text)
{
    auto button = new UIDynamicButton(anchor, WindowPos { horizontal, vertical }, clickCallback, text);
    elements.emplace_back(button);
    return button;
}

UIToggleButton *UICoordinator::create_toggle_button(UIAnchor anchor, int horizontal,
    int vertical, Callback clickCallback, const std::wstring &label, Getter<bool> stateGetter)
{
    auto button = new UIToggleButton(anchor, WindowPos { horizontal, vertical }, clickCallback, label, stateGetter);
    elements.emplace_back(button);
    return button;
}

UIConditionalButton *UICoordinator::create_conditional_button(UIAnchor anchor,
    int horizontal, int vertical, Callback clickCallback, 
    const std::wstring &label, Getter<bool> condition)
{
    auto button = new UIConditionalButton(anchor, WindowPos { horizontal, vertical },
        clickCallback, label, condition);
    elements.emplace_back(button);
    return button;
}

UITextField *UICoordinator::create_text_field(UIAnchor screenAnchor, int horizontal,
    int vertical, UIAnchor textAnchor, const std::wstring &text, TextColor color)
{
    auto field = new UITextField(screenAnchor, WindowPos { horizontal, vertical },
        textAnchor, text, color);
    elements.emplace_back(field);
    return field;
}

UIDynamicTextField *UICoordinator::create_dynamic_text_field(UIAnchor screenAnchor,
    int horizontal, int vertical, UIAnchor textAnchor, TextGetter getter,
    TextColor color)
{
    auto field = new UIDynamicTextField(screenAnchor, WindowPos { horizontal, vertical },
        textAnchor, getter, color);
    elements.emplace_back(field);
    return field;
}

UIValueTextField *UICoordinator::create_value_text_field(UIAnchor anchor, int horizontal,
    int vertical, const std::wstring &label, TextGetter getter, TextColor color)
{
    auto field = new UIValueTextField(anchor, WindowPos { horizontal, vertical },
        label, getter, color);
    elements.emplace_back(field);
    return field;
}

UIBannerTextField *UICoordinator::create_banner_text_field(UIAnchor screenAnchor,
    int horizontal, int vertical, UIAnchor textAnchor, const std::wstring &text,
    TextColor color)
{
    auto field = new UIBannerTextField(screenAnchor, WindowPos { horizontal, vertical },
        textAnchor, text, color);
    elements.emplace_back(field);
    return field;
}

UIDynamicBannerTextField *UICoordinator::create_dynamic_banner_text_field(UIAnchor screenAnchor,
    int horizontal, int vertical, UIAnchor textAnchor, TextGetter getter, TextColor color)
{
    auto field = new UIDynamicBannerTextField(screenAnchor, WindowPos { horizontal, vertical },
        textAnchor, getter, color);
    elements.emplace_back(field);
    return field;
}

UIBehaviorTreeTextField *UICoordinator::create_behavior_tree_text_field(UIAnchor screenAnchor,
    int horizontal, int vertical, BehaviorAgent *agent)
{
    auto field = new UIBehaviorTreeTextField(screenAnchor, WindowPos { horizontal, vertical },
        agent);
    elements.emplace_back(field);
    return field;
}

UIButton *UICoordinator::create_button(UIAnchor anchor, UIElement *element,
    int padding, Callback clickCallback, const std::wstring &label)
{
    auto button = new UIButton(anchor, element, padding, clickCallback, label);
    elements.emplace_back(button);
    return button;
}

UIDynamicButton *UICoordinator::create_dynamic_button(UIAnchor anchor, UIElement *element,
    int padding, Callback clickCallback, TextGetter text)
{
    auto button = new UIDynamicButton(anchor, element, padding, clickCallback, text);
    elements.emplace_back(button);
    return button;
}

UIToggleButton *UICoordinator::create_toggle_button(UIAnchor anchor, UIElement *element,
    int padding, Callback clickCallback, const std::wstring &label, Getter<bool> stateGetter)
{
    auto button = new UIToggleButton(anchor, element, padding, clickCallback, label, stateGetter);
    elements.emplace_back(button);
    return button;
}

UIConditionalButton *UICoordinator::create_conditional_button(UIAnchor anchor,
    UIElement *element, int padding, Callback clickCallback, const std::wstring &label,
    Getter<bool> condition)
{
    auto button = new UIConditionalButton(anchor, element, padding, clickCallback, label, condition);
    elements.emplace_back(button);
    return button;
}

UITextField *UICoordinator::create_text_field(UIAnchor elementAnchor, UIElement *element,
    int padding, UIAnchor textAnchor, const std::wstring &text, TextColor color)
{
    auto field = new UITextField(elementAnchor, element, padding, textAnchor, text, color);
    elements.emplace_back(field);
    return field;
}

UIDynamicTextField *UICoordinator::create_dynamic_text_field(UIAnchor elementAnchor,
    UIElement *element, int padding, UIAnchor textAnchor, TextGetter getter, TextColor color)
{
    auto field = new UIDynamicTextField(elementAnchor, element, padding, textAnchor, getter, color);
    elements.emplace_back(field);
    return field;
}

UIValueTextField *UICoordinator::create_value_text_field(UIAnchor anchor, UIElement *element,
    int padding, const std::wstring &label, TextGetter getter, TextColor color)
{
    auto field = new UIValueTextField(anchor, element, padding, label, getter, color);
    elements.emplace_back(field);
    return field;
}

UIBannerTextField *UICoordinator::create_banner_text_field(UIAnchor screenAnchor,
    UIElement *element, int padding, UIAnchor textAnchor, const std::wstring &text, TextColor color)
{
    auto field = new UIBannerTextField(screenAnchor, element, padding, textAnchor, text, color);
    elements.emplace_back(field);
    return field;
}

UIDynamicBannerTextField *UICoordinator::create_dynamic_banner_text_field(UIAnchor screenAnchor,
    UIElement *element, int padding, UIAnchor textAnchor, TextGetter getter, TextColor color)
{
    auto field = new UIDynamicBannerTextField(screenAnchor, element, padding, textAnchor, getter, color);
    elements.emplace_back(field);
    return field;
}

UIBehaviorTreeTextField *UICoordinator::create_behavior_tree_text_field(UIElement *element,
    BehaviorAgent *agent)
{
    auto field = new UIBehaviorTreeTextField(element, agent);
    elements.emplace_back(field);
    return field;
}

void UICoordinator::start_consuming_mouse_input()
{
    consumingMouse = true;
}

void UICoordinator::stop_consuming_mouse_input()
{
    consumingMouse = false;
}

void UICoordinator::start_consuming_keyboard_input()
{
    consumingKeyboard = true;
}

void UICoordinator::stop_consuming_keyboard_input()
{
    consumingKeyboard = false;
}

bool UICoordinator::acquire_focus(UIElement *element)
{
    if (focusedElement == nullptr)
    {
        focusedElement = element;
        return true;
    }

    return false;
}

bool UICoordinator::relinquish_focus(UIElement *element)
{
    if (focusedElement == element)
    {
        focusedElement = nullptr;
        return true;
    }

    return false;
}

bool UICoordinator::initialize()
{
    std::cout << "    Initializing UI System..." << std::endl;
    return true;
}

void UICoordinator::shutdown()
{
    std::cout << "    Shutting Down UI System..." << std::endl;
    for (auto && element : elements)
    {
        delete element;
    }
}

void UICoordinator::on_keyboard_pressed(KBKeys key)
{
    if (focusedElement != nullptr)
    {
        focusedElement->on_key_pressed(key);
    }
    else if (hoveredElement != nullptr)
    {
        hoveredElement->on_key_pressed(key);
    }
}

void UICoordinator::on_keyboard_released(KBKeys key)
{
    if (focusedElement != nullptr)
    {
        focusedElement->on_key_released(key);
    }
    else if (hoveredElement != nullptr)
    {
        hoveredElement->on_key_released(key);
    }
}

void UICoordinator::on_mouse_pressed(MouseButtons button)
{
    if (focusedElement != nullptr)
    {
        focusedElement->on_mouse_pressed(button);
    }
    else if (hoveredElement != nullptr)
    {
        hoveredElement->on_mouse_pressed(button);
    }
}

void UICoordinator::on_mouse_released(MouseButtons button)
{
    if (focusedElement != nullptr)
    {
        focusedElement->on_mouse_released(button);
    }
    else if (hoveredElement != nullptr)
    {
        hoveredElement->on_mouse_released(button);
    }
}

void UICoordinator::update()
{
    auto &mousePos = InputHandler::get_mouse_position();

    // if we don't have a focused element, we need to determine if an element is hovered
    if (focusedElement == nullptr)
    {
        auto hovered = determine_hovered_element();

        // first check if we are currently hovering something
        if (hovered != nullptr)
        {
            // if we don't currently have a hovered element, just set it
            if (hoveredElement == nullptr)
            {
                hoveredElement = hovered;
                hoveredElement->on_hover_start(mousePos);
            }
            else
            {
                // if it's the same element as last frame, ping it
                if (hovered == hoveredElement)
                {
                    hoveredElement->on_hover(mousePos);
                }
                else // the user moved the mouse quick enough between elements in one frame
                {
                    // otherwise, we need to swap the hovered states
                    hoveredElement->on_hover_end(mousePos);

                    hoveredElement = hovered;
                    hoveredElement->on_hover_start(mousePos);
                }
            }
        }
        else if (hoveredElement != nullptr)
        {
            // the user has stopped hovering the active element
            hoveredElement->on_hover_end(mousePos);
            hoveredElement = nullptr;
        }
    }
    else
    {
        // just ping the focused element with the mousePos
        focusedElement->on_focus(mousePos);
    }
}

void UICoordinator::draw_sprites()
{
    for (auto && element : elements)
    {
        element->draw_sprite();
    }
}

void UICoordinator::draw_text()
{
    for (auto && element : elements)
    {
        element->draw_text();
    }
}

void UICoordinator::draw_debug()
{
    for (auto && element : elements)
    {
        element->draw_debug();
    }
}

void UICoordinator::on_window_size_change()
{
    const auto &windowSize = renderer->get_resources().get_output_size();

    for (auto && element : elements)
    {
        element->update_position(windowSize);
    }
}


UIElement *UICoordinator::determine_hovered_element() const
{
    const auto &mousePos = InputHandler::get_mouse_position();
    UIElement *hovered = nullptr;

    for (const auto & element : elements)
    {
        // quick check on the generic bounding box
        if (element->in_bounding_box(mousePos))
        {
            // possibly more involved check that is specific to the element
            if (element->in_element_specific_space(mousePos))
            {
                // found an element we are hovering over, we can cut out
                hovered = element;
                break;
            }
        }
    }

    return hovered;
}
