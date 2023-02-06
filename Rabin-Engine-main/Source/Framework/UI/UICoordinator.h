/******************************************************************************/
/*!
\file		UICoordinator.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	UI management

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include <vector>
#include "Elements/Sliders/UISlider.h"
#include "UIAnchor.h"
#include "Input/KeyboardKeys.h"
#include "Input/MouseButtons.h"

// forward declarations
class UIElement;
class UIButton;
class UIDynamicTextField;
class UITextField;
class UIValueTextField;
class UIDynamicButton;
class UIToggleButton;
class UIConditionalButton;
class UIBannerTextField;
class UIDynamicBannerTextField;
class UIBehaviorTreeTextField;

class ProjectOne;
class ProjectTwo;
class ProjectThree;

class UICoordinator
{
    friend class ProjectOne;
    friend class ProjectTwo;
    friend class ProjectThree;
public:
    UICoordinator();

    bool is_consuming_mouse_input() const;
    bool is_consuming_keyboard_input() const;
    bool has_hovered_element() const;
    bool has_focused_element() const;

    /* 
        UI elements positions are absolute positions in window pixel coordinates.
        So (0, 0) is the top left and (w, h) is the bottom right.  Anchor points
        are used to specify which position of the window you want to offset the
        element.  All offsets are also in window pixel coordinates, so positive
        values move the position down or right, while negative values move it up
        or left;

        All UIElements are dynamically allocated and will be freed by the UICoordinator.
        The pointers are handed back primarily to facilitate chaining elements,
        for easy positioning.
    */

    // takes in a callback that is executed when the button is clicked
    UIButton *create_button(UIAnchor anchor, int horizontal, int vertical,
        Callback clickCallback, const std::wstring &label);

    // a button that has dynamic text
    UIDynamicButton *create_dynamic_button(UIAnchor anchor, int horizontal, int vertical,
        Callback clickCallback, TextGetter text);

    // a button that visualizes and toggles the state of a boolean
    UIToggleButton *create_toggle_button(UIAnchor anchor, int horizontal, int vertical,
        Callback clickCallback, const std::wstring &label, Getter<bool> stateGetter);

    // a button that is only visible when a condition is true
    UIConditionalButton *create_conditional_button(UIAnchor anchor, int horizontal, int vertical,
        Callback clickCallback, const std::wstring &label, Getter<bool> condition);

    // takes in the extrema values as well as a getter and setter to allow more flexible logic
    template <typename T>
    UISlider<T> *create_slider(UIAnchor anchor, int horizontal, int vertical, const T &minValue,
        const T &maxValue, Getter<T> getter, Setter<T> setter, TextGetter text, const std::wstring &label);

    // a simple static text field
    UITextField *create_text_field(UIAnchor screenAnchor, int horizontal, int vertical,
        UIAnchor textAnchor, const std::wstring &text, TextColor color = TextColor::BLACK);

    // a simple dynamic text field that uses a getter for the text to display
    UIDynamicTextField *create_dynamic_text_field(UIAnchor screenAnchor, int horizontal,
        int vertical, UIAnchor textAnchor, TextGetter getter, TextColor color = TextColor::BLACK);

    // takes in string to use for the left side label, and a getter for the right side text
    UIValueTextField *create_value_text_field(UIAnchor anchor, int horizontal, int vertical,
        const std::wstring &label, TextGetter getter, TextColor color = TextColor::BLACK);

    // a text field that automatically sets its width to the width of the window
    UIBannerTextField *create_banner_text_field(UIAnchor screenAnchor, int horizontal,
        int vertical, UIAnchor textAnchor, const std::wstring &text, TextColor color = TextColor::BLACK);

    // a banner text field that updates its text every frame
    UIDynamicBannerTextField *create_dynamic_banner_text_field(UIAnchor screenAnchor,
        int horizontal, int vertical, UIAnchor textAnchor, TextGetter getter,
        TextColor color = TextColor::BLACK);

    // a text field for displaying behavior tree nodes in execution
    UIBehaviorTreeTextField *create_behavior_tree_text_field(UIAnchor screenAnchor,
        int horizontal, int vertical, BehaviorAgent *agent);

    


    // chained constructors, to make it easy to offset elements from each other
    // anchor point is which direction from the other element to place the new element

    // takes in a callback that is executed when the button is clicked
    UIButton *create_button(UIAnchor anchor, UIElement *element, int padding,
        Callback clickCallback, const std::wstring &label);

    // a button that has dynamic text
    UIDynamicButton *create_dynamic_button(UIAnchor anchor, UIElement *element, int padding,
        Callback clickCallback, TextGetter text);

    // a button that visualizes and toggles the state of a boolean
    UIToggleButton *create_toggle_button(UIAnchor anchor, UIElement *element, int padding,
        Callback clickCallback, const std::wstring &label, Getter<bool> stateGetter);

    // a button that is only visible when a condition is true
    UIConditionalButton *create_conditional_button(UIAnchor anchor, UIElement *element,
        int padding, Callback clickCallback, const std::wstring &label, Getter<bool> condition);
    
    // takes in the extrema values as well as a getter and setter to allow more flexible logic
    template <typename T>
    UISlider<T> *create_slider(UIAnchor anchor, UIElement *element, int padding,
        const T &minValue, const T &maxValue, Getter<T> getter, Setter<T> setter, 
        TextGetter text, const std::wstring &label);
    
    // a simple static text field
    UITextField *create_text_field(UIAnchor elementAnchor, UIElement *element,
        int padding, UIAnchor textAnchor, const std::wstring &text,
        TextColor color = TextColor::BLACK);

    // a simple dynamic text field that uses a getter for the text to display
    UIDynamicTextField *create_dynamic_text_field(UIAnchor elementAnchor, UIElement *element,
        int padding, UIAnchor textAnchor, TextGetter getter, TextColor color = TextColor::BLACK);

    // takes in string to use for the left side label, and a getter for the right side text
    UIValueTextField *create_value_text_field(UIAnchor anchor, UIElement *element, int padding,
        const std::wstring &label, TextGetter getter, TextColor color = TextColor::BLACK);

    // a text field that automatically sets its width to the width of the window
    UIBannerTextField *create_banner_text_field(UIAnchor screenAnchor, UIElement *element,
        int padding, UIAnchor textAnchor, const std::wstring &text, TextColor color = TextColor::BLACK);

    // a banner text field that updates its text every frame
    UIDynamicBannerTextField *create_dynamic_banner_text_field(UIAnchor screenAnchor,
        UIElement *element, int padding, UIAnchor textAnchor, TextGetter getter,
        TextColor color = TextColor::BLACK);

    UIBehaviorTreeTextField *create_behavior_tree_text_field(UIElement *element, BehaviorAgent *agent);


    // signal that a ui element wants to consume input, to keep from passing it to anything else
    void start_consuming_mouse_input();
    void stop_consuming_mouse_input();
    void start_consuming_keyboard_input();
    void stop_consuming_keyboard_input();

    // become the focused ui element, this is so other elements are ignored, ie for a slider so
    // that is stays focused even when the mouse is no longer directly over it
    bool acquire_focus(UIElement *element); // returns success, will fail if another element is focused
    bool relinquish_focus(UIElement *element); // returns success, will fail if not the focused element

    void on_window_size_change();
    void draw_sprites();
    void draw_text();
    void draw_debug();
    void update();
    bool initialize();
    void shutdown();

    void on_keyboard_pressed(KBKeys key);
    void on_keyboard_released(KBKeys key);
    void on_mouse_pressed(MouseButtons button);
    void on_mouse_released(MouseButtons button);
private:
    std::vector<UIElement *> elements;
    UIElement *hoveredElement;
    UIElement *focusedElement;
    bool consumingMouse;
    bool consumingKeyboard;

    UIElement *determine_hovered_element() const;
};

template<typename T>
inline UISlider<T>* UICoordinator::create_slider(UIAnchor anchor, int horizontal,
    int vertical, const T &minValue, const T &maxValue, Getter<T> getter,
    Setter<T> setter, TextGetter text, const std::wstring &label)
{
    auto slider = new UISlider<T>(anchor, WindowPos { horizontal, vertical }, minValue, maxValue, getter, setter, text, label);
    elements.emplace_back(slider);
    return slider;
}

template<typename T>
inline UISlider<T>* UICoordinator::create_slider(UIAnchor anchor, UIElement *element,
    int padding, const T &minValue, const T &maxValue, Getter<T> getter,
    Setter<T> setter, TextGetter text, const std::wstring &label)
{
    auto slider = new UISlider<T>(anchor, element, padding, minValue, maxValue, getter, setter, text, label);
    elements.emplace_back(slider);
    return slider;
}
