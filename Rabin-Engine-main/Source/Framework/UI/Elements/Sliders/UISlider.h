/******************************************************************************/
/*!
\file		UISlider.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	UI slider that can visualize a value

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include "UI/Elements/UIElement.h"
#include "UI/UITextObject.h"
#include "UI/UIColorHighlighter.h"
#include "UI/UIColors.h"

template <typename T>
class UISlider : public UIElement
{
public:
    UISlider(UIAnchor anchor, const WindowPos &offsets, const T &min, const T &max,
        Getter<T> getter, Setter<T> setter, TextGetter text, const std::wstring &label);
    UISlider(UIAnchor, UIElement *other, int padding, const T &min, const T &max,
        Getter<T> getter, Setter<T> setter, TextGetter text, const std::wstring &label);

    virtual void on_hover_start(const WindowPos &pos) override;
    virtual void on_hover_end(const WindowPos &pos) override;
    virtual void on_mouse_pressed(MouseButtons button) override;
    virtual void on_mouse_released(MouseButtons button) override;
    virtual void on_focus(const WindowPos &pos) override;

    virtual bool in_bounding_box(const WindowPos &pos) const override;
    virtual bool in_element_specific_space(const WindowPos &pos) const override;

    virtual void draw_sprite() override;
    virtual void draw_text() override;

    virtual const WindowSize &get_element_size() const override;

    virtual void update_position(const WindowSize &size) override;

    void update_knob_position();
    
protected:
    T min;
    T max;
    Getter<T> getter;
    Setter<T> setter;
    TextGetter text;
    UITextObject display;
    UITextObject label;
    int clickOffset;
    UIBoundingBox knobBox;
    UIBoundingBox barBox;
    UIBoundingBox textBox;
    UIColorHighlighter barColor;
    UIColorHighlighter knobColor;
};

template<typename T>
inline UISlider<T>::UISlider(UIAnchor anchor, const WindowPos &offsets, const T &min,
    const T &max, Getter<T> getter, Setter<T> setter, TextGetter text, const std::wstring &label) :
    UIElement(anchor, offsets, get_element_size()), min(min), max(max),
    getter(getter), setter(setter), text(text), display(UIAnchor::RIGHT, text()),
    label(UIAnchor::LEFT, label), clickOffset(0), barColor(noninteractableBaseColor,
        noninteractableHoverColor), knobColor(interactableBaseColor, interactableHoverColor)
{
    auto topHalf = box.center();
    auto bottomHalf = topHalf;

    topHalf.y -= 16;
    bottomHalf.y += 16;

    knobBox = UIBoundingBox(bottomHalf, WindowSize { 32, 32 });
    barBox = UIBoundingBox(bottomHalf, WindowSize { 128, 32 });
    textBox = UIBoundingBox(topHalf, WindowSize { 128, 32 });

    update_knob_position();
}

template<typename T>
inline UISlider<T>::UISlider(UIAnchor anchor, UIElement *other, int padding, const T &min,
    const T &max, Getter<T> getter, Setter<T> setter, TextGetter text, const std::wstring &label) :
    UIElement(anchor, other, padding, get_element_size()), min(min), max(max),
    getter(getter), setter(setter), text(text), display(UIAnchor::RIGHT, text()),
    label(UIAnchor::LEFT, label), clickOffset(0), barColor(noninteractableBaseColor,
        noninteractableHoverColor), knobColor(interactableBaseColor, interactableHoverColor)
{
    auto topHalf = box.center();
    auto bottomHalf = topHalf;

    topHalf.y -= 16;
    bottomHalf.y += 16;

    knobBox = UIBoundingBox(bottomHalf, WindowSize { 32, 32 });
    barBox = UIBoundingBox(bottomHalf, WindowSize { 128, 32 });
    textBox = UIBoundingBox(topHalf, WindowSize { 128, 32 });

    update_knob_position();
}

template<typename T>
inline void UISlider<T>::on_hover_start(const WindowPos &pos)
{
    barColor.start_highlighting();
    knobColor.start_highlighting();

    // make it so we are set to receive mouse input
    ui->start_consuming_mouse_input();
}

template<typename T>
inline void UISlider<T>::on_hover_end(const WindowPos &pos)
{
    barColor.stop_highlighting();
    knobColor.stop_highlighting();

    ui->stop_consuming_mouse_input();
}

template<typename T>
inline void UISlider<T>::on_mouse_pressed(MouseButtons button)
{
    if (button == MouseButtons::LEFT)
    {
        if (ui->acquire_focus(this) == true)
        {
            const auto &mousePos = InputHandler::get_mouse_position();
            clickOffset = mousePos.x - knobBox.center().x;
        }
    }
    else if (button == MouseButtons::RIGHT)
    {
        int xPos = knobBox.center().x;
        xPos -= barBox.get_bounds().left;
        
        const auto size = barBox.get_size();

        float t = static_cast<float>(xPos) / static_cast<float>(size.width);

        if (t < 0.25f)
        {
            t = 0.0f;
        }
        else if (t < 0.5f)
        {
            t = 0.25f;
        }
        else if (t < 0.75f)
        {
            t = 0.5f;
        }
        else if (t < 1.0f)
        {
            t = 0.75f;
        }
        else
        {
            t = -0.25f;
        }

        t += 0.25f;

        const T val = min + static_cast<T>((max - min) * t);

        setter(val);

        const int currY = knobBox.center().y;
        xPos = barBox.get_bounds().left + static_cast<int>(t * size.width);

        knobBox.move_to(WindowPos { xPos, currY });
    }
}

template<typename T>
inline void UISlider<T>::on_mouse_released(MouseButtons button)
{
    if (button == MouseButtons::LEFT)
    {
        ui->relinquish_focus(this);
    }
}

template<typename T>
inline void UISlider<T>::on_focus(const WindowPos &pos)
{
    // offset the mouse's current position from where we were clicked on
    int xPos = pos.x - clickOffset;

    // clamp that position to the bounds of the bar
    xPos = std::max(static_cast<int>(box.get_bounds().left), xPos);
    xPos = std::min(static_cast<int>(box.get_bounds().right), xPos);

    // go ahead and move the knob to that position
    const int currY = knobBox.center().y;
    knobBox.move_to(WindowPos { xPos, currY });

    // transfer that position into the local space of the bar
    xPos -= barBox.get_bounds().left;

    // determine how far along the bar that puts us
    const float t = static_cast<float>(xPos) / static_cast<float>(barBox.get_size().width);

    // determine the new value
    const T val = min + static_cast<T>((max - min) * t);

    // and set it
    setter(val);
}

template<typename T>
inline bool UISlider<T>::in_bounding_box(const WindowPos &pos) const
{
    return knobBox.contains(pos);
}

template<typename T>
inline bool UISlider<T>::in_element_specific_space(const WindowPos &pos) const
{
    return knobBox.contains(pos);
}

template<typename T>
inline void UISlider<T>::draw_sprite()
{
    renderer->get_ui_sprite_renderer().draw_slider(barBox, knobBox, barColor.get_color(), knobColor.get_color());
}

template<typename T>
inline void UISlider<T>::draw_text()
{
    display.set_text(text());
    display.draw(textBox);

    label.draw(textBox);
}

template<typename T>
inline const WindowSize &UISlider<T>::get_element_size() const
{
    static const WindowSize size { 128, 64 };
    return size;
}

template<typename T>
inline void UISlider<T>::update_position(const WindowSize &size)
{
    const auto currCenter = box.center();

    UIElement::update_position(size);

    const auto delta = box.center() - currCenter;

    textBox.offset(delta);
    barBox.offset(delta);

    update_knob_position();
}

template<typename T>
inline void UISlider<T>::update_knob_position()
{
    const float t = static_cast<float>(getter() - min) / static_cast<float>(max - min);

    const int xPos = box.get_bounds().left + static_cast<int>(t * box.get_size().width);

    knobBox.move_to(WindowPos { xPos, barBox.center().y });
}
