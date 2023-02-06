/******************************************************************************/
/*!
\file		UITextObject.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Helper class for displaying text in the UI

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "UITextObject.h"
#include <iostream>

UITextObject::UITextObject(UIAnchor anchor, const std::wstring &text) :
    anchor(anchor), text(text)
{}

UITextObject::UITextObject(UIAnchor anchor, const wchar_t *text) :
    anchor(anchor), text(text)
{}

void UITextObject::set_text(const std::wstring &newText)
{
    text = newText;
}

void UITextObject::set_text(const wchar_t *newText)
{
    text = newText;
}

void UITextObject::draw(const UIBoundingBox &bounds, TextColor color)
{
    renderer->get_text_renderer().draw_text(text, anchor, bounds, color);
}


