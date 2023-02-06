/******************************************************************************/
/*!
\file		UIBoundingBox.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Convenience class for converting between different bounding box representations

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "UIBoundingBox.h"

UIBoundingBox::UIBoundingBox() : pos { 0, 0 }
{}

UIBoundingBox::UIBoundingBox(const WindowPos &pos, const WindowSize &size) : pos(pos)
{
    const auto halfWidth = size.width >> 1;
    const auto halfHeight = size.height >> 1;
    bounds.left = pos.x - halfWidth;
    bounds.right = pos.x + halfWidth;
    bounds.top = pos.y - halfHeight;
    bounds.bottom = pos.y + halfHeight;
}

UIBoundingBox::operator RECT() const
{
    return bounds;
}

UIBoundingBox::operator Rect() const
{
    return Rect(bounds);
}

UIBoundingBox::operator D2D1_RECT_F() const
{
    return D2D1_RECT_F { static_cast<float>(bounds.left), static_cast<float>(bounds.top),
        static_cast<float>(bounds.right), static_cast<float>(bounds.bottom) };
}

UIBoundingBox::operator Vec2() const
{
    return Vec2(static_cast<float>(pos.x), static_cast<float>(pos.y));
}

UIBoundingBox::operator WindowPos() const
{
    return pos;
}

UIBoundingBox &UIBoundingBox::operator=(const UIBoundingBox &rhs)
{
    pos.x = rhs.pos.x;
    pos.y = rhs.pos.y;
    bounds = rhs.bounds;

    return *this;
}

bool UIBoundingBox::contains(const WindowPos &pos) const
{
    return pos.x >= bounds.left && pos.x <= bounds.right &&
        pos.y >= bounds.top && pos.y <= bounds.bottom;
}

const WindowPos &UIBoundingBox::center() const
{
    return pos;
}

void UIBoundingBox::offset(const WindowSize &size)
{
    pos.x += size.width;
    pos.y += size.height;
    bounds.left += size.width;
    bounds.right += size.width;
    bounds.top += size.height;
    bounds.bottom += size.height;
}

void UIBoundingBox::offset(const WindowPos &off)
{
    pos.x += off.x;
    pos.y += off.y;
    bounds.left += off.x;
    bounds.right += off.x;
    bounds.top += off.y;
    bounds.bottom += off.y;
}

void UIBoundingBox::move_to(const WindowPos &p)
{
    const auto xDelta = p.x - pos.x;
    const auto yDelta = p.y - pos.y;

    pos = p;

    bounds.left += xDelta;
    bounds.right += xDelta;
    bounds.top += yDelta;
    bounds.bottom += yDelta;
}

const RECT &UIBoundingBox::get_bounds() const
{
    return bounds;
}

WindowSize UIBoundingBox::get_size() const
{
    return WindowSize { bounds.right - bounds.left, bounds.bottom - bounds.top };
}
