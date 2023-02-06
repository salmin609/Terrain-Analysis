/******************************************************************************/
/*!
\file		UIBoundingBox.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Convenience class for converting between different bounding box representations

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include "Misc/NiceTypes.h"

// just a convenience type to bridge the gap between RECTs and Rects for DirectX and Direct2D
class UIBoundingBox
{
public:
    UIBoundingBox();
    UIBoundingBox(const WindowPos &pos, const WindowSize &size);

    operator RECT() const;
    operator Rect() const;
    operator D2D1_RECT_F() const;
    operator Vec2() const;
    operator WindowPos() const;

    UIBoundingBox &operator=(const UIBoundingBox &rhs);

    bool contains(const WindowPos &pos) const;
    const WindowPos &center() const;

    void offset(const WindowSize &size);
    void offset(const WindowPos &off);
    void move_to(const WindowPos &pos);

    const RECT &get_bounds() const;
    WindowSize get_size() const;
private:
    WindowPos pos;
    RECT bounds;
};