/******************************************************************************/
/*!
\file		MapMath.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Simple math functions for common map operations

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "MapMath.h"

float lerp(float value0, float value1, float t)
{
    return ((1.0f - t) * value0 + t * value1);
}

bool line_intersect(const Vec2 & line0P0, const Vec2 & line0P1, const Vec2 & line1P0, const Vec2 & line1P1)
{
    const float y4y3 = line1P1.y - line1P0.y;
    const float y1y3 = line0P0.y - line1P0.y;
    const float y2y1 = line0P1.y - line0P0.y;
    const float x4x3 = line1P1.x - line1P0.x;
    const float x2x1 = line0P1.x - line0P0.x;
    const float x1x3 = line0P0.x - line1P0.x;

    const float divisor = y4y3 * x2x1 - x4x3 * y2y1;
    const float dividend0 = x4x3 * y1y3 - y4y3 * x1x3;
    const float dividend1 = x2x1 * y1y3 - y2y1 * x1x3;

    const float eps = 0.0001f;
    if (std::abs(dividend0) < eps && std::abs(dividend1) < eps && std::abs(divisor) < eps)
    {	// Lines coincident (on top of each other)
        return true;
    }

    if (std::abs(divisor) < eps)
    {	// Lines parallel
        return false;
    }

    const float quotient0 = dividend0 / divisor;
    const float quotient1 = dividend1 / divisor;

    if (quotient0 < 0.0f || quotient0 > 1.0f || quotient1 < 0.0f || quotient1 > 1.0f)
    {	// No intersection
        return false;
    }
    else
    {	// Intersection
        return true;
    }
}
