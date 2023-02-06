/******************************************************************************/
/*!
\file		MapMath.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Simple math functions for common map operations

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include "../Misc/NiceTypes.h"

float lerp(float value0, float value1, float t);

// determines if two 2D lines intersect, each line is defined by two points
bool line_intersect(const Vec2 &line0P0, const Vec2 &line0P1, const Vec2 &line1P0, const Vec2 &line1P1);