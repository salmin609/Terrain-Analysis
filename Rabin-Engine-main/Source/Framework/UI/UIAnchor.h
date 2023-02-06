/******************************************************************************/
/*!
\file		UIAnchor.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Enumeration for ui element anchoring

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once

// separated out to avoid circular include
enum class UIAnchor
{
    TOP_LEFT,
    TOP,
    TOP_RIGHT,
    LEFT,
    CENTER,
    RIGHT,
    BOTTOM_LEFT,
    BOTTOM,
    BOTTOM_RIGHT
};
