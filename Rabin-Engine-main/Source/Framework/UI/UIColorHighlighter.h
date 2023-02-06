/******************************************************************************/
/*!
\file		UIColorHighligher.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Helper class to adjust ui element colors

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include "Misc/NiceTypes.h"

class UIColorHighlighter
{
public:
    UIColorHighlighter(const Color &base, const Color &highlighted, int duration = 60, int inc = 4, int dec = -1);

    void start_highlighting();
    void stop_highlighting();

    void change_colors(const Color &base, const Color &highlighted);

    Color get_color();
private:
    Color base;
    Color highlighted;
    int adjuster;
    int current;
    int maxTicks;
    int incAmount;
    int decAmount;
};