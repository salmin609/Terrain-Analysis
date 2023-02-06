/******************************************************************************/
/*!
\file		Stopwatch.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Simple timer

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "Stopwatch.h"

void Stopwatch::start()
{
    started = hrc::now();
}

void Stopwatch::stop()
{
    const auto stopped = hrc::now();

    delta = stopped - started;
}

Stopwatch::hrc::time_point Stopwatch::now()
{
    return hrc::now();
}

std::chrono::nanoseconds Stopwatch::nanoseconds() const
{
    return delta;
}

std::chrono::microseconds Stopwatch::microseconds() const
{
    return std::chrono::duration_cast<std::chrono::microseconds>(delta);
}

std::chrono::milliseconds Stopwatch::milliseconds() const
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(delta);
}

std::chrono::seconds Stopwatch::seconds() const
{
    return std::chrono::duration_cast<std::chrono::seconds>(delta);
}
