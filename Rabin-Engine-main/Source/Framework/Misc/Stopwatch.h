/******************************************************************************/
/*!
\file		Stopwatch.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Simple timer

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include <chrono>

// wrapper around chrono high res clock, which is wrapper around windows QPC
// just making it easier to use and read
class Stopwatch
{
    using hrc = std::chrono::high_resolution_clock;
    hrc::time_point started;
    hrc::duration delta;
public:
    void start();
    void stop();
    static hrc::time_point now();
    std::chrono::nanoseconds nanoseconds() const;
    std::chrono::microseconds microseconds() const;
    std::chrono::milliseconds milliseconds() const;
    std::chrono::seconds seconds() const;
};