/******************************************************************************/
/*!
\file		Messages.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Engine message enumerations

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once

enum class Messages
{
    PATH_REQUEST_BEGIN,
    PATH_REQUEST_TICK_START,
    PATH_REQUEST_TICK_FINISH,
    PATH_REQUEST_END,

    ANALYSIS_BEGIN,
    ANALYSIS_TICK_START,
    ANALYSIS_TICK_FINISH,
    ANALYSIS_END,

    PATH_TEST_BEGIN,
    PATH_TEST_END,

    MAP_CHANGE,

    NUM_ENTRIES
};