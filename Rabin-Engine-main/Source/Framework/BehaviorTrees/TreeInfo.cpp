/******************************************************************************/
/*!
\file		TreeInfo.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Helper functions for behavior tree details

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "TreeInfo.h"

BehaviorTreeTypes string_to_behavior_tree_type(const char *name)
{
    for (size_t i = 0; i < static_cast<size_t>(BehaviorTreeTypes::NUM_ENTRIES); ++i)
    {
        if (std::strcmp(behaviorTreeNames[i], name) == 0)
        {
            return static_cast<BehaviorTreeTypes>(i);
        }
    }

    return BehaviorTreeTypes::NUM_ENTRIES;
}

const char *tree_type_to_tree_name(BehaviorTreeTypes type)
{
    return behaviorTreeNames[static_cast<size_t>(type)];
}