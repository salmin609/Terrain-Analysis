/******************************************************************************/
/*!
\file		NodeInfo.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Helper functions for behavior node details

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include "BehaviorNode.h"
#include "../Student/Project_1/NodeHeaders.h"

enum class NodeTypes
{
    #define REGISTER_CONTROLFLOW(Name, Summary) Name,
    #define REGISTER_DECORATOR(Name, Summary) Name,
    #define REGISTER_LEAF(Name, Summary) Name,

    //#include "../Student/Project_1/Nodes.def"
    #include "../Source/Student/Project_1/Nodes.def"
    NUM_ENTRIES

    #undef REGISTER_CONTROLFLOW
    #undef REGISTER_DECORATOR
    #undef REGISTER_LEAF
};

static const char *nodeNames[] =
{
    #define REGISTER_CONTROLFLOW(Name, Summary) #Name,
    #define REGISTER_DECORATOR(Name, Summary) #Name,
    #define REGISTER_LEAF(Name, Summary) #Name,

    #include "../Source/Student/Project_1/Nodes.def"
    "Invalid"

    #undef REGISTER_CONTROLFLOW
    #undef REGISTER_DECORATOR
    #undef REGISTER_LEAF
};

static const char *nodeSummaries[] =
{
    #define REGISTER_CONTROLFLOW(Name, Summary) Summary,
    #define REGISTER_DECORATOR(Name, Summary) Summary,
    #define REGISTER_LEAF(Name, Summary) Summary,

    #include "../Source/Student/Project_1/Nodes.def"
    "Invalid"

    #undef REGISTER_CONTROLFLOW
    #undef REGISTER_DECORATOR
    #undef REGISTER_LEAF
};

static BehaviorNode *nodePrototypes[] = 
{
    #define REGISTER_CONTROLFLOW(Name, Summary) new (Name)(),
    #define REGISTER_DECORATOR(Name, Summary) new (Name)(),
    #define REGISTER_LEAF(Name, Summary) new (Name)(),

    #include "../Source/Student/Project_1/Nodes.def"
    nullptr

    #undef REGISTER_CONTROLFLOW
    #undef REGISTER_DECORATOR
    #undef REGISTER_LEAF
};

BehaviorNode *node_type_to_node(NodeTypes type);

NodeTypes string_to_node_type(const char *nodeName);

const char *node_type_to_name(NodeTypes type);

const char *node_type_to_summary(NodeTypes type);