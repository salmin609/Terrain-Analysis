/******************************************************************************/
/*!
\file		BehaviorTreeBuilder.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Builds behavior trees from prototypes

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include "TreeInfo.h"
#include "BehaviorTreePrototype.h"
#include <vector>

class BehaviorTreeBuilder
{
public:
    bool initialize();
    void shutdown();

    void build_tree(BehaviorTreeTypes type, BehaviorAgent *agent);

private:
    std::vector<BehaviorTreePrototype> prototypes;

    bool deserialize_tree(const std::filesystem::path &filepath);

    BehaviorTreeTypes deserialize_tree_type(FILE *file);
    bool deserialize_node(FILE *file, NodeTypes &type, int &depth);
};