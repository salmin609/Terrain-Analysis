/******************************************************************************/
/*!
\file		BehaviorTreePrototype.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Stores the structure of behavior trees read from files

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "BehaviorTreePrototype.h"
#include "BehaviorTree.h"
#include "BehaviorNode.h"
#include "Agent/BehaviorAgent.h"

BehaviorTreePrototype::BehaviorTreePrototype() : rootNode{NodeTypes::NUM_ENTRIES, 0}, previousNode(nullptr), treeName(nullptr)
{}

void BehaviorTreePrototype::build_tree(BehaviorAgent *agent)
{
    // set the root node of the tree first
    auto &tree = agent->get_behavior_tree();
    tree.treeName = treeName;
    tree.rootNode = node_type_to_node(rootNode.type);
    tree.rootNode->set_debug_info(node_type_to_name(rootNode.type), node_type_to_summary(rootNode.type));
    tree.rootNode->agent = agent;

    // fill out the tree
    recursive_add(rootNode, tree.rootNode);
}

void BehaviorTreePrototype::add_node(NodeTypes type, int depth)
{
    if (depth == 0)
    {
        rootNode.type = type;
        previousNode = &rootNode;
    }
    else
    {
        // assuming that the behavior tree tool has verified depths, if file was manually edited all bets are off
        if (depth > previousNode->depth)
        {
            // just insert the node into the previous node and continue the chain
            previousNode->children.emplace_back(type, depth);

            previousNode = &previousNode->children.back();
        }
        else // starting a new branch of the tree
        {
            // find where the new branch starts
            const int parentDepth = depth - 1;

            previousNode = &rootNode;

            while (previousNode->depth != parentDepth)
            {
                previousNode = &previousNode->children.back();
            }

            // insert the node
            previousNode->children.emplace_back(type, depth);

            previousNode = &previousNode->children.back();
        }
    }
}

void BehaviorTreePrototype::set_tree_name(const char * name)
{
    treeName = name;
}

void BehaviorTreePrototype::recursive_add(NodeRep &rep, BehaviorNode *node)
{
    for (auto && child : rep.children)
    {
        auto childNode = node_type_to_node(child.type);
        node->add_child(childNode);

        // set the debug info
        childNode->set_debug_info(node_type_to_name(child.type), node_type_to_summary(child.type));

        recursive_add(child, childNode);
    }
}

BehaviorTreePrototype::NodeRep::NodeRep(NodeTypes type, int depth) : type(type), depth(depth)
{}
