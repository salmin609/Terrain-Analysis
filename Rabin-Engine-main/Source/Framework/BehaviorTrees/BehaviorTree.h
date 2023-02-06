/******************************************************************************/
/*!
\file		BehaviorTree.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Behavior tree declarations

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once

// forward declaration
class BehaviorNode;
class BehaviorTreePrototype;

class BehaviorTree
{
    friend class BehaviorTreePrototype;
public:
    BehaviorTree();
    ~BehaviorTree();

    void update(float dt);

    const char *get_tree_name() const;
private:
    BehaviorNode *rootNode;
    const char *treeName;
};