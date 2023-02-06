/******************************************************************************/
/*!
\file		ProjectTwo.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Specification for project two - pathfinding

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include "Project.h"
#include "../Student/Project_2/P2_Pathfinding.h"
#include "Agent/AStarAgent.h"
#include "Testing/PathingTester.h"

class ProjectTwo final : public Project
{
public:
    virtual bool initialize() override final;
    virtual bool finalize() override final;
    virtual void shutdown() override final;

    virtual void draw_meshes() override final;
    virtual void draw_sprites() override final;
    virtual void draw_text() override final;
    virtual void draw_debug() override final;

    virtual void update() override final;

    static bool implemented_floyd_warshall();
    static bool implemented_goal_bounding();
    static bool implemented_jps_plus();

private:
    AStarAgent *agent;
    PathTester tester;
    bool testRunning;

    void build_ui();
    void link_input();

    void on_left_mouse_click();
    void on_f1();
    void on_f3();

    void on_test_begin();
    void on_test_end();
};