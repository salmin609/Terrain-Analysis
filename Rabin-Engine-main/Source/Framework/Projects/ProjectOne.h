/******************************************************************************/
/*!
\file		ProjectOne.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Specification for project one - behavior trees

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include "Project.h"
#include "BehaviorTreeBuilder.h"

class ProjectOne final : public Project
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

private:
    void build_ui();
    void link_input();

    void on_f2();
    void on_f3();

    void setup();
};