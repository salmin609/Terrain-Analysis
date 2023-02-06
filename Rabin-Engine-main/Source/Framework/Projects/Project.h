/******************************************************************************/
/*!
\file		Project.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Base project

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once

class Project
{
public:
    virtual bool initialize() = 0;
    virtual bool finalize() = 0;
    virtual void shutdown() = 0;

    virtual void draw_meshes() = 0;
    virtual void draw_sprites() = 0;
    virtual void draw_text() = 0;
    virtual void draw_debug() = 0;

    virtual void update() = 0;

    enum class Type
    {
        ONE,
        TWO,
        THREE
    };

    static const Type defaultProject;
};