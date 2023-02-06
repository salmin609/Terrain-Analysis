/******************************************************************************/
/*!
\file		Engine.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Engine declarations

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include "StepTimer.h"
#include "Projects/Project.h"

class Engine
{
public:

    Engine();

    // Initialization and management
    bool initialize(HINSTANCE hInstance, int nCmdShow);
    void shutdown();

    void stop_engine();

    void finalize_project();

    // Basic loop
    void tick();

    // Messages
    void on_activated();
    void on_deactivated();
    void on_suspending();
    void on_resuming();

    void change_projects(Project::Type project);

    const std::wstring &get_fps_text();

    DX::StepTimer &get_timer();

    bool should_pause_on_exit() const;
private:
    DX::StepTimer timer;
    Project::Type projectType;
    std::wstring fpsText;
    bool shouldUpdate;
    bool shouldRender;
    bool projectChange;
    bool pauseOnExit;

    void Update(DX::StepTimer const& timer);
    bool swap_projects();
    bool allocate_project();
};