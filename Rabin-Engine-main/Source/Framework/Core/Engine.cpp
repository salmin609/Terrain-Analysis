/******************************************************************************/
/*!
\file		Engine.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Engine implementation

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "Projects/ProjectOne.h"
#include "Projects/ProjectTwo.h"
#include "Projects/ProjectThree.h"
#include "Serialization.h"

// instantiate the global system pointers
std::unique_ptr<SimpleRenderer> renderer;
std::unique_ptr<Project> project;
std::unique_ptr<Terrain> terrain;
std::unique_ptr<AgentOrganizer> agents;
std::unique_ptr<UICoordinator> ui;
std::unique_ptr<AStarPather> pather;
std::unique_ptr<BehaviorTreeBuilder> treeBuilder;

float deltaTime = 0.16f;

Engine::Engine() : shouldUpdate(true), shouldRender(true), projectChange(false), pauseOnExit(false)
{}

bool Engine::initialize(HINSTANCE hInstance, int nCmdShow)
{
    RNG::seed();

    renderer = std::make_unique<SimpleRenderer>();

    projectType = Project::defaultProject;

    //timer.SetFixedTimeStep(true);
    //timer.SetTargetElapsedSeconds(1.0 / 60);

    bool result = Serialization::initialize() &&
        renderer->initialize(hInstance, nCmdShow) &&
        allocate_project() &&
        project->initialize();

    if (result == false)
    {
        pauseOnExit = true;
    }

    return result;
}

void Engine::shutdown()
{
    project->shutdown();
    project.reset();

    renderer->shutdown();
    renderer.reset();
}

void Engine::stop_engine()
{
    std::cout << "Stopping engine" << std::endl;
    shouldUpdate = false;
    shouldRender = false;
    PostQuitMessage(0);
}

void Engine::finalize_project()
{
    project->finalize();
}

// Executes the basic game loop.
void Engine::tick()
{
    if (shouldUpdate == true)
    {
        timer.Tick([&]()
        {
            Update(timer);
        });
    }

    // don't try to render anything before the first Update or when minimized/suspended
    if (timer.GetFrameCount() != 0 && shouldRender == true)
    {
        renderer->draw();
    }

    // see if a project change was requested
    if (projectChange == true)
    {
        if (swap_projects() == false)
        {
            pauseOnExit = true;
            stop_engine();
        }
    }

    fpsText = std::to_wstring(timer.GetFramesPerSecond());
}

// Updates the world.
void Engine::Update(DX::StepTimer const& timer)
{
    deltaTime = float(timer.GetElapsedSeconds());

    project->update();
}

bool Engine::swap_projects()
{
    projectChange = false;

    // shutdown the current project
    project->shutdown();

    // free the memory
    project.reset();

    // allocate the new project
    if (allocate_project() == false)
    {
        return false;
    }

    InputHandler::forced_reset();
    Messenger::clear_all_listeners();

    // allocate, initialize, and finalize
    return allocate_project() &&
        project->initialize() &&
        project->finalize();
}

bool Engine::allocate_project()
{
    try
    {
        switch (projectType)
        {
        case Project::Type::ONE:
            project = std::make_unique<ProjectOne>();
            break;
        case Project::Type::TWO:
            project = std::make_unique<ProjectTwo>();
            break;
        case Project::Type::THREE:
            project = std::make_unique<ProjectThree>();
            break;
        }
    }
    catch (const std::exception &err)
    {
        std::cout << "Failed to allocate project: " << err.what() << std::endl;
        return false;
    }

    return true;
}


// Message handlers
void Engine::on_activated()
{
    // becoming the active window
    //shouldUpdate = true;

}

void Engine::on_deactivated()
{
    // becoming background window
    //shouldUpdate = false;
    InputHandler::reset_states();
}

void Engine::on_suspending()
{
    // being suspended or minimized
    shouldRender = false;
    shouldUpdate = false;
    InputHandler::reset_states();
}

void Engine::on_resuming()
{
    // being resumed or returned from minimized
    timer.ResetElapsedTime();
    shouldRender = true;
    shouldUpdate = false;
}

void Engine::change_projects(Project::Type proj)
{
    projectType = proj;
    projectChange = true;
}

const std::wstring &Engine::get_fps_text()
{
    return fpsText;
}

DX::StepTimer &Engine::get_timer()
{
    return timer;
}

bool Engine::should_pause_on_exit() const
{
    return pauseOnExit;
}
