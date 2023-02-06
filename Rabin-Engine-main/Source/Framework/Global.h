/******************************************************************************/
/*!
\file		Global.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Global variables and systems

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include <memory>

// which project to load on startup
#define PROJECT_ONE 0
#define PROJECT_TWO 0
#define PROJECT_THREE 1

// forward declarations
class Engine;
class SimpleRenderer;
class AgentOrganizer;
class Terrain;
class Project;
class AStarPather;
class UICoordinator;
class BehaviorTreeBuilder;
class AudioWrapper;

extern std::unique_ptr<Engine> engine;
extern std::unique_ptr<SimpleRenderer> renderer;
extern std::unique_ptr<AgentOrganizer> agents;
extern std::unique_ptr<Terrain> terrain;
extern std::unique_ptr<Project> project;
extern std::unique_ptr<AStarPather> pather;
extern std::unique_ptr<UICoordinator> ui;
extern std::unique_ptr<BehaviorTreeBuilder> treeBuilder;
extern std::unique_ptr<AudioWrapper> audio;

// scalar that is based on the size of the map, effectively desired world size / map size
extern float globalScalar;

extern float deltaTime;