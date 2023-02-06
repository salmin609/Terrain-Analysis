/******************************************************************************/
/*!
\file		Project.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Base project

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>

#if PROJECT_ONE
    const Project::Type Project::defaultProject = Project::Type::ONE;
#elif PROJECT_TWO
    const Project::Type Project::defaultProject = Project::Type::TWO;
#elif PROJECT_THREE
    const Project::Type Project::defaultProject = Project::Type::THREE;
#endif