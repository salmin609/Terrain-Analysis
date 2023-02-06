/******************************************************************************/
/*!
\file		CameraAgent.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Simple agent to use as camera declarations

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include "Agent.h"

class CameraAgent final : public Agent
{
public:
    CameraAgent();

    Mat4 get_view_matrix() const;
    void set_position_to_default();

    static const char *cameraTypeName;
private:
};