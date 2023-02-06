/******************************************************************************/
/*!
\file		ReadShader.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Helper function for reading shader files

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include <vector>

std::vector<uint8_t> read_shader(const wchar_t *filename);