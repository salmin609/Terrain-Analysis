/******************************************************************************/
/*!
\file		ReadShader.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Helper function for reading shader files

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include "ReadShader.h"
#include <pch.h>
#include <fstream>

std::vector<uint8_t> read_shader(const wchar_t * filename)
{
    std::ifstream inFile(filename, std::ios::in | std::ios::binary | std::ios::ate);
    if (!inFile)
        throw std::exception("read_shader: open");

    std::streampos len = inFile.tellg();
    if (!inFile)
        throw std::exception("read_shader: length");

    std::vector<uint8_t> blob;
    blob.resize(static_cast<size_t>(len));

    inFile.seekg(0, std::ios::beg);
    if (!inFile)
        throw std::exception("read_shader: seek");

    inFile.read(reinterpret_cast<char *>(blob.data()), len);
    if (!inFile)
        throw std::exception("read_shader: read");

    inFile.close();

    return blob;
}
