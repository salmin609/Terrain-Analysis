/******************************************************************************/
/*!
\file		Murmur2Hash.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Murmur hash wrapper

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once

#ifdef _WIN64
    const size_t MURMUR2_HASH_SEED = 0xBEEFBABEB055FACE;
#else
    const size_t MURMUR2_HASH_SEED = 0xBEEFBABE;
#endif

//-----------------------------------------------------------------------------
// MurmurHash2 was written by Austin Appleby, and is placed in the public
// domain. The author hereby disclaims copyright to this source code.
size_t MurmurHash(const void * key, size_t len, size_t seed = MURMUR2_HASH_SEED);
