/******************************************************************************/
/*!
\file		Murmur2Hash.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Murmur hash wrapper

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "Murmur2Hash.h"


//-----------------------------------------------------------------------------
// MurmurHash2, 64-bit versions, by Austin Appleby

// The same caveats as 32-bit MurmurHash2 apply here - beware of alignment 
// and endian-ness issues if used across multiple platforms.

// 64-bit hash for 64-bit platforms
#ifdef _WIN64
    size_t MurmurHash(const void * key, size_t len, size_t seed)
    {
        const size_t m = 0xc6a4a7935bd1e995;
        const int r = 47;

        size_t h = seed ^ (len * m);

        const size_t * data = (const size_t *)key;
        const size_t * end = data + (len / 8);

        while (data != end)
        {
            size_t k = *data++;

            k *= m;
            k ^= k >> r;
            k *= m;

            h ^= k;
            h *= m;
        }

        const unsigned char * data2 = (const unsigned char*)data;

        switch (len & 7)
        {
        case 7: h ^= size_t(data2[6]) << 48;
        case 6: h ^= size_t(data2[5]) << 40;
        case 5: h ^= size_t(data2[4]) << 32;
        case 4: h ^= size_t(data2[3]) << 24;
        case 3: h ^= size_t(data2[2]) << 16;
        case 2: h ^= size_t(data2[1]) << 8;
        case 1: h ^= size_t(data2[0]);
            h *= m;
        };

        h ^= h >> r;
        h *= m;
        h ^= h >> r;

        return h;
}
#else

    size_t MurmurHash(const void * key, size_t len, size_t seed)
    {
        // 'm' and 'r' are mixing constants generated offline.
        // They're not really 'magic', they just happen to work well.

        const size_t m = 0x5bd1e995;
        const int r = 24;

        // Initialize the hash to a 'random' value

        size_t h = seed ^ len;

        // Mix 4 bytes at a time into the hash

        const unsigned char * data = (const unsigned char *)key;

        while (len >= 4)
        {
            size_t k = *(size_t *)data;

            k *= m;
            k ^= k >> r;
            k *= m;

            h *= m;
            h ^= k;

            data += 4;
            len -= 4;
        }

        // Handle the last few bytes of the input array

        switch (len)
        {
        case 3: h ^= data[2] << 16;
        case 2: h ^= data[1] << 8;
        case 1: h ^= data[0];
            h *= m;
        };

        // Do a few final mixes of the hash to ensure the last few
        // bytes are well-incorporated.

        h ^= h >> 13;
        h *= m;
        h ^= h >> 15;

        return h;
    }
#endif