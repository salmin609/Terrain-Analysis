/******************************************************************************/
/*!
\file		RNG.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Simple random number generator

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "RNG.h"

std::mt19937_64 RNG::generator;

void RNG::seed(unsigned seed)
{
    generator.seed(seed);
}

void RNG::seed()
{
    std::random_device seed;
    generator.seed(seed());
}

bool RNG::coin_toss()
{
    const std::bernoulli_distribution dist;

    return dist(generator);
}

unsigned RNG::d2()
{
    const std::uniform_int_distribution<unsigned> dist(1, 2);
    
    return dist(generator);
}

unsigned RNG::d3()
{
    const std::uniform_int_distribution<unsigned> dist(1, 3);

    return dist(generator);
}

unsigned RNG::d4()
{
    const std::uniform_int_distribution<unsigned> dist(1, 4);

    return dist(generator);
}

unsigned RNG::d6()
{
    const std::uniform_int_distribution<unsigned> dist(1, 6);

    return dist(generator);
}

unsigned RNG::d8()
{
    const std::uniform_int_distribution<unsigned> dist(1, 8);

    return dist(generator);
}

unsigned RNG::d10()
{
    const std::uniform_int_distribution<unsigned> dist(1, 10);

    return dist(generator);
}

unsigned RNG::d12()
{
    const std::uniform_int_distribution<unsigned> dist(1, 12);

    return dist(generator);
}

unsigned RNG::d20()
{
    const std::uniform_int_distribution<unsigned> dist(1, 20);

    return dist(generator);
}

unsigned RNG::d100()
{
    const std::uniform_int_distribution<unsigned> dist(1, 100);

    return dist(generator);
}

Vec2 RNG::unit_vector_2D()
{
    const std::uniform_real_distribution<float> dist(0.0f, PI);

    const float azimuth = dist(generator);

    return Vec2(std::cos(azimuth), std::sin(azimuth));
}

Vec3 RNG::unit_vector_3D()
{
    const std::uniform_real_distribution<float> dist(-1.0f, 1.0f);

    const float z = dist(generator);

    const Vec2 planar = unit_vector_2D() * std::sqrt(1.0f - z * z);

    return Vec3(planar.x, planar.y, z);
}

Color RNG::color(float alpha)
{
    const std::uniform_real_distribution<float> dist(0.0f, 1.0f);

    return Color(dist(generator), dist(generator), dist(generator), alpha);
}

Vec3 RNG::world_position()
{
    const std::uniform_real_distribution<float> dist(0.0f, terrain->mapSizeInWorld);
    
    return Vec3(dist(generator), 0.0f, dist(generator));
}
