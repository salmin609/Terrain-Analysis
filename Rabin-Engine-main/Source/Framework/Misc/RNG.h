/******************************************************************************/
/*!
\file		RNG.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Simple random number generator

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include <random>
#include "NiceTypes.h"

class RNG
{
public:
    static void seed(unsigned seed);
    static void seed();

    static bool coin_toss();
    static unsigned d2();
    static unsigned d3();
    static unsigned d4();
    static unsigned d6();
    static unsigned d8();
    static unsigned d10();
    static unsigned d12();
    static unsigned d20();
    static unsigned d100();

    template <typename T>
    static T range(T min, T max);

    static Vec2 unit_vector_2D();
    static Vec3 unit_vector_3D();
    static Color color(float alpha = 1.0f);
    static Vec3 world_position();

    template <typename T>
    static const T &element(const std::vector<T> &vec);

private:
    static std::mt19937_64 generator;

    // using hidden templates so the public range function has a nice and easy to understand signature
    template <typename T>
    static std::enable_if_t<std::is_integral<T>::value, T> typed_range(T min, T max);

    template <typename T>
    static std::enable_if_t<std::is_floating_point<T>::value, T> typed_range(T min, T max);
};


template<typename T>
inline T RNG::range(T min, T max)
{
    return typed_range(min, max);
}

template<typename T>
inline const T &RNG::element(const std::vector<T> &vec)
{
    if (vec.size() == 0)
    {
        __debugbreak();
    }

    const auto index = range(0, vec.size() - 1);
    return vec[index];
}

template<typename T>
inline std::enable_if_t<std::is_integral<T>::value, T> RNG::typed_range(T min, T max)
{
    using Type = std::uniform_int_distribution<T>;

    // guard against flipped values, or negative value confusion
    const Type dist = (min < max) ? Type(min, max) : Type(max, min);

    return dist(generator);
}

template <typename T>
inline std::enable_if_t<std::is_floating_point<T>::value, T> RNG::typed_range(T min, T max)
{
    using Type = std::uniform_real_distribution<T>;

    // guard against flipped values, or negative value confusion
    const Type dist = (min < max) ? Type(min, max) : Type(max, min);

    return dist(generator);
}

