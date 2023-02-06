/******************************************************************************/
/*!
\file		PathfindingDetails.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Helper functions for displaying textual info

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "PathfindingDetails.hpp"

namespace
{
    const std::string heuristicText[] = { "Octile", "Chebyshev", "Manhattan", "Euclidean", "Invalid" };
    const std::string methodText[] = { "A*", "Floyd-Warshall", "JPS+", "Goal Bounding", "Invalid" };
}

const std::wstring &get_heuristic_wtext(Heuristic heuristic)
{
    static const std::wstring text[] = { L"Octile", L"Chebyshev", L"Manhattan", L"Euclidean", L"Invalid" };

    return text[static_cast<size_t>(heuristic)];
}

const std::string &get_heuristic_text(Heuristic heuristic)
{
    return heuristicText[static_cast<size_t>(heuristic)];
}

Heuristic string_to_heuristic(const std::string &string)
{
    for (size_t i = 0; i < static_cast<size_t>(Heuristic::NUM_ENTRIES); ++i)
    {
        if (heuristicText[i] == string)
        {
            return static_cast<Heuristic>(i);
        }
    }

    return Heuristic::NUM_ENTRIES;
}

const std::wstring &get_method_wtext(Method method)
{
    static const std::wstring text[] = { L"A*", L"Floyd-Warshall", L"JPS+", L"Goal Bounding", L"Invalid" };

    return text[static_cast<size_t>(method)];
}

const std::string &get_method_text(Method method)
{
    return methodText[static_cast<size_t>(method)];
}

Method string_to_method(const std::string &string)
{
    for (size_t i = 0; i < static_cast<size_t>(Method::NUM_ENTRIES); ++i)
    {
        if (methodText[i] == string)
        {
            return static_cast<Method>(i);
        }
    }

    return Method::NUM_ENTRIES;
}
