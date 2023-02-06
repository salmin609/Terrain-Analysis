/******************************************************************************/
/*!
\file		Serialization.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Serialization implementation

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "Serialization.h"
#include <rapidjson/filewritestream.h>
#include <rapidjson/filereadstream.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/error/en.h>
#include <iomanip>
#include <sstream>
#include <stdio.h>
#include "Projects/Testing/PathingTestCase.h"
#include "Projects/Testing/PathingTestResult.h"
#include "Projects/Testing/PathingTestData.h"
#include <time.h>

namespace rj = rapidjson;
namespace fs = std::filesystem;

namespace
{
    const size_t bufferSize = 65536;
    char buffer[bufferSize];
}

fs::path Serialization::basePath;
fs::path Serialization::mapsPath;
fs::path Serialization::testsPath;
fs::path Serialization::assetsPath;
fs::path Serialization::outputPath;
fs::path Serialization::behaviorTreePath;

bool Serialization::initialize()
{
    std::cout << "Initializing Serialization System..." << std::endl;

    /*
        Starting with the current path, ensure we find the directories we will need.
        Otherwise walk backward through the directory tree.  Running the exe from
        Visual Studio or the base project directory will always start in the correct
        place, but this will also handle launching the exe from any arbitrary build
        directory.
    */

    basePath = fs::current_path();
    const auto rootPath = basePath.root_path();

    // bound the loop with the root path, ie if we've hit C:\ we've gone too far Jim
    while (basePath != rootPath)
    {
        mapsPath = basePath / "Maps";
        testsPath = basePath / "Tests";
        assetsPath = basePath / "Assets";
        outputPath = basePath / "Output";
        behaviorTreePath = basePath / "Source" / "Student" / "Project_1" / "Trees";

        if (fs::exists(mapsPath) && fs::exists(testsPath) &&
            fs::exists(assetsPath) && 
            fs::exists(behaviorTreePath))
        {
            if (fs::exists(outputPath) == false)
            {
                if (fs::create_directory(outputPath) == false)
                {
                    return false;
                }
            }

            return true;
        }
        else
        {
            basePath = basePath.parent_path();
        }
    }

    return false;
}

void Serialization::generate_time_stamp(std::wstringstream &filename)
{
    const auto t = time(nullptr);
    tm l;
    localtime_s(&l, &t);

    static std::wstring prev;
    static int counter = 0;

    filename << std::put_time<wchar_t>(&l, L"%F-%H-%M-%S");

    if (filename.str() == prev)
    {
        ++counter;
    }
    else
    {
        counter = 0;
        prev = filename.str();
    }

    filename << L"_" << counter;

}

void Serialization::generate_time_stamp(std::stringstream &filename)
{
    const auto t = time(nullptr);
    tm l;
    localtime_s(&l, &t);

    static std::string prev;
    static int counter = 0;

    filename << std::put_time<char>(&l, "%F-%H-%M-%S");

    if (filename.str() == prev)
    {
        ++counter;
    }
    else
    {
        counter = 0;
        prev = filename.str();
    }

    filename << "_" << counter;
}

void Serialization::serialize_to(rapidjson::Value &val, const Terrain::MapData &map,
    rapidjson::MemoryPoolAllocator<> &allocator)
{
    val.SetArray();
    val.Reserve(static_cast<rj::SizeType>(map.height), allocator);

    for (const auto & row : map.data)
    {
        rj::Value rowData;
        rowData.SetArray();

        rowData.Reserve(static_cast<rj::SizeType>(map.width), allocator);

        for (const auto & cell : row)
        {
            rj::Value cellData(cell);

            rowData.PushBack(cellData, allocator);
        }

        val.PushBack(rowData, allocator);
    }
}

bool Serialization::deserialize_from(rapidjson::Value &val, Terrain::MapData &map, const std::wstring &id)
{
    // using a single boolean instead of returning on failure to allow for exposing multiple issues
    bool valid = true;

    if (val.IsArray() == false)
    {
        std::wcout << L"Map " << id << " Grid data is not an array" << std::endl;
        valid = false;
    }
    else if (val[0].IsArray() == false)
    {
        std::wcout << L"Map " << id << " Grid data is not a 2D array" << std::endl;
        valid = false;
    }
    else if (val[0][0].IsBool() == false)
    {
        std::wcout << L"Map " << id << " Grid data is not a 2D array of booleans" << std::endl;
        valid = false;
    }

    if (valid == false)
    {
        return false;
    }

    // pull the data out of the document and populate the map data
    const auto rows = val.GetArray();
    map.height = static_cast<int>(rows.Size());
    map.width = static_cast<int>(rows[0].Size());

    map.data.resize(map.height);

    for (int h = 0; h < map.height; ++h)
    {
        const auto row = rows[h].GetArray();

        // make sure the row has the correct number of elements
        if (row.Size() != map.width)
        {
            std::wcout << L"Map " << id << " Grid data has mismatched number of elements at row " << h <<
                ", expected " << map.width << " but row has " << row.Size() << std::endl;
            return false;
        }

        map.data[h].resize(map.width, false);

        for (int w = 0; w < map.width; ++w)
        {
            map.data[h][w] = row[w].GetBool();
        }
    }

    return true;
}

void Serialization::serialize_to(rapidjson::Value &val, const PathingTestCase &test,
    rapidjson::MemoryPoolAllocator<> &allocator)
{
    val.SetObject();

    rj::Value name;
    name.SetString(test.name, allocator);

    rj::Value settings;
    serialize_to(settings, test.settings, allocator);

    rj::Value results;
    results.SetArray();
    results.Reserve(static_cast<rj::SizeType>(test.expectedResults.size()), allocator);

    for (const auto & result : test.expectedResults)
    {
        rj::Value temp;
        serialize_to(temp, result, allocator);

        results.PushBack(temp, allocator);
    }

    val.AddMember("Name", name, allocator);
    val.AddMember("Settings", settings, allocator);
    val.AddMember("Expected Results", results, allocator);
}

bool Serialization::deserialize_from(rapidjson::Value &val, PathingTestCase &test, const std::wstring &id)
{
    const wchar_t *prefix = L"Test Case";

    if (val.IsObject() == false)
    {
        std::wcout << prefix << id << " is not an object" << std::endl;
        return false;
    }

    // using a single boolean instead of returning on failure to allow for exposing multiple issues
    bool valid = true;

    auto nameIter = val.FindMember("Name");
    auto settingsIter = val.FindMember("Settings");
    auto resultsIter = val.FindMember("Expected Results");

    if (extract_string(nameIter, val.MemberEnd(), test.name, prefix, L"Name", id) == false)
    {
        valid = false;
    }

    if (settingsIter == val.MemberEnd())
    {
        std::wcout << prefix << id << " does not have a Settings member" << std::endl;
        valid = false;
    }
    else if (deserialize_from(settingsIter->value, test.settings, id) == false)
    {
        valid = false;
    }


    if (resultsIter == val.MemberEnd())
    {
        std::wcout << L"Test Case " << id << " does not have an expected results member" << std::endl;
        valid = false;;
    }
    else if (resultsIter->value.IsArray() == false)
    {
        std::wcout << L"Test Case " << id << " expected results member is not an array" << std::endl;
        valid = false;
    }

    // top level of val is in correct state, begin deserializing each expected result
    if (valid == true)
    {
        test.name = nameIter->value.GetString();

        const auto &results = resultsIter->value.GetArray();
        auto size = results.Size();

        test.expectedResults.reserve(size);

        for (decltype(size) i = 0; i < size; ++i)
        {
            PathingTestData temp;
            const bool result = deserialize_from(results[i], temp, id);

            if (result == true)
            {
                test.expectedResults.emplace_back(std::move(temp));
            }
            else
            {
                valid = false;
                break;
            }
        }
    }

    return valid;
}

void Serialization::serialize_to(rapidjson::Value &val, const PathingTestData &data,
    rapidjson::MemoryPoolAllocator<> &allocator)
{
    val.SetObject();

    rj::Value mapValue;
    mapValue.SetInt(data.map);

    rj::Value startValue;
    serialize_to(startValue, data.start, allocator);

    rj::Value goalValue;
    serialize_to(goalValue, data.goal, allocator);

    rj::Value distCardValue;
    distCardValue.SetInt(data.distCard);

    rj::Value distDiagValue;
    distDiagValue.SetInt(data.distDiag);

    rj::Value solutionValue;
    solutionValue.SetBool(data.hasSolution);

    rj::Value visualValue;
    visualValue.SetBool(data.requiresVisualConfirmation);

    val.AddMember("Map", mapValue, allocator);
    val.AddMember("Start", startValue, allocator);
    val.AddMember("Goal", goalValue, allocator);
    val.AddMember("Cardinal Distance", distCardValue, allocator);
    val.AddMember("Diagonal Distance", distDiagValue, allocator);
    val.AddMember("Has Solution", solutionValue, allocator);
    val.AddMember("Requires Visual Confirmation", visualValue, allocator);

    if (data.message != nullptr)
    {
        rj::Value messageValue;
        messageValue.SetString(data.message, allocator);
        val.AddMember("Status", messageValue, allocator);
    }
}

bool Serialization::deserialize_from(rapidjson::Value &val, PathingTestData &data, const std::wstring &id)
{
    const wchar_t *prefix = L"Test Data ";

    if (val.IsObject() == false)
    {
        std::wcout << prefix << id << L" is not an object" << std::endl;
        return false;
    }

    // using a single boolean instead of returning on failure to allow for exposing multiple issues
    bool valid = true;

    auto mapIter = val.FindMember("Map");
    auto startIter = val.FindMember("Start");
    auto goalIter = val.FindMember("Goal");
    auto distCardIter = val.FindMember("Cardinal Distance");
    auto distDiagIter = val.FindMember("Diagonal Distance");
    auto solutionIter = val.FindMember("Has Solution");
    auto visualIter = val.FindMember("Requires Visual Confirmation");

    auto end = val.MemberEnd();

    if (extract_int(mapIter, end, data.map, prefix, L"Map", id) == false)
    {
        valid = false;
    }
    
    if (startIter == end)
    {
        std::wcout << prefix << id << L" does not have a Start member" << std::endl;
        valid = false;
    }
    else if (deserialize_from(startIter->value, data.start, id) == false)
    {
        valid = false;
    }

    if (goalIter == end)
    {
        std::wcout << prefix << id << L" does not have a Goal member" << std::endl;
        valid = false;
    }
    else if (deserialize_from(goalIter->value, data.goal, id) == false)
    {
        valid = false;
    }

    if (extract_int(distCardIter, end, data.distCard, prefix, L"Cardinal Distance", id) == false)
    {
        valid = false;
    }

    if (extract_int(distDiagIter, end, data.distDiag, prefix, L"Diagonal Distance", id) == false)
    {
        valid = false;
    }

    if (extract_bool(solutionIter, end, data.hasSolution, prefix, L"Has Solution", id) == false)
    {
        valid = false;
    }

    if (extract_bool(visualIter, end, data.requiresVisualConfirmation, prefix,
        L"Requires Visual Confirmation", id) == false)
    {
        valid = false;
    }

    return valid;
}

void Serialization::serialize_to(rapidjson::Value &val, const PathingTestResult &result,
    rapidjson::MemoryPoolAllocator<> &allocator)
{
    const auto numFailed = result.failed.size();
    const auto numVisual = result.visual.size();
    const auto numPassed = result.passed.size();

    val.SetObject();

    rj::Value name;
    name.SetString(result.name, allocator);

    std::stringstream stream;
    stream << "Passed: " << std::to_string(numPassed) << ", Failed: " << std::to_string(numFailed) << ", Visual Tests: " << std::to_string(numVisual);
    rj::Value status;
    status.SetString(stream.str(), allocator);

    rj::Value settings;
    serialize_to(settings, result.settings, allocator);

    val.AddMember("Name", name, allocator);
    val.AddMember("Status", status, allocator);
    val.AddMember("Settings", settings, allocator);

    if (numFailed > 0)
    {
        rj::Value failed;
        failed.SetArray();
        failed.Reserve(static_cast<rj::SizeType>(numFailed), allocator);

        for (const auto & test : result.failed)
        {
            rj::Value temp;
            serialize_to(temp, test, allocator);
            failed.PushBack(temp, allocator);
        }

        val.AddMember("Failed", failed, allocator);
    }
    
    if (numVisual > 0)
    {
        rj::Value visual;
        visual.SetArray();
        visual.Reserve(static_cast<rj::SizeType>(numVisual), allocator);

        for (const auto & test : result.visual)
        {
            rj::Value temp;
            serialize_to(temp, test, allocator);
            visual.PushBack(temp, allocator);
        }

        val.AddMember("Visual Tests", visual, allocator);
    }

    if (numPassed > 0)
    {
        rj::Value passed;
        passed.SetArray();
        passed.Reserve(static_cast<rj::SizeType>(numPassed), allocator);

        for (const auto & test : result.passed)
        {
            rj::Value temp;
            serialize_to(temp, test, allocator);
            passed.PushBack(temp, allocator);
        }

        val.AddMember("Passed", passed, allocator);
    }
}

void Serialization::serialize_to(rapidjson::Value &val, const GridPos &pos, rapidjson::MemoryPoolAllocator<> &allocator)
{
    val.SetObject();

    rj::Value row;
    row.SetInt(pos.row);

    rj::Value col;
    col.SetInt(pos.col);

    val.AddMember("Row", row, allocator);
    val.AddMember("Col", col, allocator);
}

bool Serialization::deserialize_from(rapidjson::Value &val, GridPos &pos, const std::wstring &id)
{
    const wchar_t *prefix = L"Grid Position ";

    if (val.IsObject() == false)
    {
        std::wcout << prefix << id << L" is not an object" << std::endl;
        return false;
    }

    bool valid = true;

    auto rowIter = val.FindMember("Row");
    auto colIter = val.FindMember("Col");

    if (extract_int(rowIter, val.MemberEnd(), pos.row, prefix, L"Row", id) == false)
    {
        valid = false;
    }

    if (extract_int(colIter, val.MemberEnd(), pos.col, prefix, L"Col", id) == false)
    {
        valid = false;
    }

    return valid;
}

void Serialization::serialize_to(rapidjson::Value &val, const PathRequest::Settings &settings, rapidjson::MemoryPoolAllocator<> &allocator)
{
    val.SetObject();

    rj::Value method;
    method.SetString(get_method_text(settings.method), allocator);

    rj::Value heuristic;
    heuristic.SetString(get_heuristic_text(settings.heuristic), allocator);

    rj::Value weight;
    weight.SetFloat(settings.weight);

    rj::Value smoothing;
    smoothing.SetBool(settings.smoothing);

    rj::Value rubberBanding;
    rubberBanding.SetBool(settings.rubberBanding);

    val.AddMember("Method", method, allocator);
    val.AddMember("Heuristic", heuristic, allocator);
    val.AddMember("Weight", weight, allocator);
    val.AddMember("Smoothing", smoothing, allocator);
    val.AddMember("Rubberbanding", rubberBanding, allocator);
}

bool Serialization::deserialize_from(rapidjson::Value &val, PathRequest::Settings &settings, const std::wstring &id)
{
    const wchar_t *prefix = L"Pathing Settings ";

    if (val.IsObject() == false)
    {
        std::wcout << prefix << id << L" is not an object" << std::endl;
        return false;
    }

    bool valid = true;

    auto methodIter = val.FindMember("Method");
    auto heuristicIter = val.FindMember("Heuristic");
    auto weightIter = val.FindMember("Weight");
    auto smoothingIter = val.FindMember("Smoothing");
    auto rubberIter = val.FindMember("Rubberbanding");
    auto end = val.MemberEnd();

    std::string temp;

    if (extract_string(methodIter, end, temp, prefix, L"Method", id) == true)
    {
        settings.method = string_to_method(temp);
    }
    else
    {
        valid = false;
    }

    if (extract_string(heuristicIter, end, temp, prefix, L"Heuristic", id) == true)
    {
        settings.heuristic = string_to_heuristic(temp);
    }
    else
    {
        valid = false;
    }

    if (extract_float(weightIter, end, settings.weight, prefix, L"Weight", id) == false)
    {
        valid = false;
    }

    if (extract_bool(smoothingIter, end, settings.smoothing, prefix, L"Smoothing", id) == false)
    {
        valid = false;
    }

    if (extract_bool(rubberIter, end, settings.rubberBanding, prefix, L"Rubberbanding", id) == false)
    {
        valid = false;
    }

    // default for all tests
    settings.singleStep = false;
    settings.debugColoring = true;

    return valid;
}

bool Serialization::open_file(FILE ** file, const std::filesystem::path &filepath, bool readOnly)
{
    const char *mode = (readOnly == true) ? "r" : "w";

    const auto result = fopen_s(file, filepath.u8string().c_str(), mode);

    if (result != 0)
    {
        std::wcout << L"Error opening file " << filepath << ": " << result << std::endl;
        return false;
    }

    return true;
}

bool Serialization::read_file(rj::Document &doc, const std::filesystem::path &filepath)
{
    FILE *file;

    if (open_file(&file, filepath, true) == false)
    {
        return false;
    }

    rj::FileReadStream stream(file, buffer, bufferSize);

    doc.ParseStream(stream);

    fclose(file);

    if (doc.HasParseError() == true)
    {
        std::wcout << L"Error parsing document " << filepath << ": " <<
            rj::GetParseError_En(doc.GetParseError()) << std::endl;
        return false;
    }

    return true;
}

bool Serialization::write_file(rj::Document &doc, const std::filesystem::path &filepath)
{
    FILE *file;

    if (open_file(&file, filepath, false) == false)
    {
        return false;
    }

    rj::FileWriteStream stream(file, buffer, bufferSize);
    rj::PrettyWriter<rj::FileWriteStream> writer(stream);

    const auto result = doc.Accept(writer);

    fclose(file);

    return result;
}

bool Serialization::extract_int(rapidjson::Value::MemberIterator iter,
    rapidjson::Value::MemberIterator end, int &data, const wchar_t *prefix,
    const wchar_t *target, const std::wstring &id)
{
    if (iter == end)
    {
        std::wcout << prefix << id << L" does not have " << target << L" member" << std::endl;
        return false;
    }
    else if (iter->value.IsInt() == false)
    {
        std::wcout << prefix << id << L" " << target << L" member is not an integer" << std::endl;
        return false;
    }
    
    data = iter->value.GetInt();
    return true;
}

bool Serialization::extract_bool(rapidjson::Value::MemberIterator iter,
    rapidjson::Value::MemberIterator end, bool &data, const wchar_t *prefix,
    const wchar_t *target, const std::wstring &id)
{
    if (iter == end)
    {
        std::wcout << prefix << id << L" does not have " << target << L" member" << std::endl;
        return false;
    }
    else if (iter->value.IsBool() == false)
    {
        std::wcout << prefix << id << L" " << target << L" member is not a boolean" << std::endl;
        return false;
    }

    data = iter->value.GetBool();
    return true;
}

bool Serialization::extract_string(rapidjson::Value::MemberIterator iter,
    rapidjson::Value::MemberIterator end, std::string &data, const wchar_t *prefix,
    const wchar_t *target, const std::wstring &id)
{
    if (iter == end)
    {
        std::wcout << prefix << id << L" does not have " << target << L" member" << std::endl;
        return false;
    }
    else if (iter->value.IsString() == false)
    {
        std::wcout << prefix << id << L" " << target << L" member is not a string" << std::endl;
        return false;
    }

    data = iter->value.GetString();
    return true;
}

bool Serialization::extract_float(rapidjson::Value::MemberIterator iter,
    rapidjson::Value::MemberIterator end, float &data, const wchar_t *prefix,
    const wchar_t *target, const std::wstring &id)
{
    if (iter == end)
    {
        std::wcout << prefix << id << L" does not have " << target << L" member" << std::endl;
        return false;
    }
    else if (iter->value.IsFloat() == false)
    {
        std::wcout << prefix << id << L" " << target << L" member is not a float" << std::endl;
        return false;
    }

    data = iter->value.GetFloat();
    return true;
}


