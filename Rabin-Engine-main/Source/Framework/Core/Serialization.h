/******************************************************************************/
/*!
\file		Serialization.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Serialization declarations

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include  <filesystem>
#include <rapidjson/document.h>
#include "Terrain/Terrain.h"
#include "Misc/PathfindingDetails.hpp"

class PathingTestCase;
class PathingTestData;
class PathingTestResult;
class BehaviorTreePrototype;

class Serialization
{
public:
    static bool initialize();

    static void generate_time_stamp(std::wstringstream &filename);
    static void generate_time_stamp(std::stringstream &filename);

    template <typename T>
    static bool serialize(const T &var, const std::filesystem::path &filepath);

    template <typename T>
    static bool serialize(const std::vector<T> &vec, const std::filesystem::path &filepath);

    template <typename T>
    static bool deserialize(T &var, const std::filesystem::path &filepath);

    template <typename T>
    static bool deserialize(std::vector<T> &var, const std::filesystem::path &filepath);

    static std::filesystem::path basePath;
    static std::filesystem::path mapsPath;
    static std::filesystem::path testsPath;
    static std::filesystem::path assetsPath;
    static std::filesystem::path outputPath;
    static std::filesystem::path behaviorTreePath;

    static bool open_file(FILE **file, const std::filesystem::path &filepath, bool readOnly);
private:
    template <typename T>
    static void serialize_to(rapidjson::Value &val, const T &obj, rapidjson::MemoryPoolAllocator<> &allocator);
    template <typename T>
    static bool deserialize_from(rapidjson::Value &val, T &obj, const std::wstring &id);

    template <rapidjson::SizeType I, typename... Ts>
    static void serialize_tuple_elem_to(rapidjson::Value &val, const std::tuple<Ts...> &obj, rapidjson::MemoryPoolAllocator<> &allocator);
    template <rapidjson::SizeType I, typename... Ts>
    static bool deserialize_tuple_elem_from(rapidjson::Value &val, std::tuple<Ts...> &obj, const std::wstring &id);

    template <rapidjson::SizeType... Is, typename... Ts>
    static void serialize_tuple_to(rapidjson::Value &val, std::index_sequence<Is...>, const std::tuple<Ts...> &obj, rapidjson::MemoryPoolAllocator<> &allocator);
    template <rapidjson::SizeType... Is, typename... Ts>
    static bool deserialize_tuple_from(rapidjson::Value &val, std::index_sequence<Is...>, std::tuple<Ts...> &obj, const std::wstring &id);

    template <typename... Ts>
    static void serialize_to(rapidjson::Value &val, const std::tuple<Ts...> &obj, rapidjson::MemoryPoolAllocator<> &allocator);
    template <typename... Ts>
    static bool deserialize_from(rapidjson::Value &val, std::tuple<Ts...> &obj, const std::wstring &id);

    static void serialize_to(rapidjson::Value &val, const Terrain::MapData &map, 
        rapidjson::MemoryPoolAllocator<> &allocator);
    static bool deserialize_from(rapidjson::Value &val, Terrain::MapData &map,
        const std::wstring &id);

    static void serialize_to(rapidjson::Value &val, const PathingTestCase &test,
        rapidjson::MemoryPoolAllocator<> &allocator);
    static bool deserialize_from(rapidjson::Value &val, PathingTestCase &test,
        const std::wstring &id);

    static void serialize_to(rapidjson::Value &val, const PathingTestData &data,
        rapidjson::MemoryPoolAllocator<> &allocator);
    static bool deserialize_from(rapidjson::Value &val, PathingTestData &data,
        const std::wstring &id);

    static void serialize_to(rapidjson::Value &val, const PathingTestResult &result,
        rapidjson::MemoryPoolAllocator<> &allocator);

    static void serialize_to(rapidjson::Value &val, const GridPos &pos,
        rapidjson::MemoryPoolAllocator<> &allocator);
    static bool deserialize_from(rapidjson::Value &val, GridPos &pos, const std::wstring &id);

    static void serialize_to(rapidjson::Value &val, const PathRequest::Settings &settings,
        rapidjson::MemoryPoolAllocator<> &allocator);
    static bool deserialize_from(rapidjson::Value &val, PathRequest::Settings &settings,
        const std::wstring &id);

    
    static bool read_file(rapidjson::Document &doc, const std::filesystem::path &filepath);
    static bool write_file(rapidjson::Document &doc, const std::filesystem::path &filepath);

    static bool extract_int(rapidjson::Value::MemberIterator iter, rapidjson::Value::MemberIterator end,
        int &data, const wchar_t *prefix, const wchar_t *target, const std::wstring &id);

    static bool extract_bool(rapidjson::Value::MemberIterator iter, rapidjson::Value::MemberIterator end,
        bool &data, const wchar_t *prefix, const wchar_t *target, const std::wstring &id);

    static bool extract_string(rapidjson::Value::MemberIterator iter, rapidjson::Value::MemberIterator end,
        std::string &data, const wchar_t *prefix, const wchar_t *target, const std::wstring &id);

    static bool extract_float(rapidjson::Value::MemberIterator iter, rapidjson::Value::MemberIterator end,
        float &data, const wchar_t *prefix, const wchar_t *target, const std::wstring &id);
};

template<typename T>
inline bool Serialization::serialize(const T &var, const std::filesystem::path &filepath)
{
    rapidjson::Document doc;

    serialize_to(doc, var, doc.GetAllocator());

    return write_file(doc, filepath);
}

template<typename T>
inline bool Serialization::serialize(const std::vector<T> &vec,
    const std::filesystem::path &filepath)
{
    rapidjson::Document doc;
    doc.SetArray();
    doc.Reserve(static_cast<rapidjson::SizeType>(vec.size()), doc.GetAllocator());

    for (const auto & obj : vec)
    {
        rapidjson::Value temp;
        serialize_to(temp, obj, doc.GetAllocator());

        doc.PushBack(temp, doc.GetAllocator());
    }

    return write_file(doc, filepath);
}

template<typename T>
inline bool Serialization::deserialize(T &var, const std::filesystem::path &filepath)
{
    rapidjson::Document doc;

    return read_file(doc, filepath) && deserialize_from(doc, var, filepath.wstring());
}

template<typename T>
inline bool Serialization::deserialize(std::vector<T> &var, const std::filesystem::path &filepath)
{
    rapidjson::Document doc;

    if (!read_file(doc, filepath))
    {
        return false;
    }

    const auto &array = doc.GetArray();
    auto size = array.Size();

    std::vector<T> temp;
    temp.reserve(size);
    for (decltype(size) i = 0; i < size; ++i)
    {
        T elem;
        if (!deserialize_from(array[i], elem, filepath.wstring() + L"[" + std::to_wstring(i) + L"]"))
        {
            return false;
        }
        temp.emplace_back(std::move(elem));
    }
    std::swap(var, temp);

    return true;
}

template<typename T>
inline void Serialization::serialize_to(rapidjson::Value &val, const T &obj, rapidjson::MemoryPoolAllocator<> &allocator)
{
    static_assert(std::is_trivial<T>::value, "Non-trivial type can't be read/written as string");

    val.SetString(reinterpret_cast<const char *>(&obj), sizeof(T), allocator);
}

template<typename... Ts>
inline void Serialization::serialize_to(rapidjson::Value &val, const std::tuple<Ts...> &obj, rapidjson::MemoryPoolAllocator<> &allocator)
{
    serialize_tuple_to(val, std::index_sequence_for<Ts...>{}, obj, allocator);
}

template<typename T>
inline bool Serialization::deserialize_from(rapidjson::Value &val, T &obj, const std::wstring &id)
{
    static_assert(std::is_trivial<T>::value, "Non-trivial type can't be read/written as string");

    if (val.IsString() == false)
    {
        std::wcout << L"Generic serialized object " << id << L" is not a string" << std::endl;
        return false;
    }

    const std::string data = val.GetString();

    memcpy(&obj, data.c_str(), sizeof(T));

    return true;
}

template<typename... Ts>
inline bool Serialization::deserialize_from(rapidjson::Value &val, std::tuple<Ts...> &obj, const std::wstring &id)
{
    return deserialize_tuple_from(val, std::index_sequence_for<Ts...>{}, obj, id);
}

template<rapidjson::SizeType I, typename... Ts>
inline void Serialization::serialize_tuple_elem_to(rapidjson::Value &val, const std::tuple<Ts...> &obj, rapidjson::MemoryPoolAllocator<> &allocator)
{
    rapidjson::Value value;
    serialize_to(value, std::get<I>(obj), allocator);
    val.PushBack(value, allocator);
}

template<rapidjson::SizeType I, typename... Ts>
inline bool Serialization::deserialize_tuple_elem_from(rapidjson::Value &val, std::tuple<Ts...> &obj, const std::wstring &id)
{
    return deserialize_from(val[I], std::get<I>(obj), id);
}

template<rapidjson::SizeType... Is, typename... Ts>
inline void Serialization::serialize_tuple_to(rapidjson::Value &val, std::index_sequence<Is...>, const std::tuple<Ts...> &obj, rapidjson::MemoryPoolAllocator<> &allocator)
{
    val.SetArray();
    (..., serialize_tuple_elem_to<Is>(val, obj, allocator));
}

template<rapidjson::SizeType... Is, typename... Ts>
inline bool Serialization::deserialize_tuple_from(rapidjson::Value &val, std::index_sequence<Is...>, std::tuple<Ts...> &obj, const std::wstring &id)
{
    return (... && deserialize_tuple_elem_from<Is>(val, obj, id));
}
