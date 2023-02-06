/******************************************************************************/
/*!
\file		Terrain.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Map and layer management

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include "MapLayer.h"
#include "../Misc/NiceTypes.h"
#include  <filesystem>


class Agent;
class DeviceResources;
class ProjectOne;
class ProjectTwo;
class ProjectThree;
class EnemyAgent;

class Terrain
{
    friend class ProjectOne;
    friend class ProjectTwo;
    friend class ProjectThree;
    friend class EnemyAgent;
public:
    static const size_t numLayers = 8;

    Terrain();

    int get_map_height() const;
    int get_map_width() const;

    const Vec3 &get_world_position(int row, int col) const;
    const Vec3 &get_world_position(const GridPos &gridPos);
    GridPos get_grid_position(const Vec3 &worldPos) const;

    bool is_wall(int row, int col) const;
    bool is_wall(const GridPos &gridPos) const;

    bool is_valid_grid_position(int row, int col) const;
    bool is_valid_grid_position(const GridPos &gridPos) const;

    void set_color(int row, int col, const Color &color);
    void set_color(const GridPos &gridPos, const Color &color);

    const DirectX::SimpleMath::Plane &get_terrain_plane() const;

    static const size_t maxMapHeight = 40;
    static const size_t maxMapWidth = 40;

    static const float mapSizeInWorld;

    static Color baseColor;
    static Color wallColor;
    static Color opennessColor;
    static Color totalVisibilityColor;
    static Color cellVisibilityColor;
    static Color positiveOccupancyColor;
    static Color negativeOccupancyColor;
    static Color agentVisionColor;
    static Color fogColor;
    static Color seekDetectionColor;
    static Color seekSearchColor;
    static float maxLayerAlpha;

    void draw();
    void draw_debug();

    void goto_next_map();
    bool goto_map(unsigned mapNum);
    size_t num_maps() const;

    struct MapData
    {
        MapData() = default;
        MapData(int height, int width);
        int height;
        int width;
        std::vector<std::vector<bool>> data;
    };
private:
    MapLayer<bool> wallLayer;
    MapLayer<Color> pathLayer;
    MapLayer<float> opennessLayer;
    MapLayer<float> totalVisibilityLayer;
    MapLayer<float> cellVisibilityLayer;
    MapLayer<float> occupancyLayer;
    MapLayer<float> agentVisionLayer;
    MapLayer<float> fogLayer;
    MapLayer<float> seekLayer;

    std::vector<MapData> mapData;
    std::vector<std::vector<Vec3>> positions;

    unsigned currentMap;

    bool initialize();
    void shutdown();

    void generate_positions();

    void load_map_data(const std::filesystem::path &file);
    void load_map(unsigned mapIndex);

    void configure_float_map_layer(MapLayer<float> &layer, int height, int width, const Color &color0, const Color &color1);
    void refresh_static_analysis_layers();
    void reset_path_layer();
};