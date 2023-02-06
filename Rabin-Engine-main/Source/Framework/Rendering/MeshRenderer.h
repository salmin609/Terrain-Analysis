/******************************************************************************/
/*!
\file		MeshRenderer.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Simple rendering system for grid cells

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once

class DeviceResources;

class MeshRenderer
{
public:
    MeshRenderer();

    void reset();
    bool initialize();

    static const size_t maxGridInstances;

    void add_grid_instance(const Vec3 &pos, const Vec4 &color);

    void draw();
private:
    struct GridInstanceData
    {
        Vec3 position;
        Vec4 color;
    };

    Microsoft::WRL::ComPtr<ID3D11InputLayout> gridInputLayout;
    Microsoft::WRL::ComPtr<ID3D11Buffer> gridVertexConstants;
    Microsoft::WRL::ComPtr<ID3D11Buffer> gridVertexBuffer;
    Microsoft::WRL::ComPtr<ID3D11Buffer> gridIndexBuffer;
    Microsoft::WRL::ComPtr<ID3D11Buffer> gridInstanceBuffer;
    Microsoft::WRL::ComPtr<ID3D11VertexShader> gridVertexShader;
    Microsoft::WRL::ComPtr<ID3D11PixelShader> gridPixelShader;

    std::unique_ptr<GridInstanceData[]> gridInstanceData;
    uint32_t gridNumInstances;

    struct GridConstantData // needs to be 16 byte aligned
    {
        Mat4 viewMatrix;    // 64 bytes
        Mat4 projMatrix;    // 64 bytes
        float scale;        // 4 bytes
        float unused0;      // 4 bytes
        float unused1;      // 4 bytes
        float unused2;      // 4 bytes
        //      total          144 bytes
    } gridVertexConstantData;

    bool initialize_vertex_shader();
    bool initialize_pixel_shader();
    bool initialize_vertex_buffer();
    bool initialize_instance_buffer();
    bool initialize_index_buffer();
    bool initialize_constants_buffer();

    bool update_vertex_constants();
    bool push_to_buffer(ID3D11Buffer *buffer, size_t numBytes, void *data);

    void commit();
};