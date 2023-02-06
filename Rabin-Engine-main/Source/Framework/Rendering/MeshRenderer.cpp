/******************************************************************************/
/*!
\file		MeshRenderer.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Simple rendering system for grid cells

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "ReadShader.h"

const size_t MeshRenderer::maxGridInstances = Terrain::maxMapHeight * Terrain::maxMapWidth * Terrain::numLayers;;

namespace
{
    struct Vertex
    {
        Vec3 pos;
        Vec3 norm;
    };

    const size_t numIndices = 6;
}

MeshRenderer::MeshRenderer() : gridNumInstances(0)
{
    gridInstanceData.reset(new GridInstanceData[maxGridInstances]);
    gridVertexConstantData.unused0 = 0.0f;
    gridVertexConstantData.unused1 = 0.0f;
    gridVertexConstantData.unused2 = 0.0f;
}

void MeshRenderer::commit()
{
    if (gridNumInstances > 0 && update_vertex_constants() == true) // also includes context != nullptr check
    {
        if (push_to_buffer(gridInstanceBuffer.Get(), sizeof(GridInstanceData) * gridNumInstances, gridInstanceData.get()) == true)
        {
            try
            {
                auto context = renderer->get_resources().get_context();
                auto blend = renderer->get_resources().get_blend_state();

                UINT strides[] = { sizeof(Vertex), sizeof(GridInstanceData) };
                UINT offsets[] = { 0, 0 };
                ID3D11Buffer *buffers[] = { gridVertexBuffer.Get(), gridInstanceBuffer.Get() };

                context->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

                context->OMSetBlendState(blend, NULL, D3D11_COLOR_WRITE_ENABLE_ALL);

                context->IASetInputLayout(gridInputLayout.Get());

                context->IASetVertexBuffers(0, _countof(strides), buffers, strides, offsets);

                context->IASetIndexBuffer(gridIndexBuffer.Get(), DXGI_FORMAT_R16_UINT, 0);

                context->VSSetConstantBuffers(0, 1, gridVertexConstants.GetAddressOf());

                context->VSSetShader(gridVertexShader.Get(), nullptr, 0);
                context->PSSetShader(gridPixelShader.Get(), nullptr, 0);

                context->DrawIndexedInstanced(numIndices, gridNumInstances, 0, 0, 0);
            }
            catch (const std::exception &err)
            {
                std::cout << err.what() << std::endl;
            }
        }
    }

    gridNumInstances = 0;
}

void MeshRenderer::reset()
{
    gridVertexConstants.Reset();
    gridIndexBuffer.Reset();
    gridInstanceBuffer.Reset();
    gridVertexBuffer.Reset();
    gridPixelShader.Reset();
    gridInputLayout.Reset();
    gridVertexShader.Reset();
}

bool MeshRenderer::initialize()
{
    std::cout << "Initializing Mesh Renderer..." << std::endl;

    return initialize_vertex_shader() &&
        initialize_pixel_shader() &&
        initialize_vertex_buffer() &&
        initialize_instance_buffer() &&
        initialize_index_buffer() &&
        initialize_constants_buffer();
}

void MeshRenderer::add_grid_instance(const Vec3 &pos, const Vec4 &color)
{
    gridInstanceData[gridNumInstances++] = GridInstanceData { pos, color };
}

void MeshRenderer::draw()
{
    project->draw_meshes();

    commit();
}

bool MeshRenderer::initialize_vertex_shader()
{
    //std::cout << "Initializing grid cell vertex shader..." << std::endl;

    static const D3D11_INPUT_ELEMENT_DESC inputElementDesc[] =
    {
        // SemanticName SemanticIndex   Format                          InputSlot AlignedByteOffset             InputSlotClass                    InstancedDataStepRate
        { "POSITION",   0,              DXGI_FORMAT_R32G32B32_FLOAT,    0,        0,                            D3D11_INPUT_PER_VERTEX_DATA,      0 },  // Vertex local position
        { "NORMAL",     0,              DXGI_FORMAT_R32G32B32_FLOAT,    0,        D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA,      0 },  // Vertex normal
        { "I_POSITION", 0,              DXGI_FORMAT_R32G32B32_FLOAT,    1,        0,                            D3D11_INPUT_PER_INSTANCE_DATA,    1 },  // Instance position
        { "I_COLOR",    0,              DXGI_FORMAT_R32G32B32A32_FLOAT, 1,        D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_INSTANCE_DATA,    1 },  // Instance color
    };

    // load and create shaders
    std::vector<uint8_t> shaderCode;
    try
    {
        shaderCode = read_shader(L".\\Assets\\GridCellVertexShader.cso");
    }
    catch (const std::exception &ex)
    {
        std::cout << "Unable to read grid cell vertex shader file: " << ex.what() << std::endl;
        return false;
    }

    auto device = renderer->get_resources().get_device();

    const auto createVSResult = device->CreateVertexShader(shaderCode.data(), shaderCode.size(),
        nullptr, gridVertexShader.ReleaseAndGetAddressOf());
    if (FAILED(createVSResult) == true)
    {
        std::cout << "Failed to create grid cell vertex shader from byte code: " << createVSResult << std::endl;
        return false;
    }

    const auto createVSLayout = device->CreateInputLayout(inputElementDesc, _countof(inputElementDesc),
        shaderCode.data(), shaderCode.size(), gridInputLayout.ReleaseAndGetAddressOf());
    if (FAILED(createVSLayout) == true)
    {
        std::cout << "Failed to create grid cell vertex shader layout: " << createVSLayout << std::endl;
        return false;
    }

    return true;
}

bool MeshRenderer::initialize_pixel_shader()
{
    //std::cout << "Initializing grid cell pixel shader..." << std::endl;

    std::vector<uint8_t> shaderCode;
    try
    {
        shaderCode = read_shader(L".\\Assets\\GridCellPixelShader.cso");
    }
    catch (const std::exception &ex)
    {
        std::cout << "Unable to read grid cell pixel shader file: " << ex.what() << std::endl;
        return false;
    }

    auto device = renderer->get_resources().get_device();

    const auto createPSResult = device->CreatePixelShader(shaderCode.data(), shaderCode.size(),
        nullptr, gridPixelShader.ReleaseAndGetAddressOf());
    if (FAILED(createPSResult) == true)
    {
        std::cout << "Failed to create grid cell pixel shader from byte code: " << createPSResult << std::endl;
        return false;
    }

    return true;
}

bool MeshRenderer::initialize_vertex_buffer()
{
    //std::cout << "Initializing grid cell vertex buffer..." << std::endl;

    static const Vertex vertices[] = 
    {
        { Vec3(-0.5f, 0.0f, 0.5f), Vec3(0.0f, 1.0f, 0.0f) },
        { Vec3(0.5f, 0.0f, 0.5f), Vec3(0.0f, 1.0f, 0.0f) },
        { Vec3(0.5f, 0.0f, -0.5f), Vec3(0.0f, 1.0f, 0.0f) },
        { Vec3(-0.5f, 0.0f, -0.5f), Vec3(0.0f, 1.0f, 0.0f) }
    };

    D3D11_SUBRESOURCE_DATA initialData = { vertices, 0, 0 };

    CD3D11_BUFFER_DESC bufferDesc(sizeof(vertices), D3D11_BIND_VERTEX_BUFFER, D3D11_USAGE_IMMUTABLE);
    bufferDesc.StructureByteStride = sizeof(Vertex);

    auto device = renderer->get_resources().get_device();

    const auto createResult = device->CreateBuffer(&bufferDesc, &initialData, gridVertexBuffer.ReleaseAndGetAddressOf());
    if (FAILED(createResult) == true)
    {
        std::cout << "Failed to create grid cell vertex buffer: " << createResult << std::endl;
        return false;
    }

    return true;
}

bool MeshRenderer::initialize_instance_buffer()
{
    //std::cout << "Initializing grid cell instance buffer..." << std::endl;

    CD3D11_BUFFER_DESC bufferDesc(sizeof(GridInstanceData) * maxGridInstances, D3D11_BIND_VERTEX_BUFFER,
        D3D11_USAGE_DYNAMIC, D3D11_CPU_ACCESS_WRITE);
    bufferDesc.StructureByteStride = sizeof(GridInstanceData);

    auto device = renderer->get_resources().get_device();

    const auto createResult = device->CreateBuffer(&bufferDesc, nullptr, gridInstanceBuffer.ReleaseAndGetAddressOf());
    if (FAILED(createResult) == true)
    {
        std::cout << "Failed to create grid cell instance buffer: " << createResult << std::endl;
        return false;
    }

    return true;
}

bool MeshRenderer::initialize_index_buffer()
{
    //std::cout << "Initializing grid cell index buffer..." << std::endl;
    static const uint16_t indices[] = 
    {
        0, 1, 2,
        0, 2, 3
    };

    D3D11_SUBRESOURCE_DATA initialData = { indices, 0, 0 };

    CD3D11_BUFFER_DESC bufferDesc(sizeof(indices), D3D11_BIND_INDEX_BUFFER, D3D11_USAGE_IMMUTABLE);
    bufferDesc.StructureByteStride = sizeof(uint16_t);

    auto device = renderer->get_resources().get_device();

    const auto createResult = device->CreateBuffer(&bufferDesc, &initialData, gridIndexBuffer.ReleaseAndGetAddressOf());
    if (FAILED(createResult) == true)
    {
        std::cout << "Failed to create grid cell index buffer: " << createResult << std::endl;
        return false;
    }

    return true;
}

bool MeshRenderer::initialize_constants_buffer()
{
    //std::cout << "Initializing grid cell constants buffer..." << std::endl;

    CD3D11_BUFFER_DESC bufferDesc(sizeof(GridConstantData), D3D11_BIND_CONSTANT_BUFFER,
        D3D11_USAGE_DYNAMIC, D3D11_CPU_ACCESS_WRITE);

    auto device = renderer->get_resources().get_device();

    const auto createResult = device->CreateBuffer(&bufferDesc, nullptr, gridVertexConstants.ReleaseAndGetAddressOf());
    if (FAILED(createResult) == true)
    {
        std::cout << "Failed to create grid cell constants buffer: " << createResult << std::endl;
        return false;
    }

    return true;
}

bool MeshRenderer::update_vertex_constants()
{
    auto context = renderer->get_resources().get_context();

    if (context != nullptr)
    {
        gridVertexConstantData.viewMatrix = renderer->get_view_matrix();
        gridVertexConstantData.projMatrix = renderer->get_projection_matrix();
        gridVertexConstantData.scale = globalScalar;

        return push_to_buffer(gridVertexConstants.Get(), sizeof(GridConstantData), &gridVertexConstantData);
    }

    return false;
}

bool MeshRenderer::push_to_buffer(ID3D11Buffer *buffer, size_t numBytes, void *data)
{
    auto context = renderer->get_resources().get_context();

    if (context != nullptr)
    {
        D3D11_MAPPED_SUBRESOURCE mapped;
        const auto mapResult = context->Map(buffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped);

        if (FAILED(mapResult) == true)
        {
            std::cout << "Failed to map to grid cell buffer: " << mapResult << std::endl;
            return false;
        }

        memcpy(mapped.pData, data, numBytes);
        context->Unmap(buffer, 0);
        return true;
    }

    return false;
}
