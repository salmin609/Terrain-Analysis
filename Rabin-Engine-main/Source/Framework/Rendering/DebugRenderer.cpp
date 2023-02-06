/******************************************************************************/
/*!
\file		DebugRenderer.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Simple system for rendering debug lines

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "DebugRenderer.h"

bool DebugRenderer::initialize()
{
    const auto context = renderer->get_resources().get_context();
    constexpr size_t size = 4096;

    try
    {
        batch = std::make_unique<DirectX::PrimitiveBatch<DirectX::VertexPositionColor>>(context, size * 3, size);
    }
    catch (const std::exception &err)
    {
        std::cout << "Failed to create debug renderer primitive batch: " << err.what() << std::endl;
        return false;
    }

    auto device = renderer->get_resources().get_device();

    try
    {
        effect = std::make_unique<DirectX::BasicEffect>(device);
    }
    catch (const std::exception &err)
    {
        std::cout << "Failed to create debug renderer effect: " << err.what() << std::endl;
        return false;
    }
    effect->SetVertexColorEnabled(true);
    
    const void *shaderByteCode;
    size_t byteCodeLength;

    effect->GetVertexShaderBytecode(&shaderByteCode, &byteCodeLength);

    const auto result = device->CreateInputLayout(DirectX::VertexPositionColor::InputElements,
        DirectX::VertexPositionColor::InputElementCount, shaderByteCode, byteCodeLength,
        layout.ReleaseAndGetAddressOf());
    if (FAILED(result) == true)
    {
        std::cout << "Failed to create debug renderer input layout: " << result << std::endl;
        __debugbreak();
        return false;
    }

    return true;
}

void DebugRenderer::reset()
{
    batch.reset();
}

void DebugRenderer::draw_line(const Vec3 &p0, const Vec3 &p1, const Color &color)
{
    buffer.emplace_back(p0, color);
    buffer.emplace_back(p1, color);
}

void DebugRenderer::draw_arrow(const Vec3 &head, const Vec3 &tail, const Color &color)
{
    static const auto rot0 = Quat::CreateFromAxisAngle(Vec3::Up, QTR_PI * 0.5f);
    static const auto rot1 = Quat::CreateFromAxisAngle(Vec3::Up, -QTR_PI * 0.5f);
    auto delta = tail - head;
    delta.Normalize();
    delta *= (globalScalar * 0.6f);
    
    const auto arm0 = Vec3::Transform(delta, rot0);
    const auto arm1 = Vec3::Transform(delta, rot1);

    buffer.emplace_back(head, color);
    buffer.emplace_back(tail, color);
    buffer.emplace_back(head, color);
    buffer.emplace_back(head + arm0, color);
    buffer.emplace_back(head, color);
    buffer.emplace_back(head + arm1, color);
}

void DebugRenderer::draw()
{
    auto context = renderer->get_resources().get_context();
    auto states = renderer->get_resources().get_states();

    effect->SetView(renderer->get_view_matrix());
    effect->SetProjection(renderer->get_projection_matrix());

    context->OMSetBlendState(states->Opaque(), nullptr, static_cast<UINT32>(-1));
    context->OMSetDepthStencilState(states->DepthNone(), 0);
    context->RSSetState(states->CullNone());

    effect->Apply(context);
    
    context->IASetInputLayout(layout.Get());

    batch->Begin();

    project->draw_debug();

    batch->Draw(D3D11_PRIMITIVE_TOPOLOGY_LINELIST, buffer.data(), buffer.size());

    batch->End();

    buffer.clear();
}
