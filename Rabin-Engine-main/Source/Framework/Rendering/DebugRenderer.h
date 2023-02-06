/******************************************************************************/
/*!
\file		DebugRenderer.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Simple system for rendering debug lines

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once

// thin wrapper around a primitive batch
class DebugRenderer
{
public:
    bool initialize();
    void reset();

    void draw_line(const Vec3 &p0, const Vec3 &p1, const Color &color);
    void draw_arrow(const Vec3 &head, const Vec3 &tail, const Color &color);

    void draw();

private:
    std::vector<DirectX::VertexPositionColor> buffer;
    std::unique_ptr<DirectX::PrimitiveBatch<DirectX::VertexPositionColor>> batch;
    std::unique_ptr<DirectX::BasicEffect> effect;
    Microsoft::WRL::ComPtr<ID3D11InputLayout> layout;
};