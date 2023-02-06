/******************************************************************************/
/*!
\file		SimpleRenderer.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Management of aggregate rendering systems

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include "DeviceResources.h"
#include "MeshRenderer.h"
#include "TextRenderer.h"
#include "UISpriteRenderer.h"
#include "DebugRenderer.h"

class SimpleRenderer
{
public:
    SimpleRenderer();

    bool initialize(HINSTANCE hInstance, int nCmdShow);
    void shutdown();

    void draw();

    const Mat4 &get_default_view_matrix() const;

    Vec3 get_clear_color() const;
    void set_clear_color(const Vec3 &color);

    void on_window_size_change(int width, int height);
    void on_device_lost();

    const DeviceResources &get_resources() const;
    const Mat4 &get_vp_matrix() const;
    const Mat4 &get_projection_matrix() const;
    const Mat4 &get_view_matrix() const;

    std::pair<Vec3, bool> screen_to_world(int posX, int posY, const DirectX::SimpleMath::Plane &plane);
    WindowPos screen_to_window(const POINT &point);

    MeshRenderer &get_grid_renderer();
    TextRenderer &get_text_renderer();
    UISpriteRenderer &get_ui_sprite_renderer();
    DebugRenderer &get_debug_renderer();

    bool output_screenshot(const std::wstring &filename);
private:
    DeviceResources resources;
    MeshRenderer gridRenderer;
    TextRenderer textRenderer;
    UISpriteRenderer uiRenderer;
    DebugRenderer debugRenderer;

    Mat4 vpMatrix;
    Mat4 projMatrix;
    Mat4 viewMatrix;
    Vec4 clearColor;

    bool register_window_class(HINSTANCE hInstance);
};