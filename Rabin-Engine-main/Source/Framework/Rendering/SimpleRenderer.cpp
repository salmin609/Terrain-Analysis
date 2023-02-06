/******************************************************************************/
/*!
\file		SimpleRenderer.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Management of aggregate rendering systems

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "../Agent/CameraAgent.h"
#include "Input/InputHandler.h"
#include <sstream>
#include "Core/Serialization.h"
#include <wincodec.h>
#include <Dbt.h>

namespace
{
    const auto windowClassName = L"AI_FrameworkWindowClass";
    const auto windowTitle = L"AI Framework";
    const auto assetsPath = L".\\Assets";
    const Mat4 defaultViewMatrix = Mat4::CreateLookAt(Vec3(0.0f, 1.0f, -1.0f), Vec3(0.0f, 0.0f, 0.0f), Vec3(0.0f, 1.0f, 0.0f));
}

LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);

SimpleRenderer::SimpleRenderer() : resources(),  clearColor(DirectX::Colors::CornflowerBlue)
{}

bool SimpleRenderer::initialize(HINSTANCE hInstance, int nCmdShow)
{
    if (register_window_class(hInstance) &&
        resources.create_window(hInstance, nCmdShow) &&
        resources.create_device() &&
        resources.create_resources() &&
        gridRenderer.initialize() &&
        textRenderer.initialize() &&
        uiRenderer.initialize() &&
        debugRenderer.initialize() &&
        agents->acquire_rendering_resources())
    {
        return true;
    }

    return false;
}

void SimpleRenderer::shutdown()
{

}

void SimpleRenderer::draw()
{
    const auto *camera = agents->get_camera_agent();

    const auto windowSize = resources.get_output_size();
    const float aspectRatio = static_cast<float>(windowSize.width) / static_cast<float>(windowSize.height);

    projMatrix = DirectX::SimpleMath::Matrix::CreatePerspectiveFieldOfView(QTR_PI, aspectRatio, 1.0f, 200.0f);
    viewMatrix = (camera != nullptr) ? camera->get_view_matrix() : defaultViewMatrix;

    vpMatrix = viewMatrix * projMatrix;

    resources.prepare(clearColor);

    gridRenderer.draw();

    uiRenderer.draw();

    textRenderer.draw();

    //Agent::update_model(viewMatrix, projMatrix);

    debugRenderer.draw();

    resources.present();
}


const Mat4 &SimpleRenderer::get_default_view_matrix() const
{
    static const auto matrix = Mat4::CreateLookAt(Vec3(0.0f, 10.0f, 6.0f), Vec3::Zero, Vec3::Up);
    return matrix;
}

Vec3 SimpleRenderer::get_clear_color() const
{
    return Vec3(clearColor.x, clearColor.y, clearColor.z);
}

void SimpleRenderer::set_clear_color(const Vec3 &color)
{
    clearColor = Vec4(color.x, color.y, color.z, 1.0f);
}

void SimpleRenderer::on_window_size_change(int width, int height)
{
    static bool init = false;

    // ignore the first window size change event
    if (init == true)
    {
        textRenderer.reset(); // need to free the buffer first
        resources.on_window_size_change(width, height);
        textRenderer.initialize();
        uiRenderer.on_window_size_change();
    }
    else
    {
        init = true;
    }
}

void SimpleRenderer::on_device_lost()
{
    agents->release_rendering_resources();
    debugRenderer.reset();
    gridRenderer.reset();
    textRenderer.reset();
    uiRenderer.reset();
    resources.on_device_lost();
}

const DeviceResources &SimpleRenderer::get_resources() const
{
    return resources;
}

const Mat4 &SimpleRenderer::get_vp_matrix() const
{
    return vpMatrix;
}

const Mat4 &SimpleRenderer::get_projection_matrix() const
{
    return projMatrix;
}

const Mat4 &SimpleRenderer::get_view_matrix() const
{
    return viewMatrix;
}

std::pair<Vec3, bool> SimpleRenderer::screen_to_world(int posX, int posY, const DirectX::SimpleMath::Plane &plane)
{
    const auto size = resources.get_output_size();
    const float x = (static_cast<float>(posX) / static_cast<float>(size.width)) * 2.0f - 1.0f;
    const float y = (static_cast<float>(posY) / static_cast<float>(size.height)) * -2.0f + 1.0f;
    const Vec3 nearP(x, y, 0.0f);
    const Vec3 farP(x, y, 1.0f);
    const Mat4 inverse = vpMatrix.Invert();

    const Vec3 origin = Vec3::Transform(nearP, inverse);
    const Vec3 end = Vec3::Transform(farP, inverse);
    Vec3 dir = end - origin;
    dir.Normalize();
    const DirectX::SimpleMath::Ray mouseRay(origin, dir);

    float dist;
    if (mouseRay.Intersects(plane, dist) == true)
    {
        return std::make_pair(origin + (dir * dist), true);
    }

    return std::make_pair(Vec3(), false);
}

WindowPos SimpleRenderer::screen_to_window(const POINT &point)
{
    POINT pt = point;
    ScreenToClient(resources.get_window(), &pt);
    return { pt.x, pt.y };
}

MeshRenderer &SimpleRenderer::get_grid_renderer()
{
    return gridRenderer;
}

TextRenderer &SimpleRenderer::get_text_renderer()
{
    return textRenderer;
}

UISpriteRenderer &SimpleRenderer::get_ui_sprite_renderer()
{
    return uiRenderer;
}

DebugRenderer &SimpleRenderer::get_debug_renderer()
{
    return debugRenderer;
}

bool SimpleRenderer::output_screenshot(const std::wstring &filename)
{
    std::wstringstream filenameFull;
    filenameFull << L"Output/" << filename;

    Serialization::generate_time_stamp(filenameFull);
    filenameFull << L".png";

    const auto result = DirectX::SaveWICTextureToFile(resources.get_context(),
        resources.get_render_target(), GUID_ContainerFormatPng, filenameFull.str().c_str());

    if (FAILED(result) == true)
    {
        std::wcout << L"Failed to generate screenshot " << filenameFull.str() << std::endl;
        return false;
    }

    return true;
}

bool SimpleRenderer::register_window_class(HINSTANCE hInstance)
{
    std::cout << "Registering window class..." << std::endl;

    WNDCLASSEX wcex;
    wcex.cbSize = sizeof(WNDCLASSEX);
    wcex.style = CS_HREDRAW | CS_VREDRAW;
    wcex.lpfnWndProc = WndProc;
    wcex.cbClsExtra = 0;
    wcex.cbWndExtra = 0;
    wcex.hInstance = hInstance;
    wcex.hIcon = LoadIcon(hInstance, L"IDI_ICON");
    wcex.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
    wcex.lpszMenuName = nullptr;
    wcex.lpszClassName = windowClassName;
    wcex.hIconSm = LoadIcon(wcex.hInstance, L"IDI_ICON");

    if (!RegisterClassEx(&wcex))
    {
        std::cout << "Failed to register window class" << std::endl;
        return false;
    }

    std::cout << "Window class registration complete" << std::endl;
    return true;
}

// Windows procedure
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    PAINTSTRUCT ps;
    HDC hdc;

    static bool s_in_sizemove = false;
    static bool s_in_suspend = false;
    static bool s_minimized = false;

    switch (message)
    {
    case WM_SETFOCUS:
        InputHandler::set_focus(true);
        break;

    case WM_KILLFOCUS:
        InputHandler::set_focus(false);
        break;

    case WM_KEYDOWN:
    case WM_SYSKEYDOWN:
    case WM_KEYUP:
    case WM_SYSKEYUP:
        InputHandler::process_key_message(message, wParam, lParam);
        //DirectX::Keyboard::ProcessMessage(message, wParam, lParam);
        break;

    case WM_PAINT:
        if (s_in_sizemove && engine)
        {
            engine->tick();
        }
        else
        {
            hdc = BeginPaint(hWnd, &ps);
            EndPaint(hWnd, &ps);
        }
        break;

    case WM_SIZE:
        if (wParam == SIZE_MINIMIZED)
        {
            if (!s_minimized)
            {
                s_minimized = true;
                if (!s_in_suspend && engine)
                    engine->on_suspending();
                s_in_suspend = true;
            }
        }
        else if (s_minimized)
        {
            s_minimized = false;
            if (s_in_suspend && engine)
                engine->on_resuming();
            s_in_suspend = false;
        }
        else if (!s_in_sizemove && renderer)
        {
            renderer->on_window_size_change(LOWORD(lParam), HIWORD(lParam));
        }
        break;

    case WM_ENTERSIZEMOVE:
        s_in_sizemove = true;
        break;

    case WM_EXITSIZEMOVE:
        s_in_sizemove = false;
        if (renderer)
        {
            RECT rc;
            GetClientRect(hWnd, &rc);

            renderer->on_window_size_change(rc.right - rc.left, rc.bottom - rc.top);

            if (ui)
            {
                ui->on_window_size_change();
            }
        }
        break;

    case WM_GETMINMAXINFO:
    {
        auto info = reinterpret_cast<MINMAXINFO*>(lParam);
        info->ptMinTrackSize.x = 320;
        info->ptMinTrackSize.y = 200;
    }
    break;

    case WM_ACTIVATE:
    case WM_ACTIVATEAPP:
        if (engine)
        {
            // DirectX::Keyboard::ProcessMessage(message, wParam, lParam);
            // DirectX::Mouse::ProcessMessage(message, wParam, lParam);

            if (wParam)
            {
                engine->on_activated();
            }
            else
            {
                engine->on_deactivated();
            }
        }
        break;

    case WM_POWERBROADCAST:
        switch (wParam)
        {
        case PBT_APMQUERYSUSPEND:
            if (!s_in_suspend && engine)
                engine->on_suspending();
            s_in_suspend = true;
            return TRUE;

        case PBT_APMRESUMESUSPEND:
            if (!s_minimized)
            {
                if (s_in_suspend && engine)
                    engine->on_resuming();
                s_in_suspend = false;
            }
            return TRUE;
        }
        break;
    
    /*case WM_DEVICECHANGE:
        if (wParam == DBT_DEVICEARRIVAL)
        {
            auto pDev = reinterpret_cast<PDEV_BROADCAST_HDR>(lParam);
            if (pDev != nullptr)
            {
                if (pDev->dbch_devicetype == DBT_DEVTYP_DEVICEINTERFACE)
                {
                    auto pInter = reinterpret_cast<const PDEV_BROADCAST_DEVICEINTERFACE>(pDev);
                    if (pInter->dbcc_classguid == KSCATEGORY_AUDIO)
                    {
                        if (audio)
                        {
                            audio->new_device_connected();
                        }
                    }
                }
            }
        }*/

    case WM_DESTROY:
        PostQuitMessage(0);
        break;

    case WM_MENUCHAR:
        // A menu is active and the user presses a key that does not correspond
        // to any mnemonic or accelerator key. Ignore so we don't produce an error beep.
        return MAKELRESULT(0, MNC_CLOSE);
    }

    return DefWindowProc(hWnd, message, wParam, lParam);
}

