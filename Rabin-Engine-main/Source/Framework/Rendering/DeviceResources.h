/******************************************************************************/
/*!
\file		DeviceResources.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	DirectX 11 device management

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include "Misc/NiceTypes.h"

class DeviceResources
{
public:
    static const unsigned int fastSemantics = 0x1;
    static const unsigned int enable4KUHD = 0x2;
    static const int defaultWindowWidth = 1024;
    static const int defaultWindowHeight = 768;

    DeviceResources(DXGI_FORMAT backBufferFormat = DXGI_FORMAT_B8G8R8A8_UNORM,
        DXGI_FORMAT depthBufferFormat = DXGI_FORMAT_D24_UNORM_S8_UINT,
        UINT backBufferCount = 2);

    bool create_window(HINSTANCE hInstance, int nCmdShow,
        int width = defaultWindowWidth, int height = defaultWindowHeight);
    bool create_device();
    bool create_resources();
    

    const WindowSize &get_output_size() const;
    ID3D11Device1 *get_device() const;
    ID3D11DeviceContext1 *get_context() const;
    IDXGISwapChain1 *get_swap_chain() const;
    D3D_FEATURE_LEVEL get_feature_level() const;
    ID3D11Texture2D *get_render_target() const;
    ID3D11Texture2D *get_depth_stencil() const;
    ID3D11RenderTargetView *get_render_target_view() const;
    ID3D11DepthStencilView *get_depth_stencil_view() const;
    DXGI_FORMAT get_back_buffer_format() const;
    DXGI_FORMAT get_depth_buffer_format() const;
    UINT get_back_buffer_count() const;
    DirectX::CommonStates *get_states() const;
    DirectX::EffectFactory *get_effect_factory() const;
    HWND get_window() const;
    const D3D11_VIEWPORT &get_viewport() const;
    ID3D11BlendState *get_blend_state() const;

    void on_window_size_change(int width, int height);
    void on_device_lost();

    void prepare(const Vec4 &clearColor);
    void present();
private:
    // device objects
    Microsoft::WRL::ComPtr<ID3D11Device1> d3dDevice;
    Microsoft::WRL::ComPtr<ID3D11DeviceContext1> d3dContext;
    Microsoft::WRL::ComPtr<IDXGISwapChain1> swapChain;

    // rendering objects
    Microsoft::WRL::ComPtr<ID3D11Texture2D> renderTargetTexture;
    Microsoft::WRL::ComPtr<ID3D11Texture2D> depthStencilTexture;
    Microsoft::WRL::ComPtr<ID3D11RenderTargetView> renderTargetView;
    Microsoft::WRL::ComPtr<ID3D11DepthStencilView> depthStencilView;
    Microsoft::WRL::ComPtr<ID3D11BlendState> blendState;
    std::unique_ptr<DirectX::EffectFactory> fxFactory;
    std::unique_ptr<DirectX::CommonStates> states;
    D3D11_VIEWPORT viewport;

    // properties
    DXGI_FORMAT backBufferFormat;
    DXGI_FORMAT depthBufferFormat;
    UINT backBufferCount;

    // device properties
    HWND window;
    D3D_FEATURE_LEVEL featureLevel;
    WindowSize windowSize;
};