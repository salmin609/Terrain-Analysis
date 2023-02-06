/******************************************************************************/
/*!
\file		DeviceResources.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	DirectX 11 device management

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "Agent/Agent.h"

namespace
{
    const auto windowClassName = L"AI_FrameworkWindowClass";
    const auto windowTitle = L"AI Framework";
    const auto assetsPath = L".\\Assets";
}

DeviceResources::DeviceResources(DXGI_FORMAT backBufferFormat, DXGI_FORMAT depthBufferFormat,
    UINT backBufferCount) : backBufferFormat(backBufferFormat), 
    depthBufferFormat(depthBufferFormat), backBufferCount(backBufferCount)
{}

bool DeviceResources::create_device()
{
    std::cout << "Creating rendering device..." << std::endl;
    UINT creationFlags = D3D11_CREATE_DEVICE_BGRA_SUPPORT | D3D11_CREATE_DEVICE_SINGLETHREADED;

    #ifdef DEVICE_DEBUG
        creationFlags |= D3D11_CREATE_DEVICE_DEBUG;
    #endif

    static const D3D_FEATURE_LEVEL featureLevels[] =
    {
        D3D_FEATURE_LEVEL_11_1,
        D3D_FEATURE_LEVEL_11_0
    };

    // create the DX11 API device object, and get a corresponding context
    Microsoft::WRL::ComPtr<ID3D11Device> device;
    Microsoft::WRL::ComPtr<ID3D11DeviceContext> context;

    const auto createDeviceResult = D3D11CreateDevice(
        nullptr, // specify nullptr to use the default adapter
        D3D_DRIVER_TYPE_HARDWARE,
        nullptr,
        creationFlags,
        featureLevels,
        _countof(featureLevels),
        D3D11_SDK_VERSION,
        device.ReleaseAndGetAddressOf(),
        &featureLevel,
        context.ReleaseAndGetAddressOf()
    );

    if (FAILED(createDeviceResult) == true)
    {
        std::cout << "Failed to create rendering device" << std::endl;
        return false;
    }

    #ifdef DEVICE_DEBUG
        Microsoft::WRL::ComPtr<ID3D11Debug> d3dDebug;
        if (SUCCEEDED(device.As(&d3dDebug)) == true)
        {
            Microsoft::WRL::ComPtr<ID3D11InfoQueue> d3dInfoQueue;
            if (SUCCEEDED(d3dDebug.As(&d3dInfoQueue)) == true)
            {
                d3dInfoQueue->SetBreakOnSeverity(D3D11_MESSAGE_SEVERITY_CORRUPTION, true);
                d3dInfoQueue->SetBreakOnSeverity(D3D11_MESSAGE_SEVERITY_ERROR, true);

                D3D11_MESSAGE_ID hide[] =
                {
                    D3D11_MESSAGE_ID_SETPRIVATEDATA_CHANGINGPARAMS,
                };
                D3D11_INFO_QUEUE_FILTER filter = {};
                filter.DenyList.NumIDs = _countof(hide);
                filter.DenyList.pIDList = hide;
                d3dInfoQueue->AddStorageFilterEntries(&filter);
            }
        }
    #endif

    const auto deviceSetResult = device.As(&d3dDevice);
    if (FAILED(deviceSetResult) == true)
    {
        std::cout << "Failed to set rendering device" << std::endl;
        return false;
    }

    const auto contextSetResult = context.As(&d3dContext);
    if (FAILED(contextSetResult) == true)
    {
        std::cout << "Failed to set rendering context" << std::endl;
        return false;
    }

    states = std::make_unique<DirectX::CommonStates>(d3dDevice.Get());
    fxFactory = std::make_unique<DirectX::EffectFactory>(d3dDevice.Get());
    fxFactory->SetDirectory(assetsPath);

    return true;
}

bool DeviceResources::create_resources()
{
    std::cout << "Creating rendering resources..." << std::endl;
    // clear the previous window size specific context
    ID3D11RenderTargetView *nullViews[] = { nullptr };
    d3dContext->OMSetRenderTargets(_countof(nullViews), nullViews, nullptr);
    renderTargetView.Reset();
    depthStencilView.Reset();
    renderTargetTexture.Reset();
    depthStencilTexture.Reset();
    d3dContext->Flush();

    const UINT backBufferWidth = static_cast<UINT>(windowSize.width);
    const UINT backBufferHeight = static_cast<UINT>(windowSize.height);

    // if the swap chain already exists, resize it, otherwise create a new one
    if (swapChain)
    {
        std::cout << "Swap chain already exists, resizing..." << std::endl;
        const auto swapChainResizeResult = swapChain->ResizeBuffers(backBufferCount, backBufferWidth, backBufferHeight, backBufferFormat, 0);
        if (swapChainResizeResult == DXGI_ERROR_DEVICE_REMOVED || swapChainResizeResult == DXGI_ERROR_DEVICE_RESET)
        {
            std::cout << "Device removed or reset during swap chain resizing, rebuilding from scratch" << std::endl;
            // if the device was removed for any reason, a new device and swap chain will need to be created
            on_device_lost();

            // everything is set up now, do not continue execution of this method.  on_device_lost will reenter
            // this method and correctly set up the new device
            return true;
        }
        else if (FAILED(swapChainResizeResult) == true)
        {
            std::cout << "Failed to resize swap chain buffers: " << swapChainResizeResult << std::endl;
            return false;
        }

        std::cout << "Swap chain resizing complete" << std::endl;
    }
    else
    {
        // first, retrieve the underlying DXGI device fromt he D3D device
        Microsoft::WRL::ComPtr<IDXGIDevice1> dxgiDevice;
        const auto deviceSetResult = d3dDevice.As(&dxgiDevice);
        if (FAILED(deviceSetResult) == true)
        {
            std::cout << "Failed to set the device: " << deviceSetResult << std::endl;
            return false;
        }

        // identify the physical adapter (GPU or card) this device is running on
        Microsoft::WRL::ComPtr<IDXGIAdapter> dxgiAdapter;
        const auto adapterGetResult = dxgiDevice->GetAdapter(dxgiAdapter.GetAddressOf());
        if (FAILED(adapterGetResult) == true)
        {
            std::cout << "Failed to get the physical adapter: " << adapterGetResult << std::endl;
            return false;
        }

        // and obtain the factory object that created it
        Microsoft::WRL::ComPtr<IDXGIFactory2> dxgiFactory;
        const auto factoryGetResult = dxgiAdapter->GetParent(IID_PPV_ARGS(dxgiFactory.GetAddressOf()));
        if (FAILED(factoryGetResult) == true)
        {
            std::cout << "Failed to the adapter factory: " << factoryGetResult << std::endl;
            return false;
        }

        // create a descriptor for the swap chain
        DXGI_SWAP_CHAIN_DESC1 swapChainDesc = {};
        swapChainDesc.Width = backBufferWidth;
        swapChainDesc.Height = backBufferHeight;
        swapChainDesc.Format = backBufferFormat;
        swapChainDesc.SampleDesc.Count = 1;
        swapChainDesc.SampleDesc.Quality = 0;
        swapChainDesc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
        swapChainDesc.BufferCount = backBufferCount;

        DXGI_SWAP_CHAIN_FULLSCREEN_DESC fsSwapChainDesc = {};
        fsSwapChainDesc.Windowed = TRUE;

        // create a swap chain from the window
        const auto createSwapChainResult = dxgiFactory->CreateSwapChainForHwnd(
            d3dDevice.Get(),
            window,
            &swapChainDesc,
            &fsSwapChainDesc,
            nullptr,
            swapChain.ReleaseAndGetAddressOf()
        );
        if (FAILED(createSwapChainResult) == true)
        {
            std::cout << "Failed to create the swap chain: " << createSwapChainResult << std::endl;
            return false;
        }

        // not supporting exclusive fullscreen mode, prevent DXGI from responding to alt+enter
        const auto windowAssociationResult = dxgiFactory->MakeWindowAssociation(window, DXGI_MWA_NO_ALT_ENTER);
        if (FAILED(windowAssociationResult) == true)
        {
            std::cout << "Failed to make window association: " << windowAssociationResult << std::endl;
            return false;
        }
    }

    // obtain the backbuffer for this window which will be the final 3D rendertarget
    const auto backBufferGetResult = swapChain->GetBuffer(0, IID_PPV_ARGS(renderTargetTexture.GetAddressOf()));
    if (FAILED(backBufferGetResult) == true)
    {
        std::cout << "Failed to get the back buffer from the swap chain: " << backBufferGetResult << std::endl;
        return false;
    }

    // create a view interface on the rendertarget to use on bind
    const auto renderTargetCreateResult = d3dDevice->CreateRenderTargetView(renderTargetTexture.Get(), nullptr,
        renderTargetView.ReleaseAndGetAddressOf());
    if (FAILED(renderTargetCreateResult) == true)
    {
        std::cout << "Failed to create a render target from the device: " << renderTargetCreateResult << std::endl;
        return false;
    }

    // allocate a 2D surface as the depth/stencil buffer
    CD3D11_TEXTURE2D_DESC depthStencilDesc(depthBufferFormat, backBufferWidth, backBufferHeight, 1, 1, D3D11_BIND_DEPTH_STENCIL);

    // create a depth stencil view on this surface to use on bind
    const auto depthStencilTextureResult = d3dDevice->CreateTexture2D(&depthStencilDesc, nullptr,
        depthStencilTexture.GetAddressOf());
    if (FAILED(depthStencilTextureResult) == true)
    {
        std::cout << "Failed to create a texture for the depth/stencil buffer: " << depthStencilTextureResult << std::endl;
        return false;
    }

    CD3D11_DEPTH_STENCIL_VIEW_DESC depthStencilViewDesc(D3D11_DSV_DIMENSION_TEXTURE2D);
    const auto depthStencilViewResult = d3dDevice->CreateDepthStencilView(depthStencilTexture.Get(),
        &depthStencilViewDesc, depthStencilView.ReleaseAndGetAddressOf());
    if (FAILED(depthStencilViewResult) == true)
    {
        std::cout << "Failed to create a depth/stencil view: " << depthStencilViewResult << std::endl;
        return false;
    }

    D3D11_BLEND_DESC blendDesc;
    ZeroMemory(&blendDesc, sizeof(D3D11_BLEND_DESC));

    blendDesc.RenderTarget[0].BlendEnable = TRUE;
    blendDesc.RenderTarget[0].SrcBlend = D3D11_BLEND_SRC_ALPHA;
    blendDesc.RenderTarget[0].DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
    blendDesc.RenderTarget[0].BlendOp = D3D11_BLEND_OP_ADD;
    blendDesc.RenderTarget[0].SrcBlendAlpha = D3D11_BLEND_INV_DEST_ALPHA;
    blendDesc.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_ONE;
    blendDesc.RenderTarget[0].BlendOpAlpha = D3D11_BLEND_OP_ADD;
    blendDesc.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;

    /*blendDesc.RenderTarget[0].BlendEnable = TRUE;
    blendDesc.RenderTarget[0].SrcBlend = D3D11_BLEND_SRC_ALPHA;
    blendDesc.RenderTarget[0].DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
    blendDesc.RenderTarget[0].BlendOp = D3D11_BLEND_OP_ADD;
    blendDesc.RenderTarget[0].SrcBlendAlpha = D3D11_BLEND_ONE;
    blendDesc.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_ZERO;
    blendDesc.RenderTarget[0].BlendOpAlpha = D3D11_BLEND_OP_ADD;
    blendDesc.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;*/

    const auto blendResult = d3dDevice->CreateBlendState(&blendDesc, blendState.ReleaseAndGetAddressOf());
    if (FAILED(blendResult) == true)
    {
        std::cout << "Failed to create blend state: " << blendResult << std::endl;
        return false;
    }

    const float blendFactor[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    const UINT blendMask = -1;
    d3dContext->OMSetBlendState(blendState.Get(), blendFactor, blendMask);

    d3dContext->OMSetRenderTargets(1, renderTargetView.GetAddressOf(), depthStencilView.Get());

    // set the viewport
    viewport = CD3D11_VIEWPORT(0.0f, 0.0f, static_cast<float>(windowSize.width), static_cast<float>(windowSize.height));
    d3dContext->RSSetViewports(1, &viewport);

    return true;
}


void DeviceResources::present()
{
    // The first argument instructs DXGI to block until VSync, putting the application
    // to sleep until the next VSync. This ensures we don't waste any cycles rendering
    // frames that will never be displayed to the screen.
    HRESULT hr = swapChain->Present(1, 0);

    // If the device was reset we must completely reinitialize the renderer.
    if (hr == DXGI_ERROR_DEVICE_REMOVED || hr == DXGI_ERROR_DEVICE_RESET)
    {
        on_device_lost();
    }
    else
    {
        DX::ThrowIfFailed(hr);
    }
}

const WindowSize &DeviceResources::get_output_size() const
{
    return windowSize;
}

ID3D11Device1 *DeviceResources::get_device() const
{
    return d3dDevice.Get();
}

ID3D11DeviceContext1 *DeviceResources::get_context() const
{
    return d3dContext.Get();
}

IDXGISwapChain1 *DeviceResources::get_swap_chain() const
{
    return swapChain.Get();
}

D3D_FEATURE_LEVEL DeviceResources::get_feature_level() const
{
    return featureLevel;
}

ID3D11Texture2D *DeviceResources::get_render_target() const
{
    return renderTargetTexture.Get();
}

ID3D11Texture2D *DeviceResources::get_depth_stencil() const
{
    return depthStencilTexture.Get();
}

ID3D11RenderTargetView *DeviceResources::get_render_target_view() const
{
    return renderTargetView.Get();
}

ID3D11DepthStencilView *DeviceResources::get_depth_stencil_view() const
{
    return depthStencilView.Get();
}

DXGI_FORMAT DeviceResources::get_back_buffer_format() const
{
    return backBufferFormat;
}

DXGI_FORMAT DeviceResources::get_depth_buffer_format() const
{
    return depthBufferFormat;
}

UINT DeviceResources::get_back_buffer_count() const
{
    return backBufferCount;
}

DirectX::CommonStates *DeviceResources::get_states() const
{
    return states.get();
}

DirectX::EffectFactory *DeviceResources::get_effect_factory() const
{
    return fxFactory.get();
}

HWND DeviceResources::get_window() const
{
    return window;
}

const D3D11_VIEWPORT &DeviceResources::get_viewport() const
{
    return viewport;
}

ID3D11BlendState *DeviceResources::get_blend_state() const
{
    return blendState.Get();
}

bool DeviceResources::create_window(HINSTANCE hInstance, int nCmdShow, int width, int height)
{
    std::cout << "Creating window..." << std::endl;

    RECT rc;
    rc.top = 0;
    rc.left = 0;
    rc.right = static_cast<LONG>((width > 0) ? width : defaultWindowWidth);
    rc.bottom = static_cast<LONG>((height > 0) ? height : defaultWindowHeight);

    AdjustWindowRect(&rc, WS_OVERLAPPEDWINDOW, FALSE);

    window = CreateWindowEx(0, windowClassName, windowTitle, WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT, CW_USEDEFAULT, rc.right - rc.left, rc.bottom - rc.top, nullptr,
        nullptr, hInstance, nullptr);

    if (!window)
    {
        std::cout << "Failed to create window" << std::endl;
        return false;
    }

    ShowWindow(window, nCmdShow);

    GetClientRect(window, &rc);
    windowSize.width = rc.right - rc.left;
    windowSize.height = rc.bottom - rc.top;

    return true;
}

void DeviceResources::on_window_size_change(int width, int height)
{
    if (width != windowSize.width || height != windowSize.height)
    {
        std::cout << "Resizing window..." << std::endl;
        windowSize.width = width;
        windowSize.height = height;

        if (d3dContext)
        {
            create_resources();
        }

    }
}

void DeviceResources::on_device_lost()
{
    depthStencilView.Reset();

    renderTargetView.Reset();

    depthStencilTexture.Reset();

    renderTargetTexture.Reset();

    swapChain.Reset();

    d3dContext.Reset();

    d3dDevice.Reset();

    states.reset();

    fxFactory.reset();

    if (create_device() == false)
    {
        if (engine)
        {
            engine->stop_engine();
        }
    }

    if (create_resources() == false)
    {
        if (engine)
        {
            engine->stop_engine();
        }
    }
}

void DeviceResources::prepare(const Vec4 &clearColor)
{
    // clear the views
    d3dContext->ClearRenderTargetView(renderTargetView.Get(), &clearColor.x);
    d3dContext->ClearDepthStencilView(depthStencilView.Get(), D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);

    
}

