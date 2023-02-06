/******************************************************************************/
/*!
\file		TextRenderer.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Rendering system using DirectWrite to render text

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "TextRenderer.h"

#include "UI/UITextObject.h"

void TextRenderer::draw_text(const std::wstring &text, UIAnchor anchor,
    const UIBoundingBox &bounds, TextColor color)
{
    const D2D1_RECT_F box(bounds);

    context->DrawTextW(text.c_str(), static_cast<UINT32>(text.size()),
        fonts[static_cast<size_t>(anchor)].Get(),
        &box, brushes[static_cast<size_t>(color)].Get());
}

bool TextRenderer::initialize()
{
    return create_factories() && 
        create_devices() && 
        create_context() && 
        set_render_target() &&
        load_brushes() &&
        load_fonts();
}

void TextRenderer::reset()
{
    // release things in reverse initialize order

    for (auto && font : fonts)
    {
        font.Reset();
    }

    for (auto && brush : brushes)
    {
        brush.Reset();
    }

    targetBitmap.Reset();
    dxgiBuffer.Reset();
    context.Reset();
    device.Reset();
    dxgiDevice.Reset();
    factory.Reset();
    writeFactory.Reset();
}

void TextRenderer::on_window_size_change()
{
    if (context)
    {
        context->SetTarget(nullptr);
        set_render_target();
    }
}

void TextRenderer::draw()
{
    context->BeginDraw();

    project->draw_text();

    context->EndDraw();
}

bool TextRenderer::create_factories()
{
    auto result = DWriteCreateFactory(DWRITE_FACTORY_TYPE_SHARED, __uuidof(IDWriteFactory1), &writeFactory);
    if (FAILED(result) == true)
    {
        std::cout << "Failed to create DirectWrite factory: " << result << std::endl;
        return false;
    }

    D2D1_FACTORY_OPTIONS options;
    #ifndef NDEBUG
    options.debugLevel = D2D1_DEBUG_LEVEL_INFORMATION;
    #else
    options.debugLevel = D2D1_DEBUG_LEVEL_NONE;
    #endif

    result = D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, __uuidof(ID2D1Factory1), &options, &factory);
    if (FAILED(result) == true)
    {
        std::cout << "Failed to create Direct2D factory: " << result << std::endl;
        return false;
    }

    return true;
}

bool TextRenderer::create_devices()
{
    auto result = renderer->get_resources().get_device()->QueryInterface(__uuidof(IDXGIDevice), &dxgiDevice);
    if (FAILED(result) == true)
    {
        std::cout << "Failed to query the DXGI device: " << result << std::endl;
        return false;
    }

    result = factory->CreateDevice(dxgiDevice.Get(), &device);
    if (FAILED(result) == true)
    {
        std::cout << "Failed to create Direct2D device: " << result << std::endl;
        return false;
    }

    return true;
}

bool TextRenderer::create_context()
{
    const auto result = device->CreateDeviceContext(D2D1_DEVICE_CONTEXT_OPTIONS_ENABLE_MULTITHREADED_OPTIMIZATIONS, &context);
    if (FAILED(result) == true)
    {
        std::cout << "Failed to create Direct2D context: " << result << std::endl;
        return false;
    }

    return true;
}

bool TextRenderer::set_render_target()
{
    // GetDesktopDpi deprecated in SDK version 10.0.18362.0,
    // suggested replacement is not backwards compatible
#pragma warning(push)
#pragma warning(disable:4996)
    float dpiX;
    float dpiY;
    factory->GetDesktopDpi(&dpiX, &dpiY);
#pragma warning(pop)

    D2D1_BITMAP_PROPERTIES1 bp;
    bp.pixelFormat.format = DXGI_FORMAT_B8G8R8A8_UNORM;
    bp.pixelFormat.alphaMode = D2D1_ALPHA_MODE_IGNORE;
    bp.dpiX = dpiX;
    bp.dpiY = dpiY;
    bp.bitmapOptions = D2D1_BITMAP_OPTIONS_TARGET | D2D1_BITMAP_OPTIONS_CANNOT_DRAW;
    bp.colorContext = nullptr;

    auto swap = renderer->get_resources().get_swap_chain();
    auto result = swap->GetBuffer(0, __uuidof(IDXGISurface), &dxgiBuffer);
    if (FAILED(result) == true)
    {
        std::cout << "Failed to retrieve back buffer: " << result << std::endl;
        return false;
    }

    result = context->CreateBitmapFromDxgiSurface(dxgiBuffer.Get(), &bp, &targetBitmap);
    if (FAILED(result) == true)
    {
        std::cout << "Failed to create bitmap from DXGI surface: " << result << std::endl;
        return false;
    }

    context->SetTarget(targetBitmap.Get());

    return true;
}

bool TextRenderer::load_brushes()
{
    auto result = context->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Black),
        &brushes[static_cast<size_t>(TextColor::BLACK)]);
    if (FAILED(result) == true)
    {
        std::cout << "Failed to create black font brush: " << result << std::endl;
        return false;
    }

    result = context->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::White),
        &brushes[static_cast<size_t>(TextColor::WHITE)]);
    if (FAILED(result) == true)
    {
        std::cout << "Failed to create white font brush: " << result << std::endl;
        return false;
    }

    return true;
}

bool TextRenderer::load_font(size_t index, DWRITE_TEXT_ALIGNMENT t, DWRITE_PARAGRAPH_ALIGNMENT p)
{
    static const auto fontName = L"Microsoft Sans Serif";
    static const auto locale = L"en-us";
    static const auto size = 16.0f;

    auto result = writeFactory->CreateTextFormat(fontName, nullptr, DWRITE_FONT_WEIGHT_NORMAL,
        DWRITE_FONT_STYLE_NORMAL, DWRITE_FONT_STRETCH_NORMAL, size, locale, &fonts[index]);
    if (FAILED(result) == true)
    {
        std::cout << "Failed to load font: " << result << std::endl;
        return false;
    }

    result = fonts[index]->SetTextAlignment(t);
    if (FAILED(result) == true)
    {
        std::cout << "Failed to set font text alignment: " << result << std::endl;
        return false;
    }

    result = fonts[index]->SetParagraphAlignment(p);
    if (FAILED(result) == true)
    {
        std::cout << "Failed to set font paragraph alignment: " << result << std::endl;
        return false;
    }

    return true;
}

bool TextRenderer::load_fonts()
{
    return load_font(0, DWRITE_TEXT_ALIGNMENT_LEADING, DWRITE_PARAGRAPH_ALIGNMENT_NEAR) &&  // top left
        load_font(1, DWRITE_TEXT_ALIGNMENT_CENTER, DWRITE_PARAGRAPH_ALIGNMENT_NEAR) &&      // top
        load_font(2, DWRITE_TEXT_ALIGNMENT_TRAILING, DWRITE_PARAGRAPH_ALIGNMENT_NEAR) &&    // top right
        load_font(3, DWRITE_TEXT_ALIGNMENT_LEADING, DWRITE_PARAGRAPH_ALIGNMENT_CENTER) &&   // left
        load_font(4, DWRITE_TEXT_ALIGNMENT_CENTER, DWRITE_PARAGRAPH_ALIGNMENT_CENTER) &&    // center
        load_font(5, DWRITE_TEXT_ALIGNMENT_TRAILING, DWRITE_PARAGRAPH_ALIGNMENT_CENTER) &&  // right
        load_font(6, DWRITE_TEXT_ALIGNMENT_LEADING, DWRITE_PARAGRAPH_ALIGNMENT_FAR) &&      // bottom left
        load_font(7, DWRITE_TEXT_ALIGNMENT_CENTER, DWRITE_PARAGRAPH_ALIGNMENT_FAR) &&       // bottom
        load_font(8, DWRITE_TEXT_ALIGNMENT_TRAILING, DWRITE_PARAGRAPH_ALIGNMENT_FAR);       // bottom right
}
