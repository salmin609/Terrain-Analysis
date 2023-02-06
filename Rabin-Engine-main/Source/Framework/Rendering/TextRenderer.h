/******************************************************************************/
/*!
\file		TextRenderer.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Rendering system using DirectWrite to render text

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include <array>

class UIBoundingBox;
enum class UIAnchor;

enum class TextColor
{
    BLACK,
    WHITE,

    NUM_ENTRIES
};

class TextRenderer
{
public:
    void draw_text(const std::wstring &text, UIAnchor anchor, const UIBoundingBox &bounds, TextColor color = TextColor::BLACK);

    bool initialize();
    void reset();
    void on_window_size_change();

    void draw();
private:
    Microsoft::WRL::ComPtr<IDWriteFactory1> writeFactory;
    Microsoft::WRL::ComPtr<ID2D1Factory1> factory;
    Microsoft::WRL::ComPtr<IDXGIDevice> dxgiDevice;
    Microsoft::WRL::ComPtr<ID2D1Device> device;
    Microsoft::WRL::ComPtr<ID2D1DeviceContext> context;
    std::array<Microsoft::WRL::ComPtr<ID2D1SolidColorBrush>, static_cast<size_t>(TextColor::NUM_ENTRIES)> brushes;
    Microsoft::WRL::ComPtr<IDXGISurface> dxgiBuffer;
    Microsoft::WRL::ComPtr<ID2D1Bitmap1> targetBitmap;
    std::array<Microsoft::WRL::ComPtr<IDWriteTextFormat>, 9> fonts;

    bool create_factories();
    bool create_devices();
    bool create_context();
    bool set_render_target();
    bool load_brushes();
    bool load_fonts();
    bool load_font(size_t index, DWRITE_TEXT_ALIGNMENT t, DWRITE_PARAGRAPH_ALIGNMENT p);
};