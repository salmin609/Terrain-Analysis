/******************************************************************************/
/*!
\file		UISpriteRenderer.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Rendering system for ui elements

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once

class UIBoundingBox;

class UISpriteRenderer
{
public:
    bool initialize();
    void reset();

    void draw_button(const UIBoundingBox &bb, const Color &color);
    void draw_slider(const UIBoundingBox &barBox, const UIBoundingBox &knobBox,
        const Color &barColor, const Color &knobColor);

    void draw();

    void on_window_size_change();
private:
    std::unique_ptr<DirectX::SpriteBatch> spriteBatch;
    Microsoft::WRL::ComPtr<ID3D11BlendState> blendState;
    Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> buttonTexture;
    Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> sliderBarTexture;
    Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> sliderKnobTexture;

    bool create_sprite_batch();
    bool create_blend_state();
    bool create_textures();
    bool create_texture(const wchar_t *file, Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> &texture, ID3D11Device1 *device);
};