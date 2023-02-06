/******************************************************************************/
/*!
\file		UISpriteRenderer.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Rendering system for ui elements

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "UISpriteRenderer.h"

bool UISpriteRenderer::initialize()
{
    return create_sprite_batch() &&
        create_blend_state() &&
        create_textures();
}

void UISpriteRenderer::reset()
{
    spriteBatch.reset();
    buttonTexture.Reset();
    sliderBarTexture.Reset();
    sliderKnobTexture.Reset();
}

void UISpriteRenderer::draw_button(const UIBoundingBox &bb, const Color &color)
{
    spriteBatch->Draw(buttonTexture.Get(), bb, color);
}

void UISpriteRenderer::draw_slider(const UIBoundingBox &barBox, const UIBoundingBox &knobBox, const Color &barColor, const Color &knobColor)
{
    spriteBatch->Draw(sliderBarTexture.Get(), barBox, barColor);
    spriteBatch->Draw(sliderKnobTexture.Get(), knobBox, knobColor);
}

void UISpriteRenderer::draw()
{
    spriteBatch->Begin(DirectX::SpriteSortMode_Deferred, blendState.Get());

    if (ui)
    {
        ui->draw_sprites();
    }

    spriteBatch->End();
}

void UISpriteRenderer::on_window_size_change()
{
    if (ui)
    {
        ui->on_window_size_change();
    }
}

bool UISpriteRenderer::create_sprite_batch()
{
    auto context = renderer->get_resources().get_context();

    try
    {
        spriteBatch = std::make_unique<DirectX::SpriteBatch>(context);
    }
    catch (const std::exception &err)
    {
        std::cout << "Failed to create ui sprite batch: " << err.what() << std::endl;
        return false;
    }

    return true;
}

bool UISpriteRenderer::create_blend_state()
{
    D3D11_BLEND_DESC blendDesc;
    ZeroMemory(&blendDesc, sizeof(D3D11_BLEND_DESC));

    blendDesc.RenderTarget[0].BlendEnable = TRUE;
    blendDesc.RenderTarget[0].SrcBlend = D3D11_BLEND_SRC_ALPHA;
    blendDesc.RenderTarget[0].DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
    blendDesc.RenderTarget[0].BlendOp = D3D11_BLEND_OP_ADD;
    blendDesc.RenderTarget[0].SrcBlendAlpha = D3D11_BLEND_ONE;
    blendDesc.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_ONE;
    blendDesc.RenderTarget[0].BlendOpAlpha = D3D11_BLEND_OP_ADD;
    blendDesc.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;

    const auto blendResult = renderer->get_resources().get_device()->CreateBlendState(&blendDesc, blendState.ReleaseAndGetAddressOf());
    if (FAILED(blendResult) == true)
    {
        std::cout << "Failed to create ui blend state: " << blendResult << std::endl;
        return false;
    }

    return true;
}

bool UISpriteRenderer::create_textures()
{
    auto device = renderer->get_resources().get_device();

    return create_texture(L"Assets\\button.dds", buttonTexture, device) &&
        create_texture(L"Assets\\slider_bar.dds", sliderBarTexture, device) &&
        create_texture(L"Assets\\slider_knob.dds", sliderKnobTexture, device);
}

bool UISpriteRenderer::create_texture(const wchar_t *file,
    Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> &texture, ID3D11Device1 *device)
{
    Microsoft::WRL::ComPtr<ID3D11Resource> resource;
    const auto result = DirectX::CreateDDSTextureFromFile(device, file,
        resource.GetAddressOf(), texture.ReleaseAndGetAddressOf());

    if (FAILED(result) == true)
    {
        std::wcout << L"Failed to load ui texture " << file << ": " << result << std::endl;
        return false;
    }

    return true;
}
