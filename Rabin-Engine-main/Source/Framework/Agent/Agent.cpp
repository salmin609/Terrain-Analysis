/******************************************************************************/
/*!
\file		Agent.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Base actor implementation

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include "Agent.h"

const Vec3 globalUp = Vec3(0.0f, 1.0f, 0.0f);
const Vec3 globalRight = Vec3(-1.0f, 0.0f, 0.0f);
const Vec3 globalForward = Vec3(0.0f, 0.0f, 1.0f);

// TODO: move this temp model stuff to renderer in some capacity, if time permits
std::unique_ptr<DirectX::Model> Agent::model;

Agent::Agent(const char *type, size_t id) : position(0.0f, 0.0f, 0.0f),
    scaling(3.0f, 3.0f, 3.0f), eulerAngles(0.0f, 0.0f, 0.0f), isDirty(true), color(0.7f, 0.7f, 0.7f), type(type), id(id), movementSpeed(2000.0f / 2.3f)
{}

#pragma region Getters

const Vec3 &Agent::get_position() const
{
    return position;
}

const Vec3 &Agent::get_scaling() const
{
    return scaling;
}

Vec3 Agent::get_forward_vector() const
{
    const auto rotation = Quat::CreateFromYawPitchRoll(eulerAngles.y, eulerAngles.x, eulerAngles.z);
    return Vec3::Transform(globalForward, rotation);
}

Vec3 Agent::get_right_vector() const
{
    const auto rotation = Quat::CreateFromYawPitchRoll(eulerAngles.y, eulerAngles.x, eulerAngles.z);
    return Vec3::Transform(globalRight, rotation);
}

Vec3 Agent::get_up_vector() const
{
    const auto rotation = Quat::CreateFromYawPitchRoll(eulerAngles.y, eulerAngles.x, eulerAngles.z);
    return Vec3::Transform(globalUp, rotation);
}

float Agent::get_pitch() const
{
  return eulerAngles.x;
}

float Agent::get_yaw() const
{
    return eulerAngles.y;
}

float Agent::get_roll() const
{
    return eulerAngles.z;
}

const Vec3 &Agent::get_color() const
{
    return color;
}

const char *Agent::get_type() const
{
    return type;
}

const size_t &Agent::get_id() const
{
    return id;
}

float Agent::get_movement_speed() const
{
    return movementSpeed;
}

const Mat4 &Agent::get_local_to_world()
{
    if (isDirty == true)
    {
        build_transformation();
        isDirty = false;
    }

    return localToWorld;
}

#pragma endregion

#pragma region Setters
void Agent::set_position(const Vec3 &pos)
{
    position = pos;
    isDirty = true;
}

void Agent::set_scaling(const Vec3 &scale)
{
    if(scale.x > 0.f && scale.y > 0.f && scale.z > 0.f)
    {
        scaling = scale;
        isDirty = true;
    }
}

void Agent::set_scaling(float scalar)
{
    scaling = Vec3(scalar, scalar, scalar);
    isDirty = true;
}

void Agent::set_pitch(float angleRadians)
{
    eulerAngles.x = angleRadians;
    isDirty = true;
}

void Agent::set_yaw(float angleRadians)
{
    eulerAngles.y = angleRadians;
    isDirty = true;
}

void Agent::set_roll(float angleRadians)
{
    eulerAngles.z = angleRadians;
    isDirty = true;
}

void Agent::set_color(const Vec3 &newColor)
{
    color = newColor;
}

void Agent::set_movement_speed(float speed)
{
    movementSpeed = speed;
}
#pragma endregion

void Agent::update(float dt)
{}

void Agent::draw_mesh()
{
    model->UpdateEffects([this](DirectX::IEffect *effect)
    {
        auto basic = dynamic_cast<DirectX::BasicEffect *>(effect);
        basic->SetDiffuseColor(color);
    });
    
    auto &resources = renderer->get_resources();
    model->Draw(resources.get_context(), *resources.get_states(), get_local_to_world(), renderer->get_view_matrix(), renderer->get_projection_matrix());
}

void Agent::draw_debug()
{
    // no op
}

bool Agent::acquire_rendering_resources()
{
    std::cout << "Loading Agent model..." << std::endl;
    auto device = renderer->get_resources().get_device();
    auto factory = renderer->get_resources().get_effect_factory();
    try
    {
        model = DirectX::Model::CreateFromSDKMESH(device, L"Assets\\mannequin_rig.sdkmesh", *factory);
    }
    catch (const std::exception &err)
    {
        std::cout << "Unable to load Agent model: " << err.what() << std::endl;
        return false;
    }

    model->UpdateEffects([](DirectX::IEffect *effect)
    {
        auto basic = dynamic_cast<DirectX::BasicEffect*>(effect);
        if (basic)
        {
            basic->SetLightingEnabled(true);
            basic->SetPerPixelLighting(true);
            basic->SetLightEnabled(0, true);
            basic->SetLightDiffuseColor(0, DirectX::Colors::White);

            basic->SetLightEnabled(1, false);
            basic->SetLightEnabled(2, false);
        }
    });

    return true;
}

void Agent::release_rendering_resources()
{
    model.reset();
}

void Agent::update_model(const Mat4 &view, const Mat4 &proj)
{
    model->UpdateEffects([view, proj](DirectX::IEffect *effect)
    {
        auto basic = dynamic_cast<DirectX::BasicEffect *>(effect);
        basic->SetView(view);
        basic->SetProjection(proj);
    });
}

void Agent::build_transformation()
{
    const auto translationMatrix = Mat4::CreateTranslation(position);
    const auto scalingMatrix = Mat4::CreateScale(scaling * globalScalar);
    const auto rotation = Quat::CreateFromYawPitchRoll(eulerAngles.y, eulerAngles.x, eulerAngles.z);
    const auto rotationMatrix = Mat4::CreateFromQuaternion(rotation);

    localToWorld = scalingMatrix * rotationMatrix * translationMatrix;
}
