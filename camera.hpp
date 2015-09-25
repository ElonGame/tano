#pragma once
#include "dyn_particles.hpp"
#include "generated/demo.types.hpp"

namespace tano
{
  struct UpdateState;

  //------------------------------------------------------------------------------
  struct Camera
  {
    Camera();
    virtual ~Camera() {}
    virtual void Update(float deltaTime);

    void FromProtocol(const CameraSettings& settings);
    void ToProtocol(CameraSettings* settings);

    vec3 _pos = vec3(0,0,0);
    vec3 _dir = vec3(0, 0, 1);
    vec3 _right = vec3(1, 0, 0);
    vec3 _up = vec3(0,1,0);

    float _fov = XMConvertToRadians(60);
    float _aspectRatio;
    float _nearPlane = 1.f;
    float _farPlane = 2000.f;

    Matrix _mtx;
    Matrix _view;
    Matrix _proj;
  };

  //------------------------------------------------------------------------------
  struct FreeflyCamera : public Camera
  {
    virtual void Update(float deltaTime) override;

    void FromProtocol(const FreeflyCameraSettings& settings);
    void ToProtocol(FreeflyCameraSettings* settings);

    float _yaw = 0;
    float _pitch = 0;
    float _roll = 0;
  };

  //------------------------------------------------------------------------------
  struct FollowCam : public Camera
  {
    FollowCam();
    ~FollowCam();
    virtual void Update(float deltaTime) override;
    void SetFollowTarget(const vec3& followTarget);
    void AddKinematic(ParticleKinematics* k, float weight = 1);
    void SetMaxSpeedAndForce(float maxSpeed, float maxForce);

    DynParticles _particle;
    BehaviorSeek _seek;
    vector<ParticleKinematics*> _kinematics;
  };
}