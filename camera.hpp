#pragma once
#include "dyn_particles.hpp"

namespace tano
{
  struct UpdateState;

  //------------------------------------------------------------------------------
  struct Camera
  {
    Camera();
    virtual ~Camera() {}
    virtual void Update(const FixedUpdateState& state) {}

    Vector3 _pos = Vector3(0,0,0);
    Vector3 _target;
    Vector3 _dir = Vector3(0, 0, 1);
    Vector3 _right = Vector3(1, 0, 0);
    Vector3 _up = Vector3(0,1,0);

    float _fov = XMConvertToRadians(60);
    float _aspectRatio;
    float _nearPlane = 1.f;
    float _farPlane = 2000.f;

    Matrix _mtx;
    Matrix _view;
    Matrix _proj;
  };

  //------------------------------------------------------------------------------
  struct FreeFlyCamera : public Camera
  {
    virtual void Update(const FixedUpdateState& state) override;

    float _yaw = 0;
    float _pitch = 0;
    float _roll = 0;
  };

  //------------------------------------------------------------------------------
  struct FollowCam : public Camera
  {
    FollowCam();
    ~FollowCam();
    virtual void Update(const FixedUpdateState& state) override;
    void SetFollowTarget(const FXMVECTOR& followTarget);
    void AddKinematic(ParticleKinematics* k, float weight = 1);

    DynParticles _particle;
    BehaviorSeek _seek;
    vector<ParticleKinematics*> _kinematics;
  };
}