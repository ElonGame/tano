#pragma once

namespace tano
{
  struct UpdateState;

  //------------------------------------------------------------------------------
  struct Camera
  {
    Camera();
    virtual void Update(const UpdateState& state);

    void GetFrustumCorners(Vector3* pts);
    void GetFrustumCenter(Vector3* pts);

    float _yaw = 0;
    float _pitch = 0;
    float _roll = 0;
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
  struct FollowCam : public Camera
  {
    virtual void Update(const UpdateState& state) override;

    void SetFollowTarget(const Vector3& followTarget);

    Vector3 _followTarget;
  };
}