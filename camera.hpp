#pragma once

namespace tano
{
  struct Camera
  {
    Camera();
    void Update();

    float _yaw = 0;
    float _pitch = 0;
    Vector3 _pos = Vector3(0,0,0);
    Vector3 _target = Vector3(0,0,0);
    Vector3 _up = Vector3(0,1,0);

    Matrix _view;
    Matrix _proj;
  };
}