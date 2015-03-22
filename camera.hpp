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

    Matrix _view;
    Matrix _proj;
  };
}