#pragma once

namespace tano
{
  struct Camera
  {
    Camera();
    void Update();

    Quaternion _rot;
    Vector3 _pos;

    Matrix _view;
    Matrix _proj;
  };
}