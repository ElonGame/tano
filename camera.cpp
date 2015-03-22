#include "camera.hpp"
#include "tano.hpp"
#include "mesh_utils.hpp"

using namespace tano;
using namespace bristol;

//------------------------------------------------------------------------------
Camera::Camera()
{
  Update();
}

//------------------------------------------------------------------------------
void Camera::Update()
{
  const IoState& state = TANO.GetIoState();

  // rotation
  if (state.keysPressed['A'])
    _yaw -= 0.1f / XM_PI;

  if (state.keysPressed['D'])
    _yaw += 0.1f / XM_PI;

  if (state.keysPressed['Q'])
    _pitch -= 0.1f / XM_PI;

  if (state.keysPressed['E'])
    _pitch += 0.1f / XM_PI;

  Matrix mtx = Matrix::CreateFromYawPitchRoll(_yaw, _pitch, 0);
  Vector3 dir = Vector3::Transform(Vector3(0, 0, 1), mtx);
  Vector3 up = Vector3::Transform(Vector3(0, 1, 0), mtx);
  Vector3 right = Cross(up, dir);

  // movement
  float s = state.shiftPressed ? 5.f : 1.f;

  if (state.keysPressed['Z'])
    _pos -= s * right;

  if (state.keysPressed['C'])
    _pos += s * right;

  if (state.keysPressed['W'])
    _pos += s * dir;

  if (state.keysPressed['S'])
    _pos -= s * dir;

  _target = _pos + dir;
  _up = up;
  _view = Matrix::CreateLookAt(_pos, _target, _up);
}
