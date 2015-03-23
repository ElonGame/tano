#include "camera.hpp"
#include "tano.hpp"
#include "mesh_utils.hpp"
#include "graphics.hpp"

using namespace tano;
using namespace bristol;

//------------------------------------------------------------------------------
Camera::Camera()
{
  int w, h;
  GRAPHICS.GetBackBufferSize(&w, &h);
  _proj = Matrix::CreatePerspectiveFieldOfView(_fov, (float)w/h, _nearPlane, _farPlane);
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

  if (state.keysPressed['R'])
    _roll += 0.1f / XM_PI;

  if (state.keysPressed['T'])
    _roll -= 0.1f / XM_PI;

  _mtx = Matrix::CreateFromYawPitchRoll(_yaw, _pitch, _roll);
  Vector3 dir = Vector3::Transform(Vector3(0, 0, 1), _mtx);
  Vector3 up = Vector3::Transform(Vector3(0, 1, 0), _mtx);
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
