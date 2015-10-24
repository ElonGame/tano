#include "camera.hpp"
#include "tano.hpp"
#include "mesh_utils.hpp"
#include "graphics.hpp"
#include "update_state.hpp"
#include "tano_math.hpp"
#include "tano_math_convert.hpp"

using namespace tano;
using namespace bristol;

//------------------------------------------------------------------------------
void Camera::FromProtocol(const CameraSettings& settings)
{
  // TODO(magnus): add the _target and _useTarget flags
  _pos = settings.pos;
  _dir = settings.dir;
  _right = settings.right;
  _up = settings.up;

  _fov = settings.fov;
  _nearPlane = settings.near_plane;
  _farPlane = settings.far_plane;
}

//------------------------------------------------------------------------------
void Camera::ToProtocol(CameraSettings* settings)
{
  settings->pos = _pos;
  settings->dir = _dir;
  settings->right = _right;
  settings->up = _up;

  settings->fov = _fov;
  settings->near_plane = _nearPlane;
  settings->far_plane = _farPlane;
}

//------------------------------------------------------------------------------
void FreeflyCamera::FromProtocol(const FreeflyCameraSettings& settings)
{
  Camera::FromProtocol(settings.camera);
  _yaw = settings.yaw;
  _pitch = settings.pitch;
  _roll = settings.roll;
}

//------------------------------------------------------------------------------
void FreeflyCamera::ToProtocol(FreeflyCameraSettings* settings)
{
  Camera::ToProtocol(&settings->camera);
  settings->yaw = _yaw;
  settings->pitch = _pitch;
  settings->roll = _roll;
}

//------------------------------------------------------------------------------
Camera::Camera()
{
  int w, h;
  g_Graphics->GetBackBufferSize(&w, &h);
  _aspectRatio = (float)w / h;
  _proj = Matrix::CreatePerspectiveFieldOfView(_fov, _aspectRatio, _nearPlane, _farPlane);
}

//------------------------------------------------------------------------------
void Camera::Update(float deltaTime)
{
  _proj = Matrix::CreatePerspectiveFieldOfView(_fov, _aspectRatio, _nearPlane, _farPlane);
  if (!_useTarget)
  {
    _target = _pos + _dir;
  }
  _view = Matrix::CreateLookAt(ToVector3(_pos), ToVector3(_target), ToVector3(_up));
}

//------------------------------------------------------------------------------
void FreeflyCamera::Update(float deltaTime)
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
  _dir = FromVector3(Vector3::Transform(Vector3(0, 0, 1), _mtx));
  _up = FromVector3(Vector3::Transform(Vector3(0, 1, 0), _mtx));
  _right = Cross(_up, _dir);

  // movement
  float s = state.shiftPressed ? 5.f : 1.f;

  if (state.keysPressed['Z'])
    _pos -= s * _right;

  if (state.keysPressed['C'])
    _pos += s * _right;

  if (state.keysPressed['W'])
    _pos += s * _dir;

  if (state.keysPressed['S'])
    _pos -= s * _dir;

  if (state.keysPressed['F'])
    _pos += s * _up;

  if (state.keysPressed['V'])
    _pos -= s * _up;

  vec3 target = _pos + _dir;
  _view = Matrix::CreateLookAt(ToVector3(_pos), ToVector3(target), ToVector3(_up));
  _proj = Matrix::CreatePerspectiveFieldOfView(_fov, _aspectRatio, _nearPlane, _farPlane);
}
