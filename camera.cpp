#include "camera.hpp"
#include "tano.hpp"
#include "mesh_utils.hpp"
#include "graphics.hpp"
#include "update_state.hpp"
#include "tano_math.hpp"

using namespace tano;
using namespace bristol;

//------------------------------------------------------------------------------
void Camera::FromProtocol(const CameraSettings& settings)
{
  _pos = settings.pos;
  _dir = settings.dir;
  _right = settings.right;
  _up = settings.up;

  _fov = settings.fov;
  _aspectRatio = settings.aspect_ratio;
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
  settings->aspect_ratio = _aspectRatio;
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
  GRAPHICS.GetBackBufferSize(&w, &h);
  _aspectRatio = (float)w / h;
  _proj = Matrix::CreatePerspectiveFieldOfView(_fov, _aspectRatio, _nearPlane, _farPlane);
}

//------------------------------------------------------------------------------
void Camera::Update(const FixedUpdateState& state)
{
  _proj = Matrix::CreatePerspectiveFieldOfView(_fov, _aspectRatio, _nearPlane, _farPlane);
  Vector3 target = _pos + _dir;
  _view = Matrix::CreateLookAt(_pos, target, _up);
}

//------------------------------------------------------------------------------
void FreeflyCamera::Update(const FixedUpdateState& updateState)
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
  _dir = Vector3::Transform(Vector3(0, 0, 1), _mtx);
  _up = Vector3::Transform(Vector3(0, 1, 0), _mtx);
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

  Vector3 target = _pos + _dir;
  _view = Matrix::CreateLookAt(_pos, target, _up);
  _proj = Matrix::CreatePerspectiveFieldOfView(_fov, _aspectRatio, _nearPlane, _farPlane);
}

//------------------------------------------------------------------------------
FollowCam::FollowCam()
  : _seek(50, 50)
{
  _particle.Init(1);
  _particle._bodies.pos[0] = { 0, 0, 0 };
  _particle.AddKinematics(&_seek, 1);
  _particle._maxSpeed = 50;
}

//------------------------------------------------------------------------------
FollowCam::~FollowCam()
{
  SeqDelete(&_kinematics);
}

//------------------------------------------------------------------------------
void FollowCam::Update(const FixedUpdateState& state)
{
  _particle.Update(state, true);
  XMVECTOR* pos = _particle._bodies.pos;
  XMVECTOR* vel = _particle._bodies.vel;

  _pos = Vector3(pos[0]);
  Vector3 target = _pos + Vector3(vel[0]);
  _view = Matrix::CreateLookAt(_pos, target, _up);
}

//------------------------------------------------------------------------------
void FollowCam::SetMaxSpeedAndForce(float maxSpeed, float maxForce)
{
  _seek.maxSpeed = maxSpeed;
  _seek.maxForce = maxForce;
  _particle._maxSpeed;
}

//------------------------------------------------------------------------------
void FollowCam::SetFollowTarget(const FXMVECTOR& followTarget)
{
  _seek.target = followTarget;
}

//------------------------------------------------------------------------------
void FollowCam::AddKinematic(ParticleKinematics* k, float weight)
{
  _particle.AddKinematics(k, weight);
  _kinematics.push_back(k);
}
