#include "camera.hpp"
#include "tano.hpp"
#include "mesh_utils.hpp"
#include "graphics.hpp"
#include "update_state.hpp"
#include "tano_math.hpp"

using namespace tano;
using namespace bristol;

//------------------------------------------------------------------------------
Camera::Camera()
{
  int w, h;
  GRAPHICS.GetBackBufferSize(&w, &h);
  _aspectRatio = (float)w / h;
  _proj = Matrix::CreatePerspectiveFieldOfView(_fov, _aspectRatio, _nearPlane, _farPlane);
  UpdateState state;
  Update(state);
}

#if 0
//------------------------------------------------------------------------------
void Camera::GetFrustumCorners(Vector3* pts)
{
  float hNear = 2 * tan(_fov / 2) * _nearPlane;
  float wNear = _aspectRatio * hNear;

  float hFar = 2 * tan(_fov / 2) * _farPlane;
  float wFar = _aspectRatio * hFar;

  // because all the points are relative the center of the planes, we just
  // need the half distances
  hNear /= 2;
  wNear /= 2;

  hFar /= 2;
  wFar /= 2;

  // calc the 8 corner points of the view frustum (in world space)

  // 0, 1
  // 2, 3

  // far plane
  Vector3 fc = _pos + _dir * _farPlane;
  pts[0] = fc + hFar * _up - wFar * _right;
  pts[1] = fc + hFar * _up + wFar * _right;
  pts[2] = fc - hFar * _up - wFar * _right;
  pts[3] = fc - hFar * _up + wFar * _right;

  // near plane
  Vector3 nc = _pos + _dir * _nearPlane;
  pts[4] = nc + hNear * _up - wNear * _right;
  pts[5] = nc + hNear * _up + wNear * _right;
  pts[6] = nc - hNear * _up - wNear * _right;
  pts[7] = nc - hNear * _up + wNear * _right;

}

//------------------------------------------------------------------------------
void Camera::GetFrustumCenter(Vector3* pts)
{
  float hNear = 2 * tan(_fov / 2) * _nearPlane;
  float wNear = _aspectRatio * hNear;

  float hFar = 2 * tan(_fov / 2) * _farPlane;
  float wFar = _aspectRatio * hFar;

  // because all the points are relative the center of the planes, we just
  // need the half distances
  hNear /= 2;
  wNear /= 2;

  hFar /= 2;
  wFar /= 2;

  // calc the 4 center points of the view frustum (in world space)

  // 0, 1

  Matrix mtxFlat = Matrix::CreateFromYawPitchRoll(_yaw, 0, 0);
  Vector3 dir = Vector3::Transform(Vector3(0, 0, 1), mtxFlat);
  Vector3 right = Vector3::Transform(Vector3(1, 0, 0), mtxFlat);

  // far plane
  Vector3 fc = _pos + dir * _farPlane;
  pts[0] = fc - wFar * right;
  pts[1] = fc + wFar * right;

  // near plane
  Vector3 nc = _pos + dir * _nearPlane;
  pts[2] = nc - wNear * right;
  pts[3] = nc + wNear * right;

  // center points
  pts[4] = fc;
  pts[5] = nc;
}
#endif

//------------------------------------------------------------------------------
void FreeFlyCamera::Update(const UpdateState& updateState)
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

  _target = _pos + _dir;
  _view = Matrix::CreateLookAt(_pos, _target, _up);
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
void FollowCam::Update(const UpdateState& state)
{
  _particle.Update(state);
  V3* pos = _particle._bodies.pos;
  V3* vel = _particle._bodies.vel;

  //const DynParticles::Body& b = _particle._bodies[0];
  //_pos = Vector3(b.pos.x, b.pos.y, b.pos.z);
  _pos = Vector3(pos[0].x, pos[0].y, pos[0].z);
  _target = _pos + Vector3(vel[0].x, vel[0].y, vel[0].z);
  _view = Matrix::CreateLookAt(_pos, _target, _up);
}

//------------------------------------------------------------------------------
void FollowCam::SetFollowTarget(const V3& followTarget)
{
  _seek.target = followTarget;
}
