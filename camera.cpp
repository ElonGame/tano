#include "camera.hpp"
#include "tano.hpp"
#include "mesh_utils.hpp"
#include "graphics.hpp"
#include "update_state.hpp"

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

//------------------------------------------------------------------------------
void Camera::Update(const UpdateState& updateState)
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

//------------------------------------------------------------------------------
void FollowCam::Update(const UpdateState& state)
{
  // Seek behavior
  //Vector3 tmp = _followTarget - _pos;;
  //tmp.Normalize();
  //Vector3 desiredVel = tmp * 10.f; // _settings.boids.max_speed;
  //return desiredVel - boid.vel;
}

//------------------------------------------------------------------------------
void FollowCam::SetFollowTarget(const Vector3& followTarget)
{
  _followTarget = followTarget;
}
