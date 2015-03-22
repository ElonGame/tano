#include "camera.hpp"
#include "tano.hpp"

using namespace tano;
using namespace bristol;

//------------------------------------------------------------------------------
Camera::Camera()
  : _rot(Quaternion::CreateFromAxisAngle(Vector3(1, 0, 0), -XM_PI/10.f))
  , _pos(Vector3(0, 200, 0))
{
  _rot = { 0.f, -1.f, 0.1f, 0.01f };
  _pos = { 38.f, 176.f, 93.f };
  Update();
}

//------------------------------------------------------------------------------
Vector3 FromQuat(const Quaternion& q)
{
  return Vector3(q.x, q.y, q.z);
}

//------------------------------------------------------------------------------
void Camera::Update()
{
  const IoState& state = TANO.GetIoState();

  Quaternion conj;
  _rot.Conjugate(conj);

  // rotation
  if (state.keysPressed['A'])
    _rot = Quaternion::Concatenate(Quaternion::CreateFromAxisAngle(Vector3(0, 1, 0), +0.1f / XM_PI), _rot);

  if (state.keysPressed['D'])
    _rot = _rot * Quaternion::CreateFromAxisAngle(Vector3(0, 1, 0), -0.1f / XM_PI);

  if (state.keysPressed['Q'])
    _rot = Quaternion::CreateFromAxisAngle(Vector3(1, 0, 0), +0.1f / XM_PI) * _rot;

  if (state.keysPressed['E'])
    _rot = Quaternion::CreateFromAxisAngle(Vector3(1, 0, 0), -0.1f / XM_PI) * _rot;

  // movement
  if (state.keysPressed['Z'])
    _pos += FromQuat(_rot * Quaternion(-1, 0, 0, 0) * conj);

  if (state.keysPressed['C'])
    _pos += FromQuat(_rot * Quaternion(+1, 0, 0, 0) * conj);

  if (state.keysPressed['W'])
    _pos += FromQuat(_rot * Quaternion(0, 0, +1, 0) * conj);

  if (state.keysPressed['S'])
    _pos += FromQuat(_rot * Quaternion(0, 0, -1, 0) * conj);

  Matrix tmp = Matrix::CreateFromQuaternion(_rot);
  tmp.Transpose(_view);

//   _view._41 = -Dot(_pos, Vector3(tmp._11, tmp._21, tmp._31));
//   _view._42 = -Dot(_pos, Vector3(tmp._12, tmp._22, tmp._32));
//   _view._43 = -Dot(_pos, Vector3(tmp._13, tmp._23, tmp._33));

  _view._41 = -Dot(_pos, tmp.Right());
  _view._42 = -Dot(_pos, tmp.Up());
  _view._43 = -Dot(_pos, tmp.Forward());

}
