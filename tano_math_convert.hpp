#pragma once
#include "b2/dx/DirectXTK/SimpleMath.h"
#include "tano_math.hpp"


namespace tano
{
  //------------------------------------------------------------------------------
  inline DirectX::SimpleMath::Vector3 ToVector3(const vec3& v)
  {
    return DirectX::SimpleMath::Vector3(v.x, v.y, v.z);
  }

  //------------------------------------------------------------------------------
  inline vec3 FromVector3(const DirectX::SimpleMath::Vector3& v)
  {
    return vec3(v.x, v.y, v.z);
  }

  //------------------------------------------------------------------------------
  inline DirectX::SimpleMath::Vector4 ToVector4(const vec4& v)
  {
    return DirectX::SimpleMath::Vector4(v.x, v.y, v.z, v.w);
  }

  //------------------------------------------------------------------------------
  inline DirectX::SimpleMath::Vector4 Expand(const DirectX::SimpleMath::Vector3& v, float x)
  {
    return DirectX::SimpleMath::Vector4(v.x, v.y, v.z, x);
  }

}