#pragma once
#include "DirectXTK/SimpleMath.h"
#pragma warning(push)
#pragma warning(disable: 4005)
#include <dxgi.h>
#include <dxgidebug.h>
#include <d3d11.h>
#pragma warning(pop)

namespace bristol
{
  enum VertexFlags
  {
    VF_POS            = 1 << 0,
    VF_POS_XY         = 1 << 1,
    VF_NORMAL         = 1 << 4,
    VF_TEX2_0         = 1 << 7,
    VF_TEX3_0         = 1 << 8,
    VF_COLOR          = 1 << 10,
    VF_COLOR_U32      = 1 << 11,

    // flags specifying order
    VF_ORDER_TEX_COL  = 1 << 16,
    VF_ORDER_CUSTOM   = 1 << 17,
  };

  struct CD3D11_INPUT_ELEMENT_DESC : public D3D11_INPUT_ELEMENT_DESC
  {
    CD3D11_INPUT_ELEMENT_DESC(LPCSTR name, DXGI_FORMAT format)
    {
      SemanticName = name;
      SemanticIndex = 0;
      Format = format;
      InputSlot = 0;
      AlignedByteOffset = 0;
      InputSlotClass = D3D11_INPUT_PER_VERTEX_DATA;
      InstanceDataStepRate = 0;
    }
  };

  uint32_t VertexSizeFromFlags(uint32_t flags);
  uint32_t IndexSizeFromFormat(DXGI_FORMAT format);
  uint32_t SizeFromFormat(DXGI_FORMAT format);

  DirectX::SimpleMath::Vector3 GetRow(const DirectX::SimpleMath::Matrix& m, uint32_t row);

  uint32_t ColorToU32(float r, float g, float b, float a);

  float Raycast(
      const DirectX::SimpleMath::Vector3& center,
      float radius,
      const DirectX::SimpleMath::Vector3& o,
      const DirectX::SimpleMath::Vector3& d);

  template <typename T>
  float Dot(const T& a, const T& b)
  {
    return a.Dot(b);
  }

  // (w,h) = backbuffer size
  DirectX::SimpleMath::Vector3 ScreenToViewSpace(
    const DirectX::SimpleMath::Matrix& proj,
    uint32_t x,
    uint32_t y,
    int backbufferWidth,
    int backbufferHeight);

  //------------------------------------------------------------------------------
  //inline DirectX::SimpleMath::Vector3 Cross(const DirectX::SimpleMath::Vector3& a, const DirectX::SimpleMath::Vector3& b)
  //{
  //  return DirectX::SimpleMath::Vector3(
  //    (a.y * b.z) - (a.z * b.y),
  //    (a.z * b.x) - (a.x * b.z),
  //    (a.x * b.y) - (a.y * b.x));
  //}

  //------------------------------------------------------------------------------
  inline void Cross(
    float ax, float ay, float az,
    float bx, float by, float bz,
    float* nx, float* ny, float* nz)
  {
    *nx = (ay * bz) - (az * by);
    *ny = (az * bx) - (ax * bz);
    *nz = (ax * by) - (ay * bx);
  }

  //------------------------------------------------------------------------------

  //inline DirectX::SimpleMath::Vector3 Normalize(const DirectX::SimpleMath::Vector3& v)
  //{
  //  float len = v.Length();
  //  if (len == 0.f)
  //    return DirectX::SimpleMath::Vector3(0,0,0);
  //  return 1/len * v;
  //}

  //------------------------------------------------------------------------------
  inline void Normalize(float* x, float* y, float* z)
  {
    float xx = *x;
    float yy = *y;
    float zz = *z;

    float len = sqrtf(xx*xx + yy*yy + zz*zz);
    if (len == 0)
      return;

    float recipLen = 1.f / sqrtf(xx*xx + yy*yy + zz*zz);
    *x = xx * recipLen;
    *y = yy * recipLen;
    *z = zz * recipLen;
  }

  //------------------------------------------------------------------------------
  void ExtractPlanes(
    const DirectX::SimpleMath::Matrix& mtx,
    bool normalize,
    DirectX::SimpleMath::Plane* planes);

  //------------------------------------------------------------------------------
  inline float DistanceToPoint(const DirectX::SimpleMath::Plane& plane, const DirectX::SimpleMath::Vector3& pt)
  {
    return plane.x * pt.x + plane.y * pt.y + plane.z * pt.z + plane.w;

  }

  //------------------------------------------------------------------------------
  inline float Dot(const DirectX::SimpleMath::Plane& plane, const DirectX::SimpleMath::Vector3& pt)
  {
    // Note, this is just the dot product between the plane's normal and pt
    return plane.x * pt.x + plane.y * pt.y + plane.z * pt.z;
  }

  //------------------------------------------------------------------------------
}