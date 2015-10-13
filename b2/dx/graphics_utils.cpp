#include "graphics_utils.hpp"
#include <algorithm>
#include "../utils.hpp"

using namespace DirectX::SimpleMath;
using namespace std;
namespace bristol
{
  uint32_t SizeFromFormat(DXGI_FORMAT format)
  {
    switch (format)
    {
      case DXGI_FORMAT_R32G32B32A32_TYPELESS:
      case DXGI_FORMAT_R32G32B32A32_FLOAT:
      case DXGI_FORMAT_R32G32B32A32_UINT:
      case DXGI_FORMAT_R32G32B32A32_SINT:
        return 128 / 8;

      case DXGI_FORMAT_R32G32B32_TYPELESS:
      case DXGI_FORMAT_R32G32B32_FLOAT:
      case DXGI_FORMAT_R32G32B32_UINT:
      case DXGI_FORMAT_R32G32B32_SINT:
        return 96 / 8;

      case DXGI_FORMAT_R16G16B16A16_TYPELESS:
      case DXGI_FORMAT_R16G16B16A16_FLOAT:
      case DXGI_FORMAT_R16G16B16A16_UNORM:
      case DXGI_FORMAT_R16G16B16A16_UINT:
      case DXGI_FORMAT_R16G16B16A16_SNORM:
      case DXGI_FORMAT_R16G16B16A16_SINT:
      case DXGI_FORMAT_R32G32_TYPELESS:
      case DXGI_FORMAT_R32G32_FLOAT:
      case DXGI_FORMAT_R32G32_UINT:
      case DXGI_FORMAT_R32G32_SINT:
      case DXGI_FORMAT_R32G8X24_TYPELESS:
      case DXGI_FORMAT_D32_FLOAT_S8X24_UINT:
      case DXGI_FORMAT_R32_FLOAT_X8X24_TYPELESS:
      case DXGI_FORMAT_X32_TYPELESS_G8X24_UINT:
        return 64 / 8;

      case DXGI_FORMAT_R10G10B10A2_TYPELESS:
      case DXGI_FORMAT_R10G10B10A2_UNORM:
      case DXGI_FORMAT_R10G10B10A2_UINT:
      case DXGI_FORMAT_R11G11B10_FLOAT:
      case DXGI_FORMAT_R8G8B8A8_TYPELESS:
      case DXGI_FORMAT_R8G8B8A8_UNORM:
      case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB:
      case DXGI_FORMAT_R8G8B8A8_UINT:
      case DXGI_FORMAT_R8G8B8A8_SNORM:
      case DXGI_FORMAT_R8G8B8A8_SINT:
      case DXGI_FORMAT_R16G16_TYPELESS:
      case DXGI_FORMAT_R16G16_FLOAT:
      case DXGI_FORMAT_R16G16_UNORM:
      case DXGI_FORMAT_R16G16_UINT:
      case DXGI_FORMAT_R16G16_SNORM:
      case DXGI_FORMAT_R16G16_SINT:
      case DXGI_FORMAT_R32_TYPELESS:
      case DXGI_FORMAT_D32_FLOAT:
      case DXGI_FORMAT_R32_FLOAT:
      case DXGI_FORMAT_R32_UINT:
      case DXGI_FORMAT_R32_SINT:
      case DXGI_FORMAT_R24G8_TYPELESS:
      case DXGI_FORMAT_D24_UNORM_S8_UINT:
      case DXGI_FORMAT_R24_UNORM_X8_TYPELESS:
      case DXGI_FORMAT_X24_TYPELESS_G8_UINT:
      case DXGI_FORMAT_B8G8R8A8_UNORM:
      case DXGI_FORMAT_B8G8R8X8_UNORM:
        return 32 / 8;

      case DXGI_FORMAT_R8G8_TYPELESS:
      case DXGI_FORMAT_R8G8_UNORM:
      case DXGI_FORMAT_R8G8_UINT:
      case DXGI_FORMAT_R8G8_SNORM:
      case DXGI_FORMAT_R8G8_SINT:
      case DXGI_FORMAT_R16_TYPELESS:
      case DXGI_FORMAT_R16_FLOAT:
      case DXGI_FORMAT_D16_UNORM:
      case DXGI_FORMAT_R16_UNORM:
      case DXGI_FORMAT_R16_UINT:
      case DXGI_FORMAT_R16_SNORM:
      case DXGI_FORMAT_R16_SINT:
      case DXGI_FORMAT_B5G6R5_UNORM:
      case DXGI_FORMAT_B5G5R5A1_UNORM:
        return 16 / 8;

      case DXGI_FORMAT_R8_TYPELESS:
      case DXGI_FORMAT_R8_UNORM:
      case DXGI_FORMAT_R8_UINT:
      case DXGI_FORMAT_R8_SNORM:
      case DXGI_FORMAT_R8_SINT:
      case DXGI_FORMAT_A8_UNORM:
        return 8 / 8;

        // Compressed format; http://msdn2.microsoft.com/en-us/library/bb694531(VS.85).aspx
      case DXGI_FORMAT_BC2_TYPELESS:
      case DXGI_FORMAT_BC2_UNORM:
      case DXGI_FORMAT_BC2_UNORM_SRGB:
      case DXGI_FORMAT_BC3_TYPELESS:
      case DXGI_FORMAT_BC3_UNORM:
      case DXGI_FORMAT_BC3_UNORM_SRGB:
      case DXGI_FORMAT_BC5_TYPELESS:
      case DXGI_FORMAT_BC5_UNORM:
      case DXGI_FORMAT_BC5_SNORM:
        return 128 / 8;

        // Compressed format; http://msdn2.microsoft.com/en-us/library/bb694531(VS.85).aspx
      case DXGI_FORMAT_R1_UNORM:
      case DXGI_FORMAT_BC1_TYPELESS:
      case DXGI_FORMAT_BC1_UNORM:
      case DXGI_FORMAT_BC1_UNORM_SRGB:
      case DXGI_FORMAT_BC4_TYPELESS:
      case DXGI_FORMAT_BC4_UNORM:
      case DXGI_FORMAT_BC4_SNORM:
        return 64 / 8;

        // Compressed format; http://msdn2.microsoft.com/en-us/library/bb694531(VS.85).aspx
      case DXGI_FORMAT_R9G9B9E5_SHAREDEXP:
        return 32 / 8;

        // These are compressed, but bit-size information is unclear.
      case DXGI_FORMAT_R8G8_B8G8_UNORM:
      case DXGI_FORMAT_G8R8_G8B8_UNORM:
        return 32 / 8;

      case DXGI_FORMAT_UNKNOWN:
      default:
        return ~0;
      }
  }
  //------------------------------------------------------------------------------
  uint32_t VertexSizeFromFlags(uint32_t flags)
  {
    uint32_t size = 0;
    if (flags & VF_POS) size += 12;
    if (flags & VF_POS_XY) size += 8;
    if (flags & VF_NORMAL) size += 12;
    if (flags & VF_TEX2_0) size += 8;
    if (flags & VF_TEX3_0) size += 12;
    if (flags & VF_COLOR) size += 16;
    if (flags & VF_COLOR_U32) size += 4;

    return size;
  }

  //------------------------------------------------------------------------------
  uint32_t IndexSizeFromFormat(DXGI_FORMAT format)
  {
    if (format == DXGI_FORMAT_R16_UINT) return 2;
    if (format == DXGI_FORMAT_R32_UINT) return 4;
    assert(!"Unsupported index buffer format");
    return 0;
  }

  //------------------------------------------------------------------------------
  Vector3 ScreenToViewSpace(const Matrix& proj, uint32_t x, uint32_t y, int w, int h)
  {
    float xv = (2.0f * x / w - 1) / proj(0, 0);
    float yv = (-2.0f * y / h + 1) / proj(1, 1);

    return Vector3(xv, yv, 1);
  }

  //------------------------------------------------------------------------------
  float Raycast(const Vector3& center, float radius, const Vector3& o, const Vector3& d)
  {
    // ray -> sphere intersection
    // Compute A, B and C coefficients
    float a = Dot(d, d);
    float b = 2 * Dot(d, o - center);
    float c = Dot(o - center, o - center) - (radius * radius);

    // Find discriminant
    float disc = b * b - 4 * a * c;

    // if discriminant is negative there are no real roots, so ray misses sphere
    if (disc < 0)
      return -1;

    float t0 = (-b - sqrt(disc)) / (2 * a);
    float t1 = (-b + sqrt(disc)) / (2 * a);

    // return smallest non-negative 
    float ta = min(t0, t1);
    float tb = max(t0, t1);
    return ta < 0 ? tb : ta;
  }

  //------------------------------------------------------------------------------
  uint32_t ColorToU32(float r, float g, float b, float a)
  {
    return 
      ((uint32_t)(255 * Clamp(0.0f, 1.0f, r)) << 0) +
      ((uint32_t)(255 * Clamp(0.0f, 1.0f, g)) << 8) +
      ((uint32_t)(255 * Clamp(0.0f, 1.0f, b)) << 16) +
      ((uint32_t)(255 * Clamp(0.0f, 1.0f, a)) << 24);
  }

    //------------------------------------------------------------------------------
  // From http://gamedevs.org/uploads/fast-extraction-viewing-frustum-planes-from-world-view-projection-matrix.pdf
  void ExtractPlanes(const Matrix& mtx, bool normalize, Plane* planes)
  {
    // Left clipping plane
    planes[0].x = mtx._14 + mtx._11;
    planes[0].y = mtx._24 + mtx._21;
    planes[0].z = mtx._34 + mtx._31;
    planes[0].w = mtx._44 + mtx._41;

    // Right clipping plane
    planes[1].x = mtx._14 - mtx._11;
    planes[1].y = mtx._24 - mtx._21;
    planes[1].z = mtx._34 - mtx._31;
    planes[1].w = mtx._44 - mtx._41;

    // Top clipping plane
    planes[2].x = mtx._14 - mtx._12;
    planes[2].y = mtx._24 - mtx._22;
    planes[2].z = mtx._34 - mtx._32;
    planes[2].w = mtx._44 - mtx._42;

    // Bottom clipping plane
    planes[3].x = mtx._14 + mtx._12;
    planes[3].y = mtx._24 + mtx._22;
    planes[3].z = mtx._34 + mtx._32;
    planes[3].w = mtx._44 + mtx._42;

    // Near clipping plane
    planes[4].x = mtx._13;
    planes[4].y = mtx._23;
    planes[4].z = mtx._33;
    planes[4].w = mtx._43;

    // Far clipping plane
    planes[5].x = mtx._14 - mtx._13;
    planes[5].y = mtx._24 - mtx._23;
    planes[5].z = mtx._34 - mtx._33;
    planes[5].w = mtx._44 - mtx._43;

    if (normalize)
    {
      for (int i = 0; i < 6; ++i)
        planes[i].Normalize();
    }
  }
}


