#pragma once
#include "graphics.hpp"
#include "object_handle.hpp"

namespace tano
{
  class GraphicsContext;

  // Note, the objects here just hold state. The code for actually setting the state
  // on the context is found in GraphicsContext (SetGpuState et al)

  //------------------------------------------------------------------------------
  struct GpuObjects
  {
    bool CreateDynamic(
        u32 ibSize, DXGI_FORMAT ibFormat,
        u32 vbSize, u32 vbElemSize);

    bool CreateDynamic(
        u32 ibSize, DXGI_FORMAT ibFormat, const  void* ibData,
        u32 vbSize, u32 vbElemSize, const  void* vbData);

    bool CreateDynamicVb(u32 vbSize, u32 vbElemSize, const  void* vbData = nullptr);
    bool CreateDynamicIb(u32 ibSize, DXGI_FORMAT ibFormat, const  void* ibData = nullptr);

    // TODO: replace the flags with a list of semnatic/format tuples
    bool LoadShadersFromFile(
      const char* filename,
      const char* vsEntry,
      const char* gsEntry,
      const char* psEntry,
      u32 flags = 0);

    D3D11_PRIMITIVE_TOPOLOGY _topology = D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;

    ObjectHandle _vs;
    ObjectHandle _gs;
    ObjectHandle _ps;
    ObjectHandle _layout;

    ObjectHandle _vb;
    ObjectHandle _ib;

    u32 _vbSize;
    u32 _ibSize;
    u32 _vbElemSize;
    DXGI_FORMAT _ibFormat;
  };

  //------------------------------------------------------------------------------
  template <typename T>
  struct ConstantBuffer : public T
  {
    bool Create()
    {
      handle = GRAPHICS.CreateBuffer(D3D11_BIND_CONSTANT_BUFFER, sizeof(T), true);
      return handle.IsValid();
    }

    ObjectHandle handle;
  };

  //------------------------------------------------------------------------------
  struct GpuState
  {
    // Passing 0 uses the default settings
    bool Create(
        const  D3D11_DEPTH_STENCIL_DESC* dssDesc = nullptr,
        const  D3D11_BLEND_DESC* blendDesc = nullptr,
        const  D3D11_RASTERIZER_DESC* rasterizerDesc = nullptr);

    ObjectHandle _depthStencilState;
    ObjectHandle _blendState;
    ObjectHandle _rasterizerState;

    enum Samplers
    {
      Linear,
      LinearWrap,
      LinearBorder,
      Point,
    };

    ObjectHandle _samplers[4];
  };
}
