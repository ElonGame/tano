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

    bool CreateVertexBuffer(u32 vbSize, u32 vbElemSize, const  void* vbData);
    bool CreateIndexBuffer(u32 ibSize, DXGI_FORMAT ibFormat, const  void* ibData);

    bool LoadShadersFromFile(
      const char* filename,
      const char* vsEntry,
      const char* gsEntry,
      const char* psEntry,
      u32 flags = 0,
      vector<D3D11_INPUT_ELEMENT_DESC>* elements = nullptr);

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
    u32 _numVerts = 0;
    u32 _numIndices = 0;
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
      Point,
      Linear,
      LinearWrap,
      LinearBorder,
    };

    ObjectHandle _samplers[4];
  };

  //------------------------------------------------------------------------------
  // Bundles Gpu state & objects in a cute little package :)
  struct BundleOptions;
  struct GpuBundle
  {
    bool Create(const BundleOptions& options);
    GpuState state;
    GpuObjects objects;
  };

  struct BundleOptions
  {
    struct OptionFlag {
      enum Enum {
        DepthStencilDesc  = 1 << 0,
        BlendDesc         = 1 << 1,
        RasterizerDesc    = 1 << 2,
        DynamicVb         = 1 << 3,
        DynamicIb         = 1 << 3,
      };

      struct Bits {
        u32 depthStencilDesc : 1;
        u32 blendDesc : 1;
        u32 rasterizerDesc : 1;
        u32 dynamicVb : 1;
        u32 dynamicIb : 1;
      };
    };

    typedef Flags<OptionFlag> OptionFlags;

    BundleOptions& DepthStencilDesc(const D3D11_DEPTH_STENCIL_DESC& desc);
    BundleOptions& BlendDesc(const D3D11_BLEND_DESC& desc);
    BundleOptions& RasterizerDesc(const D3D11_RASTERIZER_DESC& desc);

    BundleOptions& ShaderFile(const char* filename);
    BundleOptions& VsEntry(const char* entrypoint);
    BundleOptions& GsEntry(const char* entrypoint);
    BundleOptions& PsEntry(const char* entrypoint);
    BundleOptions& CsEntry(const char* entrypoint);

    BundleOptions& VertexFlags(u32 flags);
    BundleOptions& InputElements(const vector<D3D11_INPUT_ELEMENT_DESC>& elems);

    BundleOptions& DynamicVb(int numElements, int elementSize);
    BundleOptions& DynamicIb(int numElements, int elementSize);

    D3D11_DEPTH_STENCIL_DESC depthStencilDesc;
    D3D11_BLEND_DESC blendDesc;
    D3D11_RASTERIZER_DESC rasterizerDesc;

    const char* shaderFile = nullptr;
    const char* vsEntry = nullptr;
    const char* gsEntry = nullptr;
    const char* psEntry = nullptr;
    const char* csEntry = nullptr;

    u32 vertexFlags = 0;
    vector<D3D11_INPUT_ELEMENT_DESC> inputElements;

    int vbNumElems = 0;
    int vbElemSize = 0;

    int ibNumElems = 0;
    int ibElemSize = 0;

    OptionFlags flags;
  };
}
