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

    bool LoadVertexShader(const char* filename, const char* entryPoint, u32 flags = 0, vector<D3D11_INPUT_ELEMENT_DESC>* elements = nullptr);
    bool LoadPixelShader(const char* filename, const char* entryPoint);
    bool LoadGeometryShader(const char* filename, const char* entryPoint);

    D3D11_PRIMITIVE_TOPOLOGY _topology = D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;

    ObjectHandle _vs;
    ObjectHandle _gs;
    ObjectHandle _ps;
    ObjectHandle _layout;

    ObjectHandle _vb;
    ObjectHandle _ib;

    u32 _vbSize = 0;
    u32 _ibSize = 0;
    u32 _vbElemSize = 0;
    DXGI_FORMAT _ibFormat = DXGI_FORMAT_UNKNOWN;
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
    BundleOptions();
    struct OptionFlag {
      enum Enum {
        DepthStencilDesc  = 1 << 0,
        BlendDesc         = 1 << 1,
        RasterizerDesc    = 1 << 2,
        DynamicVb         = 1 << 3,
        DynamicIb         = 1 << 4,
        StaticVb          = 1 << 5,
        StaticIb          = 1 << 6,
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

    BundleOptions& DepthStencilDesc(const CD3D11_DEPTH_STENCIL_DESC& desc);
    BundleOptions& BlendDesc(const CD3D11_BLEND_DESC& desc);
    BundleOptions& RasterizerDesc(const CD3D11_RASTERIZER_DESC& desc);

    BundleOptions& VertexShader(const char* filename, const char* entrypoint);
    BundleOptions& GeometryShader(const char* filename, const char* entrypoint);
    BundleOptions& PixelShader(const char* filename, const char* entrypoint);
    BundleOptions& ComputeShader(const char* filename, const char* entrypoint);

    BundleOptions& VertexFlags(u32 flags);
    BundleOptions& InputElements(const vector<D3D11_INPUT_ELEMENT_DESC>& elems);
    BundleOptions& Topology(D3D11_PRIMITIVE_TOPOLOGY topology);

    BundleOptions& DynamicVb(int numElements, int elementSize);
    BundleOptions& DynamicIb(int numElements, int elementSize);
    BundleOptions& StaticVb(int numElements, int elementSize, void* vertices);
    BundleOptions& StaticIb(int numElements, int elementSize, void* indices);

    CD3D11_DEPTH_STENCIL_DESC depthStencilDesc = CD3D11_DEPTH_STENCIL_DESC(CD3D11_DEFAULT());
    CD3D11_BLEND_DESC blendDesc = CD3D11_BLEND_DESC(CD3D11_DEFAULT());
    CD3D11_RASTERIZER_DESC rasterizerDesc = CD3D11_RASTERIZER_DESC(CD3D11_DEFAULT());

    const char* vsShaderFile = nullptr;
    const char* gsShaderFile = nullptr;
    const char* psShaderFile = nullptr;
    const char* csShaderFile = nullptr;
    const char* vsEntry = nullptr;
    const char* gsEntry = nullptr;
    const char* psEntry = nullptr;
    const char* csEntry = nullptr;

    u32 vertexFlags = 0;
    vector<D3D11_INPUT_ELEMENT_DESC> inputElements;

    D3D11_PRIMITIVE_TOPOLOGY topology = D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;

    int vbNumElems = 0;
    int vbElemSize = 0;

    int ibNumElems = 0;
    int ibElemSize = 0;

    void* staticVb = nullptr;
    void* staticIb = nullptr;

    OptionFlags flags;
  };

  //------------------------------------------------------------------------------
  template<
    typename Vs0 = void, typename Ps0 = void, typename Gs0 = void,
    typename Vs1 = void, typename Ps1 = void, typename Gs1 = void
  >
  struct ConstantBufferBundle
  {
    struct DummyCb
    {
      bool Create() { return true; }
    };

    typedef typename conditional<is_void<Vs0>::value, DummyCb, ConstantBuffer<Vs0>>::type VsCBuffer0;
    typedef typename conditional<is_void<Vs1>::value, DummyCb, ConstantBuffer<Vs1>>::type VsCBuffer1;
    typedef typename conditional<is_void<Ps0>::value, DummyCb, ConstantBuffer<Ps0>>::type PsCBuffer0;
    typedef typename conditional<is_void<Ps1>::value, DummyCb, ConstantBuffer<Ps1>>::type PsCBuffer1;
    typedef typename conditional<is_void<Gs0>::value, DummyCb, ConstantBuffer<Gs0>>::type GsCBuffer0;
    typedef typename conditional<is_void<Gs1>::value, DummyCb, ConstantBuffer<Gs1>>::type GsCBuffer1;

    VsCBuffer0 vs0;
    VsCBuffer1 vs1;
    PsCBuffer0 ps0;
    PsCBuffer1 ps1;
    GsCBuffer0 gs0;
    GsCBuffer1 gs1;

    bool Create()
    {
      return 
        vs0.Create() && ps0.Create() && gs0.Create() && vs1.Create() && ps1.Create() && gs1.Create();
    }

    void Set(GraphicsContext* ctx, const DummyCb& cb, u32 type, u32 slot)
    {
    }

    template<typename T>
    void Set(GraphicsContext* ctx, ConstantBuffer<T>& cb, u32 type, u32 slot)
    {
      ctx->SetConstantBuffer(cb, type, slot);
    }

    void Set(GraphicsContext* ctx, u32 slot)
    {
      switch (slot)
      {
      case 0:
        Set(ctx, vs0, VertexShader, 0);
        Set(ctx, ps0, PixelShader, 0);
        Set(ctx, gs0, GeometryShader, 0);
        break;

      case 1:
        Set(ctx, vs1, VertexShader, 1);
        Set(ctx, ps1, PixelShader, 1);
        Set(ctx, gs1, GeometryShader, 1);
        break;
      }
    }
  };
}
