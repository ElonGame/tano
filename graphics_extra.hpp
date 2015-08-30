#pragma once
#include "object_handle.hpp"

namespace tano
{
  class GraphicsContext;

  //------------------------------------------------------------------------------
  void VertexFlagsToLayoutDesc(u32 vertexFlags, vector<D3D11_INPUT_ELEMENT_DESC>* desc);
  void SetClientSize(HWND hwnd, int width, int height);
  bool InitConfigDialog(HINSTANCE hInstance);

  //------------------------------------------------------------------------------
  enum ShaderType
  {
    VertexShader   = 1 << 0,
    PixelShader    = 1 << 1,
    GeometryShader = 1 << 2,
    ComputeShader  = 1 << 3,
  };

  //------------------------------------------------------------------------------
  struct BufferFlag
  {
    enum Enum
    {
      CreateMipMaps = 1 << 0,
      CreateSrv     = 1 << 1,
      CreateUav     = 1 << 2,
    };

    struct Bits
    {
      u32 mipMaps : 1;
      u32 srv : 1;
      u32 uav : 1;
    };
  };

  typedef Flags<BufferFlag> BufferFlags;

  //------------------------------------------------------------------------------
  struct VideoAdapter
  {
    CComPtr<IDXGIAdapter> adapter;
    DXGI_ADAPTER_DESC desc;
    vector<DXGI_MODE_DESC> displayModes;
  };

  //------------------------------------------------------------------------------
  struct Setup
  {
    CComPtr<IDXGIFactory1> dxgi_factory;
    vector<VideoAdapter> videoAdapters;
    int selectedAdapter     = -1;
    int SelectedDisplayMode = -1;
    int selectedAspectRatio = -1;
    int multisampleCount    = -1;
    int width               = -1;
    int height              = -1;
    bool windowed           = false;
  };

  //------------------------------------------------------------------------------
  template <class Resource, class Desc>
  struct PointerAndDesc
  {
    void Release()
    {
      if (ptr)
        ptr->Release();
    }

    Resource* ptr = nullptr;
    Desc desc;
  };

  //------------------------------------------------------------------------------
  struct DepthStencilResource
  {
    DepthStencilResource() { Reset(); }

    void Reset()
    {
      texture.Release();
      view.Release();
      srv.Release();
    }

    PointerAndDesc<ID3D11Texture2D, D3D11_TEXTURE2D_DESC> texture;
    PointerAndDesc<ID3D11DepthStencilView, D3D11_DEPTH_STENCIL_VIEW_DESC> view;
    PointerAndDesc<ID3D11ShaderResourceView, D3D11_SHADER_RESOURCE_VIEW_DESC> srv;
  };

  //------------------------------------------------------------------------------
  struct RenderTargetResource
  {
    RenderTargetResource() { Reset(); }

    void Reset()
    {
      texture.Release();
      view.Release();
      srv.Release();
      uav.Release();
    }

    PointerAndDesc<ID3D11Texture2D, D3D11_TEXTURE2D_DESC> texture;
    PointerAndDesc<ID3D11RenderTargetView, D3D11_RENDER_TARGET_VIEW_DESC> view;
    PointerAndDesc<ID3D11ShaderResourceView, D3D11_SHADER_RESOURCE_VIEW_DESC> srv;
    PointerAndDesc<ID3D11UnorderedAccessView, D3D11_UNORDERED_ACCESS_VIEW_DESC> uav;
  };

  //------------------------------------------------------------------------------
  struct TextureResource
  {
    void Reset()
    {
      texture.Release();
      view.Release();
    }
    PointerAndDesc<ID3D11Texture2D, D3D11_TEXTURE2D_DESC> texture;
    PointerAndDesc<ID3D11ShaderResourceView, D3D11_SHADER_RESOURCE_VIEW_DESC> view;
  };

  //------------------------------------------------------------------------------
  struct SimpleResource
  {
    void Reset()
    {
      if (resource)
        resource->Release();
      view.Release();
    }
    ID3D11Resource* resource = 0;
    PointerAndDesc<ID3D11ShaderResourceView, D3D11_SHADER_RESOURCE_VIEW_DESC> view;
  };

  //------------------------------------------------------------------------------
  struct StructuredBuffer
  {
    PointerAndDesc<ID3D11Buffer, D3D11_BUFFER_DESC> buffer;
    PointerAndDesc<ID3D11ShaderResourceView, D3D11_SHADER_RESOURCE_VIEW_DESC> srv;
    PointerAndDesc<ID3D11UnorderedAccessView, D3D11_UNORDERED_ACCESS_VIEW_DESC> uav;
  };

  //------------------------------------------------------------------------------
  struct SwapChain
  {
    SwapChain(const char* name) : _name(name) {}
    bool CreateBackBuffers(u32 width, u32 height, DXGI_FORMAT format);
    void Present();

    string _name;
    HWND _hwnd;
    CComPtr<IDXGISwapChain> _swapChain;
    DXGI_SWAP_CHAIN_DESC _desc;
    ObjectHandle _renderTarget;
    ObjectHandle _depthStencil;
    CD3D11_VIEWPORT _viewport;
    u32 _width, _height;
  };

  //------------------------------------------------------------------------------
  struct RenderTargetDesc
  {
    RenderTargetDesc() {}
    RenderTargetDesc(int width, int height, DXGI_FORMAT format) : width(width), height(height), format(format)
    {
    }
    int width;
    int height;
    DXGI_FORMAT format;
  };

  //------------------------------------------------------------------------------
  struct ScopedRenderTarget
  {
    ScopedRenderTarget() {}
    ScopedRenderTarget(int width, int height, DXGI_FORMAT format,
        const BufferFlags& bufferFlags = BufferFlags(BufferFlag::CreateSrv));
    ScopedRenderTarget(
        const RenderTargetDesc& desc, const BufferFlags& bufferFlags = BufferFlags(BufferFlag::CreateSrv));
    ScopedRenderTarget(
        DXGI_FORMAT format, const BufferFlags& bufferFlags = BufferFlags(BufferFlag::CreateSrv));
    ~ScopedRenderTarget();

    operator ObjectHandle() { return _rtHandle; }

    RenderTargetDesc _desc;
    ObjectHandle _rtHandle;
  };

  //------------------------------------------------------------------------------
  struct ScopedRenderTargetFull
  {
    ScopedRenderTargetFull() {}
    ScopedRenderTargetFull(DXGI_FORMAT format, BufferFlags rtFlags, BufferFlags dsFlags);
    ~ScopedRenderTargetFull();

    operator ObjectHandle() { return _rtHandle; }

    RenderTargetDesc _desc;

    ObjectHandle _rtHandle;
    ObjectHandle _dsHandle;
  };

  //------------------------------------------------------------------------------
  void InitDefaultDescs();
  extern CD3D11_BLEND_DESC blendDescBlendSrcAlpha;
  extern CD3D11_BLEND_DESC blendDescBlendOneOne;
  extern CD3D11_BLEND_DESC blendDescPreMultipliedAlpha;

  extern CD3D11_BLEND_DESC blendDescWeightedBlend;
  extern CD3D11_BLEND_DESC blendDescWeightedBlendResolve;

  extern CD3D11_RASTERIZER_DESC rasterizeDescCullNone;
  extern CD3D11_RASTERIZER_DESC rasterizeDescWireframe;

  extern CD3D11_DEPTH_STENCIL_DESC depthDescDepthDisabled;
  extern CD3D11_DEPTH_STENCIL_DESC depthDescDepthWriteDisabled;
}