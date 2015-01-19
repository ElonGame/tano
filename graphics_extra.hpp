#pragma once
#include "object_handle.hpp"

namespace tano
{
  //------------------------------------------------------------------------------
  void VertexFlagsToLayoutDesc(u32 vertexFlags, vector<D3D11_INPUT_ELEMENT_DESC>* desc);
  void SetClientSize(HWND hwnd, int width, int height);
  bool InitConfigDialog(HINSTANCE hInstance);

  //------------------------------------------------------------------------------
  enum class ShaderType
  {
    VertexShader,
    PixelShader,
    GeometryShader,
    ComputeShader,
  };

  //------------------------------------------------------------------------------
  struct BufferFlag
  {
    enum Enum
    {
      CreateMipMaps        = 1 << 0,
      CreateDepthBuffer    = 1 << 1,
      CreateSrv            = 1 << 2,
      CreateUav            = 1 << 3,
    };

    struct Bits
    {
      u32 mipMaps : 1;
      u32 depthBuffer : 1;
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
    Setup() : selectedAdapter(-1), SelectedDisplayMode(-1), multisampleCount(-1), selectedAspectRatio(-1), windowed(false) {}

    CComPtr<IDXGIFactory> dxgi_factory;
    vector<VideoAdapter> videoAdapters;
    int selectedAdapter;
    int SelectedDisplayMode;
    int selectedAspectRatio;
    int multisampleCount;
    bool windowed;
    int width, height;
  };

  //------------------------------------------------------------------------------
  template<class Resource, class Desc>
  struct ResourceAndDesc
  {
    void release() {
      resource.Release();
    }
    CComPtr<Resource> resource;
    Desc desc;
  };

  //------------------------------------------------------------------------------
  struct RenderTargetResource
  {
    RenderTargetResource() : in_use(true) {
      reset();
    }

    void reset() {
      texture.release();
      depth_stencil.release();
      rtv.release();
      dsv.release();
      srv.release();
      uav.release();
    }

    bool in_use;
    ResourceAndDesc<ID3D11Texture2D, D3D11_TEXTURE2D_DESC> texture;
    ResourceAndDesc<ID3D11Texture2D, D3D11_TEXTURE2D_DESC> depth_stencil;
    ResourceAndDesc<ID3D11RenderTargetView, D3D11_RENDER_TARGET_VIEW_DESC> rtv;
    ResourceAndDesc<ID3D11DepthStencilView, D3D11_DEPTH_STENCIL_VIEW_DESC> dsv;
    ResourceAndDesc<ID3D11ShaderResourceView, D3D11_SHADER_RESOURCE_VIEW_DESC> srv;
    ResourceAndDesc<ID3D11UnorderedAccessView, D3D11_UNORDERED_ACCESS_VIEW_DESC> uav;
  };

  //------------------------------------------------------------------------------
  struct TextureResource
  {
    void reset() {
      texture.release();
      view.release();
    }
    ResourceAndDesc<ID3D11Texture2D, D3D11_TEXTURE2D_DESC> texture;
    ResourceAndDesc<ID3D11ShaderResourceView, D3D11_SHADER_RESOURCE_VIEW_DESC> view;
  };

  //------------------------------------------------------------------------------
  struct SimpleResource
  {
    void reset() {
      resource.Release();
      view.release();
    }
    CComPtr<ID3D11Resource> resource;
    ResourceAndDesc<ID3D11ShaderResourceView, D3D11_SHADER_RESOURCE_VIEW_DESC> view;
  };

  //------------------------------------------------------------------------------
  struct StructuredBuffer
  {
    ResourceAndDesc<ID3D11Buffer, D3D11_BUFFER_DESC> buffer;
    ResourceAndDesc<ID3D11ShaderResourceView, D3D11_SHADER_RESOURCE_VIEW_DESC> srv;
    ResourceAndDesc<ID3D11UnorderedAccessView, D3D11_UNORDERED_ACCESS_VIEW_DESC> uav;
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
    CD3D11_VIEWPORT _viewport;
  };

  //------------------------------------------------------------------------------
  struct ScopedRenderTarget
  {
    ScopedRenderTarget(int width, int height, DXGI_FORMAT format, const BufferFlags& bufferFlags);
    ~ScopedRenderTarget();

    ObjectHandle h;
  };

}