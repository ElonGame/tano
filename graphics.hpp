/*
  repeat after me: 
    directx is left-handed.
    z goes into the screen.
    clockwise winding order.
*/

#pragma once
#include "object_handle.hpp"
#include "append_buffer.hpp"
#include "graphics_extra.hpp"

namespace tano
{

  class PostProcess;

  class Graphics
  {
    friend class GraphicsContext;
    friend class PackedResourceManager;
    friend class ResourceManager;
    friend struct SwapChain;
    friend bool EnumerateDisplayModes(HWND hWnd);
    friend INT_PTR CALLBACK dialogWndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
  public:

    static bool Create(HINSTANCE hInstance);
    static bool Destroy();
    static Graphics& Instance();

    ObjectHandle LoadTexture(const char* filename, bool srgb = false, D3DX11_IMAGE_INFO* info = nullptr);
    ObjectHandle LoadTextureFromMemory(const void* buf, u32 len, bool srgb = false, D3DX11_IMAGE_INFO* info = nullptr);

    ObjectHandle CreateInputLayout(const vector<D3D11_INPUT_ELEMENT_DESC> &desc, const vector<char> &shaderBytecode);

    ObjectHandle CreateBuffer(D3D11_BIND_FLAG bind, int size, bool dynamic, const void* buf = nullptr, int userData = 0);

    ObjectHandle CreateVertexShader(const vector<char> &shaderBytecode);
    ObjectHandle CreatePixelShader(const vector<char> &shaderBytecode);
    ObjectHandle CreateComputeShader(const vector<char> &shaderBytecode);
    ObjectHandle CreateGeometryShader(const vector<char> &shaderBytecode);

    ObjectHandle CreateRasterizerState(const D3D11_RASTERIZER_DESC &desc);
    ObjectHandle CreateBlendState(const D3D11_BLEND_DESC &desc);
    ObjectHandle CreateDepthStencilState(const D3D11_DEPTH_STENCIL_DESC &desc);
    ObjectHandle CreateSamplerState(const D3D11_SAMPLER_DESC &desc);
    ObjectHandle CreateSwapChain(const TCHAR* name, u32 width, u32 height, DXGI_FORMAT format, WNDPROC wndProc, HINSTANCE instance);

    D3D_FEATURE_LEVEL FeatureLevel() const { return _featureLevel; }

    bool GetTextureSize(ObjectHandle h, u32* x, u32* y);

    ObjectHandle GetTempRenderTarget(int width, int height, DXGI_FORMAT format, const BufferFlags& bufferFlags);
    ObjectHandle GetTempDepthStencil(int width, int height, const BufferFlags& bufferFlags);
    void ReleaseTempRenderTarget(ObjectHandle h);
    void ReleaseTempDepthStencil(ObjectHandle h);

    ObjectHandle CreateRenderTarget(int width, int height, DXGI_FORMAT format, const BufferFlags& bufferFlags);
    ObjectHandle CreateDepthStencil(int width, int height, const BufferFlags& bufferFlags);

    ObjectHandle CreateStructuredBuffer(int elemSize, int numElems, bool createSrv);
    ObjectHandle CreateTexture(const D3D11_TEXTURE2D_DESC &desc);
    ObjectHandle GetTexture(const char *filename);

    bool ReadTexture(const char *filename, D3DX11_IMAGE_INFO *info, u32 *pitch, vector<u8> *bits);

    // Create a texture, and fill it with data
    ObjectHandle CreateTexture(int width, int height, DXGI_FORMAT fmt, void *data, int data_width, int data_height, int data_pitch);
    ObjectHandle CreateTexture(int width, int height, DXGI_FORMAT fmt, void *data, int pitch);

    SwapChain* GetSwapChain(ObjectHandle h);

    GraphicsContext *GetGraphicsContext();
    PostProcess* GetPostProcess();

    bool GetVSync() const { return _vsync; }
    void SetVSync(bool value) { _vsync = value; }

    const Setup& CurSetup() const { return _curSetup; }
    void SetDisplayAllModes(bool value) { _displayAllModes = value; }
    bool DisplayAllModes() const { return _displayAllModes; }
    const DXGI_MODE_DESC& SelectedDisplayMode() const;

    ID3D11Device* Device() { return _device.p; }

    void CreateDefaultSwapChain(u32 width, u32 height, DXGI_FORMAT format, WNDPROC wndProc, HINSTANCE instance);

    void ClearRenderTarget(ObjectHandle h);
    void Present();
    
    void GetBackBufferSize(int* width, int* height);
    ObjectHandle GetBackBuffer();
    ObjectHandle GetDepthStencil();
    ObjectHandle DefaultSwapChain();

    bool LoadShadersFromFile(
        const string& filenameBase,
        ObjectHandle* vs,
        ObjectHandle* gs,
        ObjectHandle* ps,
        ObjectHandle* inputLayout,
        vector<D3D11_INPUT_ELEMENT_DESC>* elements,
        const char* vsEntry = "VsMain",
        const char* gsEntry = "GsMain",
        const char* psEntry = "PsMain");

    ObjectHandle LoadVertexShaderFromFile(
      const string& filenameBase,
      const char* entry,
      ObjectHandle* inputLayout = nullptr,
      vector<D3D11_INPUT_ELEMENT_DESC>* elements = nullptr);
    ObjectHandle LoadPixelShaderFromFile(const string& filenameBase, const char* entry);
    ObjectHandle LoadComputeShaderFromFile(const string& filenameBase, const char* entry);
    ObjectHandle LoadGeometryShaderFromFile(const string& filenameBase, const char* entry);

    static ObjectHandle MakeObjectHandle(ObjectHandle::Type type, int idx, int data = 0);

  private:
    ~Graphics();

    ObjectHandle ReserveObjectHandle(ObjectHandle::Type type);

    bool CreateDevice();

    bool Init(HINSTANCE hInstance);

    bool CreateBufferInner(D3D11_BIND_FLAG bind, int size, bool dynamic, const void* data, ID3D11Buffer** buffer);

    RenderTargetResource* CreateRenderTargetPtr(int width, int height, DXGI_FORMAT format, const BufferFlags& bufferFlags);
    DepthStencilResource* CreateDepthStencilPtr(int width, int height, const BufferFlags& bufferFlags);

    TextureResource* CreateTexturePtr(const D3D11_TEXTURE2D_DESC &desc);
    TextureResource* CreateTexturePtr(int width, int height, DXGI_FORMAT fmt, void *data, int data_width, int data_height, int data_pitch);

    ID3D11ShaderResourceView* GetShaderResourceView(ObjectHandle h);

    ObjectHandle InsertTexture(TextureResource* data);

    Setup _curSetup;

    CComPtr<ID3D11Device> _device;
    CComPtr<ID3D11DeviceContext> _immediateContext;
    GraphicsContext* _graphicsContext = nullptr;
    PostProcess* _postProcess = nullptr;

#if WITH_DXGI_DEBUG
    CComPtr<ID3D11Debug> _d3dDebug;
#endif

    // resources
    enum { IdCount = 1 << ObjectHandle::cIdBits };
    AppendBuffer<ID3D11VertexShader*, IdCount, ReleaseMixin> _vertexShaders;
    AppendBuffer<ID3D11PixelShader*, IdCount, ReleaseMixin> _pixelShaders;
    AppendBuffer<ID3D11ComputeShader*, IdCount, ReleaseMixin> _computeShaders;
    AppendBuffer<ID3D11GeometryShader*, IdCount, ReleaseMixin> _geometryShaders;
    AppendBuffer<ID3D11InputLayout*, IdCount, ReleaseMixin> _inputLayouts;
    AppendBuffer<ID3D11Buffer*, IdCount, ReleaseMixin> _vertexBuffers;
    AppendBuffer<ID3D11Buffer*, IdCount, ReleaseMixin> _indexBuffers;
    AppendBuffer<ID3D11Buffer*, IdCount, ReleaseMixin> _constantBuffers;

    AppendBuffer<ID3D11BlendState*, IdCount, ReleaseMixin> _blendStates;
    AppendBuffer<ID3D11DepthStencilState*, IdCount, ReleaseMixin> _depthStencilStates;
    AppendBuffer<ID3D11RasterizerState*, IdCount, ReleaseMixin> _rasterizerStates;
    AppendBuffer<ID3D11SamplerState*, IdCount, ReleaseMixin> _samplerStates;
    AppendBuffer<ID3D11ShaderResourceView*, IdCount, ReleaseMixin> _shaderResourceViews;

    AppendBuffer<TextureResource*, IdCount, DeleteMixin> _textures;
    AppendBuffer<RenderTargetResource*, IdCount, DeleteMixin> _renderTargets;
    AppendBuffer<DepthStencilResource*, IdCount, DeleteMixin> _depthStencils;
    AppendBuffer<SimpleResource*, IdCount, DeleteMixin> _resources;
    AppendBuffer<StructuredBuffer*, IdCount, DeleteMixin> _structuredBuffers;
    AppendBuffer<SwapChain*, 16, DeleteMixin> _swapChains;

    static Graphics* _instance;
    static IDXGIDebug* _debugInterface;
    static HMODULE _debugModule;

    D3D_FEATURE_LEVEL _featureLevel;

    ObjectHandle _defaultRenderTarget;

    bool _vsync = true;
    int _totalBytesAllocated = 0;

    ObjectHandle _defaultSwapChainHandle;
    SwapChain* _defaultSwapChain = nullptr;

    bool _displayAllModes = false;

    struct TempRenderTarget
    {
      D3D11_TEXTURE2D_DESC desc;
      BufferFlags flags;
      u32 idx;
      bool inUse;
    };

    struct TempDepthStencil
    {
      BufferFlags flags;
      u32 idx;
      bool inUse;
    };

    SimpleAppendBuffer<TempRenderTarget, 64> _tempRenderTargets;
    SimpleAppendBuffer<TempDepthStencil, 64> _tempDepthStencils;
  };

#define GRAPHICS Graphics::Instance()
#define GFX_CreateBuffer(bind, size, dynamic, buf, data) GRAPHICS.CreateBuffer(bind, size, dynamic, buf, data);

}
