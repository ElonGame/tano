/*
  repeat after me: 
    directx is left-handed.
    z goes into the screen.
    clockwise winding order.
*/

#pragma once
#include "object_handle.hpp"
#include "id_buffer.hpp"
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

    Graphics();

    static bool Create(HINSTANCE hInstance);
    static bool Destroy();
    static Graphics& Instance();

    ObjectHandle LoadTexture(
        const char* filename,
        const char* friendlyName = nullptr,
        bool srgb = false,
        D3DX11_IMAGE_INFO* info = nullptr);

    ObjectHandle LoadTextureFromMemory(
        const void* buf,
        u32 len,
        const char* friendlyName = nullptr,
        bool srgb = false,
        D3DX11_IMAGE_INFO* info = nullptr);

    ObjectHandle CreateInputLayout(
        const vector<D3D11_INPUT_ELEMENT_DESC> &desc,
        const vector<char> &shader_bytecode);

    ObjectHandle CreateBuffer(
        D3D11_BIND_FLAG bind,
        int size,
        bool dynamic,
        const void* buf = nullptr,
        int userData = 0);

    ObjectHandle CreateVertexShader(const vector<char> &shader_bytecode, const string& id);
    ObjectHandle CreatePixelShader(const vector<char> &shader_bytecode, const string& id);
    ObjectHandle CreateComputeShader(const vector<char> &shader_bytecode, const string &id);
    ObjectHandle CreateGeometryShader(const vector<char> &shader_bytecode, const string &id);

    ObjectHandle CreateRasterizerState(const D3D11_RASTERIZER_DESC &desc, const char *name = nullptr);
    ObjectHandle CreateBlendState(const D3D11_BLEND_DESC &desc, const char *name = nullptr);
    ObjectHandle CreateDepthStencilState(const D3D11_DEPTH_STENCIL_DESC &desc, const char *name = nullptr);
    ObjectHandle CreateSamplerState(const D3D11_SAMPLER_DESC &desc, const char *name = nullptr);
    ObjectHandle CreateSwapChain(const TCHAR* name, u32 width, u32 height, DXGI_FORMAT format, WNDPROC wndProc, HINSTANCE instance);
    ObjectHandle RenderTargetForSwapChain(ObjectHandle h);

    D3D_FEATURE_LEVEL FeatureLevel() const { return _featureLevel; }

    bool GetTextureSize(ObjectHandle h, u32* x, u32* y);
    ObjectHandle GetTempRenderTarget(int width, int height, DXGI_FORMAT format, const BufferFlags& bufferFlags);
    void ReleaseTempRenderTarget(ObjectHandle h);

    ObjectHandle CreateRenderTarget(int width, int height, DXGI_FORMAT format, const BufferFlags& bufferFlags, const string& name = "");

    ObjectHandle CreateStructuredBuffer(int elemSize, int numElems, bool createSrv);
    ObjectHandle CreateTexture(const D3D11_TEXTURE2D_DESC &desc, const char *name);
    ObjectHandle GetTexture(const char *filename);

    bool ReadTexture(const char *filename, D3DX11_IMAGE_INFO *info, u32 *pitch, vector<u8> *bits);

    // Create a texture, and fill it with data
    bool CreateTexture(int width, int height, DXGI_FORMAT fmt, void *data, int data_width, int data_height, int data_pitch, TextureResource *out);
    ObjectHandle CreateTexture(int width, int height, DXGI_FORMAT fmt, void *data, int data_width, int data_height, int data_pitch, const char *friendlyName);
    ObjectHandle CreateTexture(int width, int height, DXGI_FORMAT fmt, void *data, int pitch);

    SwapChain* GetSwapChain(ObjectHandle h);

    ObjectHandle FindResource(const string &name);
    ObjectHandle FindSampler(const string &name);
    ObjectHandle FindBlendState(const string &name);
    ObjectHandle FindRasterizerState(const string &name);
    ObjectHandle FindDepthStencilState(const string &name);

    GraphicsContext *GetGraphicsContext();
    PostProcess* GetPostProcess();

    bool GetVSync() const { return _vsync; }
    void SetVSync(bool value) { _vsync = value; }

    void GetRenderTargetTextureDesc(ObjectHandle handle, D3D11_TEXTURE2D_DESC* desc);

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

    bool LoadComputeShadersFromFile(
        const string& filenameBase,
        ObjectHandle* shader,
        const char* entry = "CsMain");

    static ObjectHandle MakeObjectHandle(ObjectHandle::Type type, int idx, int data = 0);

  private:
    ~Graphics();

    ObjectHandle ReserveObjectHandle(const string& id, ObjectHandle::Type type);

    bool CreateDevice();

    bool Init(HINSTANCE hInstance);

    bool CreateBufferInner(D3D11_BIND_FLAG bind, int size, bool dynamic, const void* data, ID3D11Buffer** buffer);

    bool CreateRenderTarget(
        int width,
        int height,
        DXGI_FORMAT format,
        const BufferFlags& bufferFlags,
        RenderTargetResource *out);
    bool CreateTexture(const D3D11_TEXTURE2D_DESC &desc, TextureResource *out);

    ID3D11ShaderResourceView* GetShaderResourceView(ObjectHandle h);

    // given texture data and a name, insert it into the GOH chain
    ObjectHandle InsertTexture(TextureResource* data, const char* friendlyName = nullptr);

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
    IdBuffer<ID3D11VertexShader*, IdCount> _vertexShaders;
    IdBuffer<ID3D11PixelShader*, IdCount> _pixelShaders;
    IdBuffer<ID3D11ComputeShader*, IdCount> _computeShaders;
    IdBuffer<ID3D11GeometryShader*, IdCount> _geometryShaders;
    IdBuffer<ID3D11InputLayout*, IdCount> _inputLayouts;
    IdBuffer<ID3D11Buffer*, IdCount> _vertexBuffers;
    IdBuffer<ID3D11Buffer*, IdCount> _indexBuffers;
    IdBuffer<ID3D11Buffer*, IdCount> _constantBuffers;

    IdBuffer<ID3D11BlendState*, IdCount> _blendStates;
    IdBuffer<ID3D11DepthStencilState*, IdCount> _depthStencilStates;
    IdBuffer<ID3D11RasterizerState*, IdCount> _rasterizerStates;
    IdBuffer<ID3D11SamplerState*, IdCount> _sampler_states;

    IdBuffer<TextureResource*, IdCount> _textures;
    IdBuffer<RenderTargetResource*, IdCount> _renderTargets;
    IdBuffer<SimpleResource*, IdCount> _resources;
    IdBuffer<ID3D11ShaderResourceView*, IdCount> _shaderResourceViews;
    IdBuffer<StructuredBuffer*, IdCount> _structuredBuffers;
    IdBuffer<SwapChain*, 16> _swapChains;

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
  };

#define GRAPHICS Graphics::Instance()
#define GFX_CreateBuffer(bind, size, dynamic, buf, data) GRAPHICS.CreateBuffer(bind, size, dynamic, buf, data);

}
