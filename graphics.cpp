#include "graphics.hpp"
#include "graphics_context.hpp"
#include "resource_manager.hpp"
#include "init_sequence.hpp"
#include "post_process.hpp"

extern const TCHAR* g_AppWindowTitle;

using namespace tano;
using namespace bristol;

//------------------------------------------------------------------------------
static ObjectHandle emptyHandle;
Graphics* Graphics::_instance;

#if WITH_DXGI_DEBUG
IDXGIDebug* Graphics::_debugInterface;
HMODULE Graphics::_debugModule;
#endif

//------------------------------------------------------------------------------
Graphics& Graphics::Instance()
{
  return *_instance;
}

//------------------------------------------------------------------------------
Graphics::Graphics()
  : _vertexShaders(ReleaseObj<ID3D11VertexShader *>)
  , _pixelShaders(ReleaseObj<ID3D11PixelShader *>)
  , _computeShaders(ReleaseObj<ID3D11ComputeShader *>)
  , _geometryShaders(ReleaseObj<ID3D11GeometryShader *>)
  , _vertexBuffers(ReleaseObj<ID3D11Buffer *>)
  , _indexBuffers(ReleaseObj<ID3D11Buffer *>)
  , _constantBuffers(ReleaseObj<ID3D11Buffer *>)
  , _inputLayouts(ReleaseObj<ID3D11InputLayout *>)
  , _blendStates(ReleaseObj<ID3D11BlendState *>)
  , _depthStencilStates(ReleaseObj<ID3D11DepthStencilState *>)
  , _rasterizerStates(ReleaseObj<ID3D11RasterizerState *>)
  , _samplerStates(ReleaseObj<ID3D11SamplerState *>)
  , _shaderResourceViews(ReleaseObj<ID3D11ShaderResourceView *>)
  , _textures(DeleteObj<TextureResource *>)
  , _renderTargets(DeleteObj<RenderTargetResource *>)
  , _depthStencils(DeleteObj<DepthStencilResource *>)
  , _resources(DeleteObj<SimpleResource *>)
  , _structuredBuffers(DeleteObj<StructuredBuffer *>)
  , _swapChains(DeleteObj<SwapChain*>)
{
}

//------------------------------------------------------------------------------
Graphics::~Graphics()
{
  SAFE_DELETE(_graphicsContext);
  SAFE_DELETE(_postProcess);
}

//------------------------------------------------------------------------------
bool Graphics::Create(HINSTANCE hInstance)
{
  if (_instance)
    return true;

  _instance = new Graphics();

#if WITH_DXGI_DEBUG
  // Get the DXGIGetDebugInterface
  if (_debugModule = LoadLibraryA("Dxgidebug.dll"))
  {
    typedef HRESULT (WINAPI *DXGIGetDebugInterfaceFunc)(REFIID, void**);
    auto fn = (DXGIGetDebugInterfaceFunc)GetProcAddress(_debugModule, "DXGIGetDebugInterface");
    fn(__uuidof(IDXGIDebug), (void**)&_debugInterface);
  }
#endif

  return _instance->Init(hInstance);
}

//------------------------------------------------------------------------------
bool Graphics::Init(HINSTANCE hInstance)
{
  BEGIN_INIT_SEQUENCE();
  INIT(InitConfigDialog(hInstance));
  INIT(CreateDevice());

  _postProcess = new PostProcess(_graphicsContext);
  INIT(_postProcess->Init());

  InitDefaultDescs();

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool Graphics::Destroy()
{
  delete exch_null(_instance);
#if WITH_DXGI_DEBUG
  if (_debugModule)
  {
    OutputDebugStringA("** Dumping live objects\n");
    // todo: figure this out
    //_debugInterface->ReportLiveObjects(DXGI_DEBUG_ALL, DXGI_DEBUG_RLO_ALL);
    FreeLibrary(_debugModule);
  }
#endif

  return true;
}

//------------------------------------------------------------------------------
const DXGI_MODE_DESC &Graphics::SelectedDisplayMode() const
{
  return _curSetup.videoAdapters[_curSetup.selectedAdapter].displayModes[_curSetup.SelectedDisplayMode];
}

//------------------------------------------------------------------------------
bool Graphics::CreateDevice()
{
  int flags = 0;
#if WITH_DXGI_DEBUG
  flags |= D3D11_CREATE_DEVICE_DEBUG;
#endif

  flags |= D3D11_CREATE_DEVICE_BGRA_SUPPORT;

  // Create the DX11 device and context
  CComPtr<IDXGIAdapter> adapter = _curSetup.videoAdapters[_curSetup.selectedAdapter].adapter;
  if (FAILED(D3D11CreateDevice(adapter,D3D_DRIVER_TYPE_UNKNOWN, NULL, flags, NULL, 0, D3D11_SDK_VERSION, &_device, &_featureLevel, &_immediateContext)))
    return false;

  _graphicsContext = new GraphicsContext(_immediateContext);

  if (_featureLevel < D3D_FEATURE_LEVEL_9_3)
    return false;

#if WITH_DXGI_DEBUG
  if (FAILED(_device->QueryInterface(IID_ID3D11Debug, (void **)&(_d3dDebug.p))))
    return false;
#endif

#if WITH_REMOTERY
  rmt_BindD3D11(_device, _immediateContext);
#endif

  return true;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::CreateBuffer(
    D3D11_BIND_FLAG bind,
    int size,
    bool dynamic,
    const void* buf,
    int userData)
{
  ID3D11Buffer* buffer = 0;
  if (CreateBufferInner(bind, size, dynamic, buf, &buffer))
  {
    if (bind == D3D11_BIND_INDEX_BUFFER)
    {
      const int idx = _indexBuffers.Insert(buffer);
      assert(userData == DXGI_FORMAT_R16_UINT || userData == DXGI_FORMAT_R32_UINT);
      return MakeObjectHandle(ObjectHandle::kIndexBuffer, idx, userData);
    } 
    else if (bind == D3D11_BIND_VERTEX_BUFFER)
    {
      const int idx = _vertexBuffers.Insert(buffer);
      // userdata is vertex size
      assert(userData > 0);
      return MakeObjectHandle(ObjectHandle::kVertexBuffer, idx, userData);
    } 
    else if (bind == D3D11_BIND_CONSTANT_BUFFER)
    {
      const int idx = _constantBuffers.Insert(buffer);
      return MakeObjectHandle(ObjectHandle::kConstantBuffer, idx, size);
    }
    else
    {
      //LOG_ERROR_LN("Implement me!");
    }
  }
  return emptyHandle;
}

//------------------------------------------------------------------------------
bool Graphics::CreateBufferInner(
    D3D11_BIND_FLAG bind,
    int size,
    bool dynamic,
    const void* data,
    ID3D11Buffer** buffer)
{
  CD3D11_BUFFER_DESC desc(((size + 15) & ~0xf), bind, 
    dynamic ? D3D11_USAGE_DYNAMIC : D3D11_USAGE_DEFAULT, 
    dynamic ? D3D11_CPU_ACCESS_WRITE : 0);

  HRESULT hr;
  if (data)
  {
    D3D11_SUBRESOURCE_DATA init_data;
    ZeroMemory(&init_data, sizeof(init_data));
    init_data.pSysMem = data;
    hr = _device->CreateBuffer(&desc, &init_data, buffer);
  }
  else
  {
    hr = _device->CreateBuffer(&desc, nullptr, buffer);
  }

  if (SUCCEEDED(hr))
  {
    _totalBytesAllocated += size;
    return true;
  }

  return false;
}

//------------------------------------------------------------------------------
ID3D11ShaderResourceView* Graphics::GetShaderResourceView(ObjectHandle h)
{
  ObjectHandle::Type type = h.type();

  if (type == ObjectHandle::kTexture)
  {
    return _textures.Get(h)->view.ptr;
  }
  else if (type == ObjectHandle::kResource)
  {
    return _resources.Get(h)->view.ptr;
  }
  else if (type == ObjectHandle::kRenderTarget)
  {
    return _renderTargets.Get(h)->srv.ptr;
  }
  else if (type == ObjectHandle::kStructuredBuffer)
  {
    return _structuredBuffers.Get(h)->srv.ptr;
  }
  return nullptr;
}

//------------------------------------------------------------------------------
bool Graphics::GetTextureSize(ObjectHandle h, u32* x, u32* y)
{
  ObjectHandle::Type type = h.type();

  if (type == ObjectHandle::kTexture)
  {
    *x = _textures.Get(h)->texture.desc.Width;
    *y = _textures.Get(h)->texture.desc.Height;
    return true;
  }
  else if (type == ObjectHandle::kResource)
  {
    u32 mipLevel = _resources.Get(h)->view.desc.Texture2D.MostDetailedMip;

    CComPtr<ID3D11Texture2D> texture;
    D3D11_TEXTURE2D_DESC desc;
    texture.Attach((ID3D11Texture2D*)_resources.Get(h)->view.ptr.p);
    texture->GetDesc(&desc);

    *x = max(desc.Width / (1 << mipLevel), 1u);
    *y = max(desc.Height / (1 << mipLevel), 1u);
    return true;
  }
  else if (type == ObjectHandle::kRenderTarget)
  {
    const D3D11_RENDER_TARGET_VIEW_DESC& rtDesc = _renderTargets.Get(h)->view.desc;
    const D3D11_TEXTURE2D_DESC& desc = _renderTargets.Get(h)->texture.desc;
    u32 mipLevel = rtDesc.Texture2D.MipSlice;
    *x = max(desc.Width / (1 << mipLevel), 1u);
    *y = max(desc.Height / (1 << mipLevel), 1u);
    return true;
  }
  else if (type == ObjectHandle::kStructuredBuffer)
  {
    assert(false);
    return true;
//    return _structuredBuffers.Get(h)->srv.resource;
  }
  else
  {
    LOG_ERROR("Attemping to get texture size for unknown object type");
  }

  return false;
}

//------------------------------------------------------------------------------
void Graphics::GetTempRenderTarget(
  int width,
  int height,
  DXGI_FORMAT format,
  const BufferFlags& bufferFlags,
  ObjectHandle* rtHandle,
  ObjectHandle* dsHandle)
{
  // look for a free render target with the wanted properties
  UINT flags = bufferFlags.IsSet(BufferFlag::CreateMipMaps) ? D3D11_RESOURCE_MISC_GENERATE_MIPS : 0;
  bool requiresDepthBuffer = bufferFlags.IsSet(BufferFlag::CreateDepthBuffer);

  auto rtComp = [=](const RenderTargetResource* res)
  {
    const D3D11_TEXTURE2D_DESC& desc = res->texture.desc;
    return !res->inUse && desc.Width == width && desc.Height == height && desc.Format == format && desc.MiscFlags == flags;
    // TODO: handle depth/stencil
//      && requiresDepthBuffer == (res->depthStencil.resource.p != nullptr);
  };

  int idx = _renderTargets.Find(rtComp);
  if (idx != _renderTargets.INVALID_INDEX)
  {
    RenderTargetResource* rt = _renderTargets.Get(idx);
    rt->inUse = true;
    *rtHandle = MakeObjectHandle(ObjectHandle::kRenderTarget, idx);
  }
  else
  {
    // nothing suitable found, so we create a render target
    CreateRenderTarget(width, height, format, bufferFlags, nullptr, rtHandle, dsHandle);
  }
}

//------------------------------------------------------------------------------
void Graphics::ReleaseTempRenderTarget(ObjectHandle h)
{
  RenderTargetResource* rt = _renderTargets.Get(h);
  assert(rt->inUse);
  rt->inUse = false;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::CreateStructuredBuffer(
    int elemSize,
    int numElems,
    bool createSrv)
{
  unique_ptr<StructuredBuffer> sb(new StructuredBuffer);

  // Create Structured Buffer
  D3D11_BUFFER_DESC sbDesc;
  sbDesc.BindFlags            = D3D11_BIND_UNORDERED_ACCESS | D3D11_BIND_SHADER_RESOURCE;
  sbDesc.CPUAccessFlags       = 0;
  sbDesc.MiscFlags            = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
  sbDesc.StructureByteStride  = elemSize;
  sbDesc.ByteWidth            = elemSize * numElems;
  sbDesc.Usage                = D3D11_USAGE_DEFAULT;
  if (FAILED(_device->CreateBuffer(&sbDesc, NULL, &sb->buffer.ptr)))
    return emptyHandle;

  auto buf = sb->buffer.ptr.p;

  // create the UAV for the structured buffer
  D3D11_UNORDERED_ACCESS_VIEW_DESC sbUAVDesc;
  sbUAVDesc.Buffer.FirstElement       = 0;
  sbUAVDesc.Buffer.Flags              = 0;
  sbUAVDesc.Buffer.NumElements        = numElems;
  sbUAVDesc.Format                    = DXGI_FORMAT_UNKNOWN;
  sbUAVDesc.ViewDimension             = D3D11_UAV_DIMENSION_BUFFER;
  if (FAILED(_device->CreateUnorderedAccessView(buf, &sbUAVDesc, &sb->uav.ptr)))
    return emptyHandle;

  if (createSrv)
  {
    // create the Shader Resource View (SRV) for the structured buffer
    D3D11_SHADER_RESOURCE_VIEW_DESC sbSRVDesc;
    sbSRVDesc.Buffer.ElementOffset          = 0;
    sbSRVDesc.Buffer.ElementWidth           = elemSize;
    sbSRVDesc.Buffer.FirstElement           = 0;
    sbSRVDesc.Buffer.NumElements            = numElems;
    sbSRVDesc.Format                        = DXGI_FORMAT_UNKNOWN;
    sbSRVDesc.ViewDimension                 = D3D11_SRV_DIMENSION_BUFFER;
    if (FAILED(_device->CreateShaderResourceView(buf, &sbSRVDesc, &sb->srv.ptr)))
      return emptyHandle;
  }

  return MakeObjectHandle(
      ObjectHandle::kStructuredBuffer, _structuredBuffers.Insert(sb.release()));
}

//------------------------------------------------------------------------------
bool Graphics::CreateRenderTarget(
    int width,
    int height,
    DXGI_FORMAT format,
    const BufferFlags& bufferFlags,
    const char* name,
    ObjectHandle* rtHandle,
    ObjectHandle* dsHandle)
{
  ObjectHandle goh;

  RenderTargetResource* rt = new RenderTargetResource();
  DepthStencilResource* ds = bufferFlags.IsSet(BufferFlag::CreateDepthBuffer) ? new DepthStencilResource : nullptr;
  if (CreateRenderTarget(width, height, format, bufferFlags, rt, ds))
  {
    int rtIdx = name ? _renderTargets.Insert(name, rt) : _renderTargets.Insert(rt);
    *rtHandle = MakeObjectHandle(ObjectHandle::kRenderTarget, rtIdx);
    if (ds)
    {
      int dsIdx = _depthStencils.Insert(ds);
      *dsHandle = MakeObjectHandle(ObjectHandle::kDepthStencil, dsIdx);
    }
    return true;
  }

  delete rt;
  delete ds;
  return false;
}

//------------------------------------------------------------------------------
bool Graphics::CreateRenderTarget(
    int width,
    int height,
    DXGI_FORMAT format,
    const BufferFlags& bufferFlags,
    RenderTargetResource* rt,
    DepthStencilResource* ds)
{
  rt->Reset();
  rt->inUse = true;

  // create the render target
  int mip_levels = bufferFlags.IsSet(BufferFlag::CreateMipMaps) ? 0 : 1;
  u32 flags = D3D11_BIND_RENDER_TARGET 
    | (bufferFlags.IsSet(BufferFlag::CreateSrv) ? D3D11_BIND_SHADER_RESOURCE : 0)
    | (bufferFlags.IsSet(BufferFlag::CreateUav) ? D3D11_BIND_UNORDERED_ACCESS : 0);

  rt->texture.desc = CD3D11_TEXTURE2D_DESC(format, width, height, 1, mip_levels, flags);
  rt->texture.desc.MiscFlags = bufferFlags.IsSet(BufferFlag::CreateMipMaps) ? D3D11_RESOURCE_MISC_GENERATE_MIPS : 0;
  if (FAILED(_device->CreateTexture2D(&rt->texture.desc, NULL, &rt->texture.ptr.p)))
    return false;

  // create the render target view
  rt->view.desc = CD3D11_RENDER_TARGET_VIEW_DESC(D3D11_RTV_DIMENSION_TEXTURE2D, rt->texture.desc.Format);
  if (FAILED(_device->CreateRenderTargetView(rt->texture.ptr, &rt->view.desc, &rt->view.ptr.p)))
    return false;

  if (bufferFlags.IsSet(BufferFlag::CreateDepthBuffer))
  {
    // create the depth stencil texture
    ds->texture.desc = CD3D11_TEXTURE2D_DESC(DXGI_FORMAT_D24_UNORM_S8_UINT, width, height, 1, 1, D3D11_BIND_DEPTH_STENCIL);
    if (FAILED(_device->CreateTexture2D(&ds->texture.desc, NULL, &ds->texture.ptr.p)))
      return false;

    // create depth stencil view
    ds->view.desc = CD3D11_DEPTH_STENCIL_VIEW_DESC(D3D11_DSV_DIMENSION_TEXTURE2D, DXGI_FORMAT_D24_UNORM_S8_UINT);
    if (FAILED(_device->CreateDepthStencilView(ds->texture.ptr, &ds->view.desc, &ds->view.ptr.p)))
      return false;
  }

  if (bufferFlags.IsSet(BufferFlag::CreateSrv))
  {
    // create the shader resource view
    rt->srv.desc = CD3D11_SHADER_RESOURCE_VIEW_DESC(D3D11_SRV_DIMENSION_TEXTURE2D, rt->texture.desc.Format);
    if (FAILED(_device->CreateShaderResourceView(rt->texture.ptr, &rt->srv.desc, &rt->srv.ptr.p)))
      return false;
  }

  if (bufferFlags.IsSet(BufferFlag::CreateUav))
  {
    rt->uav.desc = CD3D11_UNORDERED_ACCESS_VIEW_DESC(D3D11_UAV_DIMENSION_TEXTURE2D, format, 0, 0, width*height);
    if (FAILED(_device->CreateUnorderedAccessView(rt->texture.ptr, &rt->uav.desc, &rt->uav.ptr)))
      return false;
  }

  return true;
}

//------------------------------------------------------------------------------
bool Graphics::ReadTexture(
    const char *filename,
    D3DX11_IMAGE_INFO *info,
    u32 *pitch,
    vector<u8> *bits)
{
  HRESULT hr;
  D3DX11GetImageInfoFromFileA(filename, NULL, info, &hr);
  if (FAILED(hr))
    return false;

  D3DX11_IMAGE_LOAD_INFO loadinfo;
  ZeroMemory(&loadinfo, sizeof(D3DX11_IMAGE_LOAD_INFO));
  loadinfo.CpuAccessFlags = D3D11_CPU_ACCESS_READ;
  loadinfo.Usage = D3D11_USAGE_STAGING;
  loadinfo.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
  CComPtr<ID3D11Resource> resource;
  D3DX11CreateTextureFromFileA(_device, filename, &loadinfo, NULL, &resource.p, &hr);
  if (FAILED(hr))
    return false;

  D3D11_MAPPED_SUBRESOURCE sub;
  if (FAILED(_immediateContext->Map(resource, 0, D3D11_MAP_READ, 0, &sub)))
    return false;

  u8 *src = (u8 *)sub.pData;
  bits->resize(sub.RowPitch * info->Height);
  u8 *dst = &(*bits)[0];

  int row_size = 4 * info->Width;
  for (u32 i = 0; i < info->Height; ++i)
  {
    memcpy(&dst[i*sub.RowPitch], &src[i*sub.RowPitch], row_size);
  }

  _immediateContext->Unmap(resource, 0);
  *pitch = sub.RowPitch;
  return true;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::GetTexture(const char *filename)
{
  return MakeObjectHandle(ObjectHandle::kResource, _resources.IndexFromKey(filename));
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::LoadTexture(
    const char* filename,
    const char* friendlyName,
    bool srgb,
    D3DX11_IMAGE_INFO *info)
{
  D3DX11_IMAGE_INFO imageInfo;
  if (FAILED(D3DX11GetImageInfoFromFileA(filename, NULL, &imageInfo, NULL)))
    return emptyHandle;

  if (info)
    *info = imageInfo;

  auto data = unique_ptr<SimpleResource>(new SimpleResource());
  if (FAILED(D3DX11CreateTextureFromFileA(_device, filename, NULL, NULL, &data->resource, NULL)))
    return emptyHandle;

  // TODO: allow for srgb loading
  auto fmt = DXGI_FORMAT_R8G8B8A8_UNORM;
  // check if this is a cube map
  auto dim = imageInfo.MiscFlags & D3D11_RESOURCE_MISC_TEXTURECUBE ? D3D11_SRV_DIMENSION_TEXTURECUBE : D3D11_SRV_DIMENSION_TEXTURE2D;
  auto desc = CD3D11_SHADER_RESOURCE_VIEW_DESC(dim, fmt);
  if (FAILED(_device->CreateShaderResourceView(data->resource, &desc, &data->view.ptr)))
    return emptyHandle;

  return MakeObjectHandle(ObjectHandle::kResource, 
    _resources.Insert(friendlyName ? friendlyName : filename, data.release()));
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::LoadTextureFromMemory(
    const void *buf,
    u32 len,
    const char *friendlyName,
    bool srgb,
    D3DX11_IMAGE_INFO *info)
{
  HRESULT hr;
  if (info && FAILED(D3DX11GetImageInfoFromMemory(buf, len, NULL, info, &hr)))
    return emptyHandle;

  auto data = unique_ptr<SimpleResource>(new SimpleResource());
  if (FAILED(D3DX11CreateTextureFromMemory(_device, buf, len, NULL, NULL, &data->resource, &hr)))
  {
    return emptyHandle;
  }

  // TODO: allow for srgb loading
  auto desc = CD3D11_SHADER_RESOURCE_VIEW_DESC(D3D11_SRV_DIMENSION_TEXTURE2D, DXGI_FORMAT_R8G8B8A8_UNORM);
  if (FAILED(_device->CreateShaderResourceView(data->resource, &desc, &data->view.ptr)))
    return emptyHandle;

  return MakeObjectHandle(
      ObjectHandle::kResource, _resources.Insert(friendlyName, data.release()));
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::InsertTexture(
    TextureResource *data,
    const char* friendlyName)
{
  return friendlyName 
    ? MakeObjectHandle(ObjectHandle::kTexture, _textures.Insert(friendlyName, data))
    : MakeObjectHandle(ObjectHandle::kTexture, _textures.Insert(data));
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::CreateTexture(
    const D3D11_TEXTURE2D_DESC &desc,
    const char *name)
{
  TextureResource *data = new TextureResource;
  if (!CreateTexture(desc, data))
  {
    delete exch_null(data);
    return emptyHandle;
  }
  return InsertTexture(data, name);
}

//------------------------------------------------------------------------------
bool Graphics::CreateTexture(
    const D3D11_TEXTURE2D_DESC &desc,
    TextureResource *out)
{
  out->Reset();

  // create the texture
  out->texture.desc = desc;
  if (FAILED(_device->CreateTexture2D(&out->texture.desc, NULL, &out->texture.ptr.p)))
    return false;

  // create the shader resource view if the texture has a shader resource bind flag
  if (desc.BindFlags & D3D11_BIND_SHADER_RESOURCE)
  {
    out->view.desc = CD3D11_SHADER_RESOURCE_VIEW_DESC(D3D11_SRV_DIMENSION_TEXTURE2D, out->texture.desc.Format);
    if (FAILED(_device->CreateShaderResourceView(out->texture.ptr, &out->view.desc, &out->view.ptr.p)))
      return false;
  }

  return true;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::CreateTexture(int width, int height, DXGI_FORMAT fmt, void* data, int pitch)
{
  TextureResource* resource = new TextureResource();
  if (!CreateTexture(width, height, fmt, data, width, height, pitch, resource))
  {
    delete exch_null(resource);
    return emptyHandle;
  }

  return InsertTexture(resource);
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::CreateTexture(
    int width,
    int height,
    DXGI_FORMAT fmt, 
    void *data_bits,
    int data_width,
    int data_height,
    int data_pitch,
    const char *friendlyName)
{
  TextureResource *data = new TextureResource();
  if (!CreateTexture(width, height, fmt, data_bits, data_width, data_height, data_pitch, data))
  {
    delete exch_null(data);
    return emptyHandle;
  }
  return InsertTexture(data, friendlyName);
}

//------------------------------------------------------------------------------
bool Graphics::CreateTexture(
    int width,
    int height,
    DXGI_FORMAT fmt,
    void *data,
    int data_width,
    int data_height,
    int data_pitch,
    TextureResource *out)
{
  if (!CreateTexture(CD3D11_TEXTURE2D_DESC(fmt, width, height, 1, 1, D3D11_BIND_SHADER_RESOURCE, D3D11_USAGE_DYNAMIC, D3D11_CPU_ACCESS_WRITE), out))
    return false;

  D3D11_MAPPED_SUBRESOURCE resource;
  if (FAILED(_immediateContext->Map(out->texture.ptr, 0, D3D11_MAP_WRITE_DISCARD, 0, &resource)))
    return false;

  uint8_t *src = (uint8_t *)data;
  uint8_t *dst = (uint8_t *)resource.pData;
  const int w = std::min<int>(width, data_width);
  const int h = std::min<int>(height, data_height);
  for (int i = 0; i < h; ++i)
  {
    memcpy(dst, src, data_pitch);
    src += data_pitch;
    dst += resource.RowPitch;
  }
  _immediateContext->Unmap(out->texture.ptr, 0);
  return true;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::CreateInputLayout(
    const vector<D3D11_INPUT_ELEMENT_DESC> &desc,
    const vector<char> &shader_bytecode)
{
  ID3D11InputLayout* layout = nullptr;
  if (FAILED(_device->CreateInputLayout(&desc[0], (u32)desc.size(), &shader_bytecode[0], shader_bytecode.size(), &layout)))
  {
    LOG_WARN("Error creating input layout");
    return emptyHandle;
  }

  return ObjectHandle(ObjectHandle::kInputLayout, _inputLayouts.Insert(layout));
}

//------------------------------------------------------------------------------
template<typename T, class Cont>
ObjectHandle AddShader(
    Cont &cont,
    T *shader,
    const string &id,
    ObjectHandle::Type type)
{
  return Graphics::MakeObjectHandle(type, cont.Insert(id, shader));
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::ReserveObjectHandle(const string& id, ObjectHandle::Type type)
{
  switch (type)
  {
    case ObjectHandle::kVertexShader: return AddShader<ID3D11VertexShader>(_vertexShaders, nullptr, id, type);
    case ObjectHandle::kGeometryShader: return AddShader<ID3D11GeometryShader>(_geometryShaders, nullptr, id, type);
    case ObjectHandle::kPixelShader: return AddShader<ID3D11PixelShader>(_pixelShaders, nullptr, id, type);
    case ObjectHandle::kComputeShader: return AddShader<ID3D11ComputeShader>(_computeShaders, nullptr, id, type);
    default: assert("Unsupported object type");
  }

  return emptyHandle;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::CreateVertexShader(
    const vector<char> &shader_bytecode,
    const string &id)
{
  ID3D11VertexShader *vs = nullptr;
  if (SUCCEEDED(_device->CreateVertexShader(&shader_bytecode[0], shader_bytecode.size(), NULL, &vs)))
  {
    return AddShader(_vertexShaders, vs, id, ObjectHandle::kVertexShader);
  }
  return emptyHandle;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::CreatePixelShader(
    const vector<char> &shader_bytecode,
    const string &id)
{
  ID3D11PixelShader *ps = nullptr;
  if (SUCCEEDED(_device->CreatePixelShader(&shader_bytecode[0], shader_bytecode.size(), NULL, &ps)))
  {
    return AddShader(_pixelShaders, ps, id, ObjectHandle::kPixelShader);
  }
  return emptyHandle;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::CreateComputeShader(
    const vector<char> &shader_bytecode,
    const string &id)
{
  ID3D11ComputeShader *cs = nullptr;
  if (SUCCEEDED(_device->CreateComputeShader(&shader_bytecode[0], shader_bytecode.size(), NULL, &cs)))
  {
    return AddShader(_computeShaders, cs, id, ObjectHandle::kComputeShader);
  }
  return emptyHandle;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::CreateGeometryShader(
    const vector<char> &shader_bytecode,
    const string &id)
{
  ID3D11GeometryShader *cs = nullptr;
  if (SUCCEEDED(_device->CreateGeometryShader(&shader_bytecode[0], shader_bytecode.size(), NULL, &cs)))
  {
    return AddShader(_geometryShaders, cs, id, ObjectHandle::kGeometryShader);
  }
  return emptyHandle;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::CreateRasterizerState(
    const D3D11_RASTERIZER_DESC &desc,
    const char *name)
{
  ID3D11RasterizerState *rs;
  if (SUCCEEDED(_device->CreateRasterizerState(&desc, &rs)))
  {
    return MakeObjectHandle(ObjectHandle::kRasterizerState,
        name 
          ? _rasterizerStates.Insert(name, rs)
          : _rasterizerStates.Insert(rs));
  }
  return emptyHandle;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::CreateBlendState(
    const D3D11_BLEND_DESC &desc,
    const char *name)
{
  ID3D11BlendState *bs;
  if (SUCCEEDED(_device->CreateBlendState(&desc, &bs)))
  {
    return MakeObjectHandle(ObjectHandle::kBlendState,
        name 
          ? _blendStates.Insert(name, bs)
          : _blendStates.Insert(bs));
  }
  return emptyHandle;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::CreateDepthStencilState(
    const D3D11_DEPTH_STENCIL_DESC &desc,
    const char *name)
{
  ID3D11DepthStencilState *dss;
  if (SUCCEEDED(_device->CreateDepthStencilState(&desc, &dss)))
  {
    return MakeObjectHandle(ObjectHandle::kDepthStencilState,
        name 
          ? _depthStencilStates.Insert(name, dss)
          : _depthStencilStates.Insert(dss));
  }
  return emptyHandle;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::CreateSamplerState(
    const D3D11_SAMPLER_DESC &desc,
    const char *name)
{
  ID3D11SamplerState *ss;
  if (SUCCEEDED(_device->CreateSamplerState(&desc, &ss)))
  {
    return MakeObjectHandle(ObjectHandle::kSamplerState,
      name ? _samplerStates.Insert(name, ss) : _samplerStates.Insert(ss));
  }
  return emptyHandle;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::CreateSwapChain(
    const TCHAR* name,
    u32 width,
    u32 height,
    DXGI_FORMAT format,
    WNDPROC wndProc,
    HINSTANCE instance)
{
  // Register the window class
  WNDCLASSEX wcex;
  ZeroMemory(&wcex, sizeof(wcex));
  wcex.cbSize         = sizeof(WNDCLASSEX);
  wcex.style          = CS_HREDRAW | CS_VREDRAW;
  wcex.lpfnWndProc    = wndProc;
  wcex.hInstance      = instance;
  wcex.hbrBackground  = 0;
  wcex.lpszClassName  = name;
  wcex.hCursor        = LoadCursor(NULL, IDC_ARROW);

  if (!RegisterClassEx(&wcex))
    return emptyHandle;

#if BORDERLESS_WINDOW
  UINT windowStyle = WS_POPUP;
#else
  UINT windowStyle = WS_OVERLAPPEDWINDOW | WS_VISIBLE | WS_CLIPCHILDREN | WS_CLIPSIBLINGS;
#endif

  // Create/resize the window
  HWND hwnd = CreateWindow(name, g_AppWindowTitle, windowStyle,
    CW_USEDEFAULT, CW_USEDEFAULT, width, height, NULL, NULL, instance, NULL);

  SetClientSize(hwnd, width, height);

  // if doing borderless, center the window as well
#if BORDERLESS_WINDOW

  int desktopWidth = GetSystemMetrics(SM_CXFULLSCREEN);
  int dekstopHeight = GetSystemMetrics(SM_CYFULLSCREEN);

  int centerX = (desktopWidth - width) / 2;
  int centerY = (dekstopHeight - height) / 2;
  SetWindowPos(hwnd, NULL, centerX, centerY, - 1, -1, SWP_NOZORDER | SWP_NOSIZE);
#endif

  ShowWindow(hwnd, SW_SHOW);

  // Create the swap chain
  DXGI_SWAP_CHAIN_DESC swapChainDesc;
  IDXGISwapChain* sc = 0;
  ZeroMemory(&swapChainDesc, sizeof(swapChainDesc));
  swapChainDesc.BufferCount            = 2;
  swapChainDesc.BufferDesc.Width       = width;
  swapChainDesc.BufferDesc.Height      = height;
  swapChainDesc.BufferDesc.Format      = format;
  swapChainDesc.BufferDesc.RefreshRate = SelectedDisplayMode().RefreshRate;
  swapChainDesc.BufferUsage            = DXGI_USAGE_RENDER_TARGET_OUTPUT;
  swapChainDesc.OutputWindow           = hwnd;
  swapChainDesc.SampleDesc.Count       = _curSetup.multisampleCount;
  swapChainDesc.SampleDesc.Quality     = 0;
  swapChainDesc.Windowed               = _curSetup.windowed;

  if (FAILED(_curSetup.dxgi_factory->CreateSwapChain(_device, &swapChainDesc, &sc)))
    return emptyHandle;

  SwapChain* swapChain = new SwapChain(name);
  swapChain->_hwnd = hwnd;
  swapChain->_desc = swapChainDesc;
  swapChain->_swapChain = sc;
  if (!swapChain->CreateBackBuffers(width, height, format))
    return emptyHandle;

  return ObjectHandle(ObjectHandle::kSwapChain, _swapChains.Insert(swapChain));
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::FindResource(const string& name)
{
  // check textures, then resources, then render targets
  int idx = _textures.IndexFromKey(name);
  if (idx != _textures.INVALID_INDEX)
    return ObjectHandle(ObjectHandle::kTexture, idx);

  idx = _resources.IndexFromKey(name);
  if (idx != _resources.INVALID_INDEX)
    return ObjectHandle(ObjectHandle::kResource, idx);

  idx = _renderTargets.IndexFromKey(name);
  return MakeObjectHandle(ObjectHandle::kRenderTarget, idx);
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::FindSampler(const string &name)
{
  return MakeObjectHandle(ObjectHandle::kSamplerState, _samplerStates.IndexFromKey(name));
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::FindBlendState(const string &name)
{
  return MakeObjectHandle(ObjectHandle::kBlendState, _blendStates.IndexFromKey(name));
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::FindRasterizerState(const string &name)
{
  return MakeObjectHandle(ObjectHandle::kRasterizerState, _rasterizerStates.IndexFromKey(name));
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::FindDepthStencilState(const string &name)
{
  return MakeObjectHandle(ObjectHandle::kDepthStencilState, _depthStencilStates.IndexFromKey(name));
}

//------------------------------------------------------------------------------
SwapChain* Graphics::GetSwapChain(ObjectHandle h)
{
  return h.IsValid() ? _swapChains.Get(h) : _defaultSwapChain;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::MakeObjectHandle(
    ObjectHandle::Type type,
    int idx,
    int data)
{
  return idx != -1 ? ObjectHandle(type, idx, data) : emptyHandle;
}

//------------------------------------------------------------------------------
GraphicsContext* Graphics::GetGraphicsContext()
{
  return _graphicsContext;
}

//------------------------------------------------------------------------------
PostProcess* Graphics::GetPostProcess()
{
  return _postProcess;
}

//------------------------------------------------------------------------------
void Graphics::CreateDefaultSwapChain(
    u32 width,
    u32 height,
    DXGI_FORMAT format,
    WNDPROC wndProc,
    HINSTANCE instance)
{
  _defaultSwapChainHandle = CreateSwapChain("default", width, height, format, wndProc, instance);
  _defaultSwapChain = _swapChains.Get(_defaultSwapChainHandle);
}

//------------------------------------------------------------------------------
void Graphics::Present()
{
  _defaultSwapChain->Present();
}

//------------------------------------------------------------------------------
void Graphics::GetBackBufferSize(int* width, int* height)
{
  *width = (int)_defaultSwapChain->_viewport.Width;
  *height = (int)_defaultSwapChain->_viewport.Height;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::GetBackBuffer()
{
  return _defaultSwapChain->_renderTarget;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::GetDepthStencil()
{
  return _defaultSwapChain->_depthStencil;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::DefaultSwapChain()
{
  return _defaultSwapChainHandle;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::RenderTargetForSwapChain(ObjectHandle h)
{
  if (!h.IsValid())
    return ObjectHandle();

  auto swapChain = GRAPHICS._swapChains.Get(h);
  return swapChain->_renderTarget;
}

//------------------------------------------------------------------------------
void Graphics::GetRenderTargetTextureDesc(
  ObjectHandle handle,
  D3D11_TEXTURE2D_DESC* desc)
{
  auto rt = GRAPHICS._renderTargets.Get(handle.IsValid() ? handle : GRAPHICS._defaultRenderTarget);
  *desc = rt->texture.desc;
}

//------------------------------------------------------------------------------
bool Graphics::LoadComputeShadersFromFile(
    const string& filenameBase,
    ObjectHandle* shader,
    const char* entry)
{
#if WITH_DEBUG_SHADERS
  string suffix = ToString("_%sD.cso", entry);
#else
  string suffix = ToString("_%s.cso", entry);
#endif

  AddFileWatchResult res = RESOURCE_MANAGER.AddFileWatch((filenameBase + suffix).c_str(), nullptr, true, 
  [=](const string& filename, void* token)
  {
    vector<char> buf;
    if (!RESOURCE_MANAGER.LoadFile(filename.c_str(), &buf))
    {
      LOG_WARN("Unable to load shader" << LogKeyValue("filename", filename));
      return false;
    }

    *shader = GRAPHICS.CreateComputeShader(buf, entry);
    if (!shader->IsValid())
    {
      LOG_WARN("Unable to create shader" << LogKeyValue("filename", filename));
      return false;
    }

    return true;
  });

  if (!res.initialResult)
    return false;

  return true;
}

//------------------------------------------------------------------------------
bool Graphics::LoadShadersFromFile(
    const string& filenameBase,
    ObjectHandle* vs,
    ObjectHandle* gs,
    ObjectHandle* ps,
    ObjectHandle* inputLayout,
    vector<D3D11_INPUT_ELEMENT_DESC>* elements,
    const char* vsEntry,
    const char* gsEntry,
    const char* psEntry)
{
#if WITH_DEBUG_SHADERS
  string vsSuffix = vs ? ToString("_%sD.vso", vsEntry) : string();
  string gsSuffix = gs ? ToString("_%sD.gso", gsEntry) : string();
  string psSuffix = ps ? ToString("_%sD.pso", psEntry) : string();
#else
  string vsSuffix = vs ? ToString("_%s.vso", vsEntry) : string();
  string gsSuffix = gs ? ToString("_%s.gso", gsEntry) : string();
  string psSuffix = ps ? ToString("_%s.pso", psEntry) : string();
#endif

  string vsId = vs ? filenameBase + vsEntry : string();
  string gsId = gs ? filenameBase + gsEntry : string();
  string psId = ps ? filenameBase + psEntry : string();

  AddFileWatchResult res;
  if (vs)
  {
    ObjectHandle vsHandle = ReserveObjectHandle(vsId, ObjectHandle::kVertexShader);
    *vs = vsHandle;

    vector<D3D11_INPUT_ELEMENT_DESC> localElementDesc;
    if (elements)
      localElementDesc = *elements;

    res = RESOURCE_MANAGER.AddFileWatch((filenameBase + vsSuffix).c_str(), nullptr, true, [=](const string& filename, void* token)
    {
      vector<char> buf;
      ID3D11VertexShader *vs = nullptr;

      if (
        RESOURCE_MANAGER.LoadFile(filename.c_str(), &buf) &&
        SUCCEEDED(_device->CreateVertexShader(buf.data(), buf.size(), NULL, &vs)))
      {
        _vertexShaders.Update(vsHandle, vs);
        if (inputLayout)
        {
          *inputLayout = GRAPHICS.CreateInputLayout(localElementDesc, buf);
          if (!inputLayout->IsValid())
            return false;
        }
        return true;
      }

      LOG_WARN(ToString("Unable to %s vertex shader", buf.empty() ? "load" : "create").c_str()
        << LogKeyValue("filename", filename));
      return false;
    });
  }

  if (!res.initialResult)
    return false;

  if (gs)
  {
    ObjectHandle gsHandle = ReserveObjectHandle(gsId, ObjectHandle::kGeometryShader);
    *gs = gsHandle;

    res = RESOURCE_MANAGER.AddFileWatch((filenameBase + gsSuffix).c_str(), nullptr, true, [=](const string& filename, void* token)
    {
      vector<char> buf;
      ID3D11GeometryShader* gs = nullptr;
      if (
        RESOURCE_MANAGER.LoadFile(filename.c_str(), &buf) && 
        SUCCEEDED(_device->CreateGeometryShader(buf.data(), buf.size(), NULL, &gs)))
      {
        _geometryShaders.Update(gsHandle, gs);
        return true;
      }
      LOG_WARN(ToString("Unable to %s geometry shader", buf.empty() ? "load" : "create").c_str()
        << LogKeyValue("filename", filename));
      return false;
    });
  }

  if (ps)
  {
    ObjectHandle psHandle = ReserveObjectHandle(psId, ObjectHandle::kPixelShader);
    *ps = psHandle;

    res = RESOURCE_MANAGER.AddFileWatch((filenameBase + psSuffix).c_str(), nullptr, true, [=](const string& filename, void* token)
    {
      vector<char> buf;
      ID3D11PixelShader *ps = nullptr;

      if (
        RESOURCE_MANAGER.LoadFile(filename.c_str(), &buf) &&
        SUCCEEDED(_device->CreatePixelShader(buf.data(), buf.size(), NULL, &ps)))
      {
        _pixelShaders.Update(psHandle, ps);
        return true;
      }

      LOG_WARN(ToString("Unable to %s pixel shader", buf.empty() ? "load" : "create").c_str()
        << LogKeyValue("filename", filename));
      return false;
    });
  }

  return res.initialResult;
}
