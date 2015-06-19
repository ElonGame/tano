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
  InitDefaultDescs();

  _postProcess = new PostProcess(_graphicsContext);
  INIT(_postProcess->Init());


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
      const int idx = _indexBuffers.Append(buffer);
      assert(userData == DXGI_FORMAT_R16_UINT || userData == DXGI_FORMAT_R32_UINT);
      return MakeObjectHandle(ObjectHandle::kIndexBuffer, idx, userData);
    } 
    else if (bind == D3D11_BIND_VERTEX_BUFFER)
    {
      const int idx = _vertexBuffers.Append(buffer);
      // userdata is vertex size
      assert(userData > 0);
      return MakeObjectHandle(ObjectHandle::kVertexBuffer, idx, userData);
    } 
    else if (bind == D3D11_BIND_CONSTANT_BUFFER)
    {
      const int idx = _constantBuffers.Append(buffer);
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

  switch (type)
  {
    case ObjectHandle::kTexture:
      return _textures.Get(h)->view.ptr;

    case ObjectHandle::kResource:
      return _resources.Get(h)->view.ptr;

    case ObjectHandle::kRenderTarget:
      return _renderTargets.Get(h)->srv.ptr;

    case ObjectHandle::kStructuredBuffer:
      return _structuredBuffers.Get(h)->srv.ptr;

    case ObjectHandle::kDepthStencil:
      return _depthStencils.Get(h)->srv.ptr;

    default:
      LOG_WARN("Trying to set a non supported resource view type!");
      return nullptr;
  }
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
    texture.Attach((ID3D11Texture2D*)_resources.Get(h)->view.ptr);
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
ObjectHandle Graphics::GetTempRenderTarget(int width, int height, DXGI_FORMAT format, const BufferFlags& flags)
{
  // Look for a temp render target with the required format etc
  for (TempRenderTarget& t : _tempRenderTargets)
  {
    const D3D11_TEXTURE2D_DESC& d = t.desc;
    if (!t.inUse && d.Width == width && d.Height == height && d.Format == format && t.flags == flags)
    {
      // Found a free render target!
      t.inUse = true;
      return MakeObjectHandle(ObjectHandle::kRenderTarget, t.idx);
    }
  }

  // Render target not found, so create a new one
  RenderTargetResource* rt = CreateRenderTargetPtr(width, height, format, flags);
  int rtIdx = _renderTargets.Append(rt);
  _tempRenderTargets.Append({ rt->texture.desc, flags, rtIdx, true });

  return MakeObjectHandle(ObjectHandle::kRenderTarget, rtIdx);
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::GetTempDepthStencil(int width, int height, const BufferFlags& flags)
{
  for (TempDepthStencil& t : _tempDepthStencils)
  {
    if (!t.inUse && t.flags == flags)
    {
      t.inUse = true;
      return MakeObjectHandle(ObjectHandle::kDepthStencil, t.idx);
    }
  }

  // Render target not found, so create a new one
  DepthStencilResource* ds = CreateDepthStencilPtr(width, height, flags);
  int dsIdx = _depthStencils.Append(ds);
  _tempDepthStencils.Append({ flags, dsIdx, true });

  return MakeObjectHandle(ObjectHandle::kDepthStencil, dsIdx);
}

//------------------------------------------------------------------------------
void Graphics::ReleaseTempRenderTarget(ObjectHandle h)
{
  if (!h.IsValid())
    return;

  for (TempRenderTarget& t : _tempRenderTargets)
  {
    if (t.idx == h.id())
    {
      assert(t.inUse);
      t.inUse = false;
      break;
    }
  }
}

//------------------------------------------------------------------------------
void Graphics::ReleaseTempDepthStencil(ObjectHandle h)
{
  if (!h.IsValid())
    return;

  for (TempDepthStencil& t : _tempDepthStencils)
  {
    if (t.idx == h.id())
    {
      assert(t.inUse);
      t.inUse = false;
      break;
    }
  }
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

  ID3D11Buffer* buf = sb->buffer.ptr;

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
      ObjectHandle::kStructuredBuffer, _structuredBuffers.Append(sb.release()));
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::CreateDepthStencil(int width, int height, const BufferFlags& bufferFlags)
{
  if (DepthStencilResource* ds = CreateDepthStencilPtr(width, height, bufferFlags))
  {
    int dsIdx = _depthStencils.Append(ds);
    return MakeObjectHandle(ObjectHandle::kDepthStencil, dsIdx);
  }

  return emptyHandle;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::CreateRenderTarget(int width, int height, DXGI_FORMAT format, const BufferFlags& flags)
{
  if (RenderTargetResource* rt = CreateRenderTargetPtr(width, height, format, flags))
  {
    int idx = _renderTargets.Append(rt);
    return MakeObjectHandle(ObjectHandle::kRenderTarget, idx);
  }
  return emptyHandle;
}

//------------------------------------------------------------------------------
RenderTargetResource* Graphics::CreateRenderTargetPtr(
    int width,
    int height,
    DXGI_FORMAT format,
    const BufferFlags& flags)
{
  RenderTargetResource* rt = new RenderTargetResource();
  ScopeGuard s([=]() { delete rt; });

  // create the render target
  int mip_levels = flags.IsSet(BufferFlag::CreateMipMaps) ? 0 : 1;
  u32 bindFlags = D3D11_BIND_RENDER_TARGET 
    | (flags.IsSet(BufferFlag::CreateSrv) ? D3D11_BIND_SHADER_RESOURCE : 0)
    | (flags.IsSet(BufferFlag::CreateUav) ? D3D11_BIND_UNORDERED_ACCESS : 0);

  rt->texture.desc = CD3D11_TEXTURE2D_DESC(format, width, height, 1, mip_levels, bindFlags);
  rt->texture.desc.MiscFlags = flags.IsSet(BufferFlag::CreateMipMaps) ? D3D11_RESOURCE_MISC_GENERATE_MIPS : 0;
  if (FAILED(_device->CreateTexture2D(&rt->texture.desc, NULL, &rt->texture.ptr)))
    return nullptr;

  // create the render target view
  rt->view.desc = CD3D11_RENDER_TARGET_VIEW_DESC(D3D11_RTV_DIMENSION_TEXTURE2D, rt->texture.desc.Format);
  if (FAILED(_device->CreateRenderTargetView(rt->texture.ptr, &rt->view.desc, &rt->view.ptr)))
    return nullptr;

  if (flags.IsSet(BufferFlag::CreateSrv))
  {
    // create the shader resource view
    rt->srv.desc = CD3D11_SHADER_RESOURCE_VIEW_DESC(D3D11_SRV_DIMENSION_TEXTURE2D, rt->texture.desc.Format);
    if (FAILED(_device->CreateShaderResourceView(rt->texture.ptr, &rt->srv.desc, &rt->srv.ptr)))
      return nullptr;
  }

  if (flags.IsSet(BufferFlag::CreateUav))
  {
    rt->uav.desc = CD3D11_UNORDERED_ACCESS_VIEW_DESC(D3D11_UAV_DIMENSION_TEXTURE2D, format, 0, 0, width*height);
    if (FAILED(_device->CreateUnorderedAccessView(rt->texture.ptr, &rt->uav.desc, &rt->uav.ptr)))
      return nullptr;
  }

  s.Commit();
  return rt;
}

//------------------------------------------------------------------------------
DepthStencilResource* Graphics::CreateDepthStencilPtr(
    int width,
    int height,
    const BufferFlags& flags)
{
  DepthStencilResource* ds = new DepthStencilResource();
  ScopeGuard s([=]() { delete ds; });

  bool srv = flags.IsSet(BufferFlag::CreateSrv);

  // create the depth stencil texture
  CD3D11_TEXTURE2D_DESC depthStencilDesc(
    DXGI_FORMAT_D24_UNORM_S8_UINT, width, height, 1, 1,
    D3D11_BIND_DEPTH_STENCIL, D3D11_USAGE_DEFAULT, 0);

  if (srv)
  {
    // SRVs require special formats for depth/stencil
    depthStencilDesc.Format = DXGI_FORMAT_R24G8_TYPELESS;
    depthStencilDesc.BindFlags |= (flags.IsSet(BufferFlag::CreateSrv) ? D3D11_BIND_SHADER_RESOURCE : 0);
  }

  // UAV for depth buffer not supported
  assert(!flags.IsSet(BufferFlag::CreateUav));

  if (FAILED(_device->CreateTexture2D(&depthStencilDesc, NULL, &ds->texture.ptr)))
    return nullptr;
  ds->texture.ptr->GetDesc(&ds->texture.desc);

  // create depth stencil view
  CD3D11_DEPTH_STENCIL_VIEW_DESC viewDesc(D3D11_DSV_DIMENSION_TEXTURE2D, DXGI_FORMAT_D24_UNORM_S8_UINT);
  if (FAILED(_device->CreateDepthStencilView(ds->texture.ptr, &viewDesc, &ds->view.ptr)))
    return nullptr;
  ds->view.ptr->GetDesc(&ds->view.desc);

  if (flags.IsSet(BufferFlag::CreateSrv))
  {
    ds->srv.desc = CD3D11_SHADER_RESOURCE_VIEW_DESC(D3D11_SRV_DIMENSION_TEXTURE2D, DXGI_FORMAT_R24_UNORM_X8_TYPELESS);
    if (FAILED(_device->CreateShaderResourceView(ds->texture.ptr, &ds->srv.desc, &ds->srv.ptr)))
      return false;
  }

  s.Commit();
  return ds;
}

//------------------------------------------------------------------------------
bool Graphics::ReadTexture(const char *filename, D3DX11_IMAGE_INFO *info, u32 *pitch, vector<u8> *bits)
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
ObjectHandle Graphics::LoadTexture(
    const char* filename,
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
    _resources.Append(data.release()));
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::LoadTextureFromMemory(
    const void *buf,
    u32 len,
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

  return MakeObjectHandle(ObjectHandle::kResource, _resources.Append(data.release()));
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::InsertTexture(TextureResource *data)
{
  return MakeObjectHandle(ObjectHandle::kTexture, _textures.Append(data));
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::CreateTexture(const D3D11_TEXTURE2D_DESC &desc)
{
  TextureResource* data = CreateTexturePtr(desc);
  return data ? InsertTexture(data) : emptyHandle;
}

//------------------------------------------------------------------------------
TextureResource* Graphics::CreateTexturePtr(const D3D11_TEXTURE2D_DESC &desc)
{
  TextureResource* t = new TextureResource();
  ScopeGuard s([=]() { delete t; });

  // create the texture
  t->texture.desc = desc;
  if (FAILED(_device->CreateTexture2D(&t->texture.desc, NULL, &t->texture.ptr)))
    return nullptr;

  // create the shader resource view if the texture has a shader resource bind flag
  if (desc.BindFlags & D3D11_BIND_SHADER_RESOURCE)
  {
    t->view.desc = CD3D11_SHADER_RESOURCE_VIEW_DESC(D3D11_SRV_DIMENSION_TEXTURE2D, t->texture.desc.Format);
    if (FAILED(_device->CreateShaderResourceView(t->texture.ptr, &t->view.desc, &t->view.ptr)))
      return nullptr;
  }

  s.Commit();
  return t;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::CreateTexture(int width, int height, DXGI_FORMAT fmt, void* data, int pitch)
{
  TextureResource* resource = CreateTexturePtr(width, height, fmt, data, width, height, pitch);
  return resource ? InsertTexture(resource) : emptyHandle;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::CreateTexture(
    int width,
    int height,
    DXGI_FORMAT fmt, 
    void *data_bits,
    int data_width,
    int data_height,
    int data_pitch)
{
  TextureResource* data = CreateTexturePtr(width, height, fmt, data_bits, data_width, data_height, data_pitch);
  return data ? InsertTexture(data) : emptyHandle;
}

//------------------------------------------------------------------------------
TextureResource* Graphics::CreateTexturePtr(
    int width,
    int height,
    DXGI_FORMAT fmt,
    void *data,
    int data_width,
    int data_height,
    int data_pitch)
{
  TextureResource* t = CreateTexturePtr(CD3D11_TEXTURE2D_DESC(
    fmt, width, height, 1, 1, D3D11_BIND_SHADER_RESOURCE, D3D11_USAGE_DYNAMIC, D3D11_CPU_ACCESS_WRITE));
  if (!t)
    return nullptr;

  D3D11_MAPPED_SUBRESOURCE resource;
  if (FAILED(_immediateContext->Map(t->texture.ptr, 0, D3D11_MAP_WRITE_DISCARD, 0, &resource)))
  {
    delete t;
    return nullptr;
  }

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
  _immediateContext->Unmap(t->texture.ptr, 0);

  return t;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::CreateInputLayout(
    const vector<D3D11_INPUT_ELEMENT_DESC> &desc,
    const vector<char> &shaderBytecode)
{
  ID3D11InputLayout* layout = nullptr;
  if (FAILED(_device->CreateInputLayout(&desc[0], (u32)desc.size(), &shaderBytecode[0], shaderBytecode.size(), &layout)))
  {
    LOG_WARN("Error creating input layout");
    return emptyHandle;
  }

  return ObjectHandle(ObjectHandle::kInputLayout, _inputLayouts.Append(layout));
}

//------------------------------------------------------------------------------
template<typename T, class Cont>
ObjectHandle AddShader(
    Cont &cont,
    T *shader,
    ObjectHandle::Type type)
{
  return Graphics::MakeObjectHandle(type, cont.Append(shader));
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::ReserveObjectHandle(ObjectHandle::Type type)
{
  switch (type)
  {
    case ObjectHandle::kVertexShader: return AddShader<ID3D11VertexShader>(_vertexShaders, nullptr, type);
    case ObjectHandle::kGeometryShader: return AddShader<ID3D11GeometryShader>(_geometryShaders, nullptr, type);
    case ObjectHandle::kPixelShader: return AddShader<ID3D11PixelShader>(_pixelShaders, nullptr, type);
    case ObjectHandle::kComputeShader: return AddShader<ID3D11ComputeShader>(_computeShaders, nullptr, type);
    default: assert("Unsupported object type");
  }

  return emptyHandle;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::CreateVertexShader(const vector<char> &shaderBytecode)
{
  ID3D11VertexShader *vs = nullptr;
  if (SUCCEEDED(_device->CreateVertexShader(&shaderBytecode[0], shaderBytecode.size(), NULL, &vs)))
  {
    return AddShader(_vertexShaders, vs, ObjectHandle::kVertexShader);
  }
  return emptyHandle;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::CreatePixelShader(const vector<char> &shaderBytecode)
{
  ID3D11PixelShader *ps = nullptr;
  if (SUCCEEDED(_device->CreatePixelShader(&shaderBytecode[0], shaderBytecode.size(), NULL, &ps)))
  {
    return AddShader(_pixelShaders, ps, ObjectHandle::kPixelShader);
  }
  return emptyHandle;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::CreateComputeShader(const vector<char> &shaderBytecode)
{
  ID3D11ComputeShader *cs = nullptr;
  if (SUCCEEDED(_device->CreateComputeShader(&shaderBytecode[0], shaderBytecode.size(), NULL, &cs)))
  {
    return AddShader(_computeShaders, cs, ObjectHandle::kComputeShader);
  }
  return emptyHandle;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::CreateGeometryShader(const vector<char> &shaderBytecode)
{
  ID3D11GeometryShader *cs = nullptr;
  if (SUCCEEDED(_device->CreateGeometryShader(&shaderBytecode[0], shaderBytecode.size(), NULL, &cs)))
  {
    return AddShader(_geometryShaders, cs, ObjectHandle::kGeometryShader);
  }
  return emptyHandle;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::CreateRasterizerState(const D3D11_RASTERIZER_DESC &desc)
{
  ID3D11RasterizerState *rs;
  if (SUCCEEDED(_device->CreateRasterizerState(&desc, &rs)))
  {
    return MakeObjectHandle(ObjectHandle::kRasterizerState, _rasterizerStates.Append(rs));
  }
  return emptyHandle;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::CreateBlendState(const D3D11_BLEND_DESC &desc)
{
  ID3D11BlendState *bs;
  if (SUCCEEDED(_device->CreateBlendState(&desc, &bs)))
  {
    return MakeObjectHandle(ObjectHandle::kBlendState, _blendStates.Append(bs));
  }
  return emptyHandle;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::CreateDepthStencilState(const D3D11_DEPTH_STENCIL_DESC &desc)
{
  ID3D11DepthStencilState *dss;
  if (SUCCEEDED(_device->CreateDepthStencilState(&desc, &dss)))
  {
    return MakeObjectHandle(ObjectHandle::kDepthStencilState, _depthStencilStates.Append(dss));
  }
  return emptyHandle;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::CreateSamplerState(const D3D11_SAMPLER_DESC &desc)
{
  ID3D11SamplerState *ss;
  if (SUCCEEDED(_device->CreateSamplerState(&desc, &ss)))
  {
    return MakeObjectHandle(ObjectHandle::kSamplerState, _samplerStates.Append(ss));
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

  return ObjectHandle(ObjectHandle::kSwapChain, _swapChains.Append(swapChain));
}

//------------------------------------------------------------------------------
SwapChain* Graphics::GetSwapChain(ObjectHandle h)
{
  return h.IsValid() ? _swapChains.Get(h) : _defaultSwapChain;
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::MakeObjectHandle(ObjectHandle::Type type, int idx, int data)
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
ObjectHandle Graphics::LoadVertexShaderFromFile(
  const string& filenameBase,
  const char* entry,
  ObjectHandle* inputLayout,
  vector<D3D11_INPUT_ELEMENT_DESC>* elements)
{
  ObjectHandle handle = ReserveObjectHandle(ObjectHandle::kVertexShader);

#if WITH_DEBUG_SHADERS
  string filename = filenameBase + ToString("_%sD.vso", entry);
#else
  string filename = filenameBase + ToString("_%s.vso", entry);
#endif

  vector<D3D11_INPUT_ELEMENT_DESC> localElementDesc;
  if (elements)
    localElementDesc = *elements;

  AddFileWatchResult res = RESOURCE_MANAGER.AddFileWatch(filename.c_str(), true, [=](const string& filename, void*)
  {
    BEGIN_INIT_SEQUENCE();
    vector<char> buf;
    ID3D11VertexShader* shader = nullptr;

    INIT_FATAL(RESOURCE_MANAGER.LoadFile(filename.c_str(), &buf));
    INIT_HR_FATAL(_device->CreateVertexShader(buf.data(), buf.size(), NULL, &shader));
    _vertexShaders.Update(handle, shader);

    if (inputLayout)
    {
      INIT_RESOURCE(*inputLayout, GRAPHICS.CreateInputLayout(localElementDesc, buf));
    }

    END_INIT_SEQUENCE();
  });

  return res.initialResult ? handle : ObjectHandle();
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::LoadPixelShaderFromFile(const string& filenameBase, const char* entry)
{
  ObjectHandle handle = ReserveObjectHandle(ObjectHandle::kPixelShader);

#if WITH_DEBUG_SHADERS
  string filename = filenameBase + ToString("_%sD.pso", entry);
#else
  string filename = filenameBase + ToString("_%s.pso", entry);
#endif

  AddFileWatchResult res = RESOURCE_MANAGER.AddFileWatch(filename, true, [=](const string& filename, void*)
  {
    BEGIN_INIT_SEQUENCE();

    vector<char> buf;
    ID3D11PixelShader* shader = nullptr;

    INIT_FATAL(RESOURCE_MANAGER.LoadFile(filename.c_str(), &buf));
    INIT_HR_FATAL(_device->CreatePixelShader(buf.data(), buf.size(), NULL, &shader));
    _pixelShaders.Update(handle, shader);

    END_INIT_SEQUENCE();
  });

  return res.initialResult ? handle : ObjectHandle();
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::LoadGeometryShaderFromFile(const string& filenameBase, const char* entry)
{
  ObjectHandle handle = ReserveObjectHandle(ObjectHandle::kGeometryShader);

#if WITH_DEBUG_SHADERS
  string filename = filenameBase + ToString("_%sD.gso", entry);
#else
  string filename = filenameBase + ToString("_%s.gso", entry);
#endif

  AddFileWatchResult res = RESOURCE_MANAGER.AddFileWatch(filename.c_str(), true, [=](const string& filename, void*)
  {
    BEGIN_INIT_SEQUENCE();

    vector<char> buf;
    ID3D11GeometryShader* shader = nullptr;

    INIT_FATAL(RESOURCE_MANAGER.LoadFile(filename.c_str(), &buf));
    INIT_HR_FATAL(_device->CreateGeometryShader(buf.data(), buf.size(), NULL, &shader));
    _geometryShaders.Update(handle, shader);

    END_INIT_SEQUENCE();
  });

  return res.initialResult ? handle : ObjectHandle();
}

//------------------------------------------------------------------------------
ObjectHandle Graphics::LoadComputeShaderFromFile(const string& filenameBase, const char* entry)
{
  ObjectHandle handle = ReserveObjectHandle(ObjectHandle::kComputeShader);

#if WITH_DEBUG_SHADERS
  string filename = filenameBase + ToString("_%sD.cso", entry);
#else
  string filename = filenameBase + ToString("_%s.cso", entry);
#endif

  AddFileWatchResult res = RESOURCE_MANAGER.AddFileWatch(filename.c_str(), true, [=](const string& filename, void*)
  {
    BEGIN_INIT_SEQUENCE();

    vector<char> buf;
    ID3D11ComputeShader* shader = nullptr;

    INIT_FATAL(RESOURCE_MANAGER.LoadFile(filename.c_str(), &buf));
    INIT_HR_FATAL(_device->CreateComputeShader(buf.data(), buf.size(), NULL, &shader));
    _computeShaders.Update(handle, shader);

    END_INIT_SEQUENCE();
  });

  return res.initialResult ? handle : ObjectHandle();
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
  if (vs)
    *vs = LoadVertexShaderFromFile(filenameBase, vsEntry, inputLayout, elements);

  if (gs)
    *gs = LoadGeometryShaderFromFile(filenameBase, gsEntry);

  if (ps)
    *ps = LoadPixelShaderFromFile(filenameBase, psEntry);

  return (!vs || vs->IsValid()) && (!ps || ps->IsValid()) && (!gs || gs->IsValid());
#if 0
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
    ObjectHandle vsHandle = ReserveObjectHandle(ObjectHandle::kVertexShader);
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
    ObjectHandle gsHandle = ReserveObjectHandle(ObjectHandle::kGeometryShader);
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
    ObjectHandle psHandle = ReserveObjectHandle(ObjectHandle::kPixelShader);
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
#endif
}
