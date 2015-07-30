#include "gpu_objects.hpp"
#include "graphics.hpp"
#include "graphics_context.hpp"
#include "init_sequence.hpp"

using namespace tano;
using namespace bristol;

//------------------------------------------------------------------------------
bool GpuObjects::CreateDynamic(u32 ibSize, DXGI_FORMAT ibFormat, u32 vbSize, u32 vbElemSize)
{
  return CreateDynamicIb(ibSize, ibFormat, nullptr) && CreateDynamicVb(vbSize, vbElemSize, nullptr);
}

//------------------------------------------------------------------------------
bool GpuObjects::CreateDynamic(
    u32 ibSize, DXGI_FORMAT ibFormat, const void* ibData, u32 vbSize, u32 vbElemSize, const void* vbData)
{
  return CreateDynamicIb(ibSize, ibFormat, ibData) && CreateDynamicVb(vbSize, vbElemSize, vbData);
}

//------------------------------------------------------------------------------
bool GpuObjects::CreateDynamicVb(u32 vbSize, u32 vbElemSize, const void* vbData)
{
  _vbSize = vbSize;
  _vbElemSize = vbElemSize;
  _numVerts = _vbSize / _vbElemSize;
  _vb = GRAPHICS.CreateBuffer(D3D11_BIND_VERTEX_BUFFER, vbSize, true, vbData, vbElemSize);
  return _vb.IsValid();
}

//------------------------------------------------------------------------------
bool GpuObjects::CreateDynamicIb(u32 ibSize, DXGI_FORMAT ibFormat, const void* ibData)
{
  _ibSize = ibSize;
  _ibFormat = ibFormat;
  _numIndices = _ibSize / IndexSizeFromFormat(ibFormat);
  _ib = GRAPHICS.CreateBuffer(D3D11_BIND_INDEX_BUFFER, ibSize, true, ibData, ibFormat);
  return _ib.IsValid();
}

//------------------------------------------------------------------------------
bool GpuObjects::CreateVertexBuffer(u32 vbSize, u32 vbElemSize, const void* vbData)
{
  _vbSize = vbSize;
  _vbElemSize = vbElemSize;
  _numVerts = _vbSize / _vbElemSize;
  _vb = GRAPHICS.CreateBuffer(D3D11_BIND_VERTEX_BUFFER, vbSize, false, vbData, vbElemSize);
  return _vb.IsValid();
}

//------------------------------------------------------------------------------
bool GpuObjects::CreateIndexBuffer(u32 ibSize, DXGI_FORMAT ibFormat, const void* ibData)
{
  _ibSize = ibSize;
  _ibFormat = ibFormat;
  _numIndices = _ibSize / IndexSizeFromFormat(ibFormat);
  _ib = GRAPHICS.CreateBuffer(D3D11_BIND_INDEX_BUFFER, ibSize, false, ibData, ibFormat);
  return _ib.IsValid();
}

//------------------------------------------------------------------------------
bool GpuObjects::LoadVertexShader(
    const char* filename, const char* entryPoint, u32 flags, vector<D3D11_INPUT_ELEMENT_DESC>* elements)
{
  BEGIN_INIT_SEQUENCE();
  ObjectHandle* layout = (flags || elements) ? &_layout : nullptr;

  if (flags)
  {
    vector<D3D11_INPUT_ELEMENT_DESC> desc;
    VertexFlagsToLayoutDesc(flags, &desc);
    INIT_RESOURCE(_vs, GRAPHICS.LoadVertexShaderFromFile(filename, entryPoint, &_layout, &desc));
  }
  else if (elements)
  {
    // fix up the offsets on the elements
    u32 ofs = 0;
    unordered_map<string, u32> semanticIndex;
    for (D3D11_INPUT_ELEMENT_DESC& e : *elements)
    {
      e.AlignedByteOffset = ofs;
      e.SemanticIndex = semanticIndex[e.SemanticName]++;
      ofs += SizeFromFormat(e.Format);
    }
    INIT_RESOURCE(_vs, GRAPHICS.LoadVertexShaderFromFile(filename, entryPoint, &_layout, elements));
  }
  else
  {
    INIT_RESOURCE(_vs, GRAPHICS.LoadVertexShaderFromFile(filename, entryPoint, nullptr, nullptr));
  }

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool GpuObjects::LoadPixelShader(const char* filename, const char* entryPoint)
{
  _ps = GRAPHICS.LoadPixelShaderFromFile(filename, entryPoint);
  return _ps.IsValid();
}

//------------------------------------------------------------------------------
bool GpuObjects::LoadGeometryShader(const char* filename, const char* entryPoint)
{
  _gs = GRAPHICS.LoadGeometryShaderFromFile(filename, entryPoint);
  return _gs.IsValid();
}

//------------------------------------------------------------------------------
bool GpuState::Create(const D3D11_DEPTH_STENCIL_DESC* dssDesc,
    const D3D11_BLEND_DESC* blendDesc,
    const D3D11_RASTERIZER_DESC* rasterizerDesc)
{
  _depthStencilState =
      GRAPHICS.CreateDepthStencilState(dssDesc ? *dssDesc : CD3D11_DEPTH_STENCIL_DESC(CD3D11_DEFAULT()));

  _blendState = GRAPHICS.CreateBlendState(blendDesc ? *blendDesc : CD3D11_BLEND_DESC(CD3D11_DEFAULT()));

  _rasterizerState = GRAPHICS.CreateRasterizerState(
      rasterizerDesc ? *rasterizerDesc : CD3D11_RASTERIZER_DESC(CD3D11_DEFAULT()));

  CD3D11_SAMPLER_DESC samplerDesc = CD3D11_SAMPLER_DESC(CD3D11_DEFAULT());
  _samplers[Linear] = GRAPHICS.CreateSamplerState(samplerDesc);

  samplerDesc.AddressU = samplerDesc.AddressV = D3D11_TEXTURE_ADDRESS_WRAP;
  _samplers[LinearWrap] = GRAPHICS.CreateSamplerState(samplerDesc);

  samplerDesc.AddressU = samplerDesc.AddressV = D3D11_TEXTURE_ADDRESS_BORDER;
  _samplers[LinearBorder] = GRAPHICS.CreateSamplerState(samplerDesc);

  samplerDesc.AddressU = samplerDesc.AddressV = D3D11_TEXTURE_ADDRESS_CLAMP;
  samplerDesc.Filter = D3D11_FILTER_MIN_MAG_MIP_POINT;
  _samplers[Point] = GRAPHICS.CreateSamplerState(samplerDesc);

  return _depthStencilState.IsValid() && _blendState.IsValid() && _rasterizerState.IsValid() &&
         _samplers[0].IsValid() && _samplers[1].IsValid() && _samplers[2].IsValid() && _samplers[3].IsValid();
}

//------------------------------------------------------------------------------
bool GpuBundle::Create(const BundleOptions& options)
{
  const BundleOptions::OptionFlags& flags = options.flags;
  BEGIN_INIT_SEQUENCE();
  INIT_FATAL(state.Create(
      flags.IsSet(BundleOptions::OptionFlag::DepthStencilDesc) ? &options.depthStencilDesc : nullptr,
      flags.IsSet(BundleOptions::OptionFlag::BlendDesc) ? &options.blendDesc : nullptr,
      flags.IsSet(BundleOptions::OptionFlag::RasterizerDesc) ? &options.rasterizerDesc : nullptr));

  objects._topology = options.topology;

  if (options.vsEntry)
  {
    vector<D3D11_INPUT_ELEMENT_DESC> inputElements(options.inputElements);
    // clang-format off
    INIT_FATAL_LOG(objects.LoadVertexShader(
      options.vsShaderFile,
      options.vsEntry,
      options.vertexFlags,
      inputElements.empty() ? nullptr : &inputElements),
      "Error loading vertex shader: ", options.vsShaderFile, ":", options.vsEntry);
    // clang-format on
  }

  if (options.psEntry)
  {
    INIT_FATAL(objects.LoadPixelShader(options.psShaderFile, options.psEntry));
  }

  if (options.gsEntry)
  {
    INIT_FATAL(objects.LoadGeometryShader(options.gsShaderFile, options.gsEntry));
  }

  if (flags.IsSet(BundleOptions::OptionFlag::DynamicVb))
  {
    INIT_FATAL(objects.CreateDynamicVb(options.vbNumElems * options.vbElemSize, options.vbElemSize));
  }

  if (flags.IsSet(BundleOptions::OptionFlag::DynamicIb))
  {
    INIT_FATAL(objects.CreateDynamicIb(options.ibNumElems * options.ibElemSize, DXGI_FORMAT_R32_UINT));
  }

  if (flags.IsSet(BundleOptions::OptionFlag::StaticVb))
  {
    INIT_FATAL(objects.CreateVertexBuffer(
        options.vbNumElems * options.vbElemSize, options.vbElemSize, options.staticVb));
  }

  if (flags.IsSet(BundleOptions::OptionFlag::StaticIb))
  {
    INIT_FATAL(objects.CreateIndexBuffer(options.ibNumElems * options.ibElemSize,
        options.ibElemSize == 2 ? DXGI_FORMAT_R16_UINT : DXGI_FORMAT_R32_UINT,
        options.staticIb));
  }

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
BundleOptions::BundleOptions()
{
  blendDesc.IndependentBlendEnable = TRUE;
}

//------------------------------------------------------------------------------
BundleOptions& BundleOptions::DepthStencilDesc(const CD3D11_DEPTH_STENCIL_DESC& desc)
{
  depthStencilDesc = desc;
  flags.Set(BundleOptions::OptionFlag::DepthStencilDesc);
  return *this;
}

//------------------------------------------------------------------------------
BundleOptions& BundleOptions::BlendDesc(const CD3D11_BLEND_DESC& desc)
{
  blendDesc = desc;
  flags.Set(BundleOptions::OptionFlag::BlendDesc);
  return *this;
}

//------------------------------------------------------------------------------
BundleOptions& BundleOptions::RasterizerDesc(const CD3D11_RASTERIZER_DESC& desc)
{
  rasterizerDesc = desc;
  flags.Set(BundleOptions::OptionFlag::RasterizerDesc);
  return *this;
}

//------------------------------------------------------------------------------
BundleOptions& BundleOptions::VertexShader(const char* filename, const char* entrypoint)
{
  vsShaderFile = filename;
  vsEntry = entrypoint;
  return *this;
}

//------------------------------------------------------------------------------
BundleOptions& BundleOptions::PixelShader(const char* filename, const char* entrypoint)
{
  psShaderFile = filename;
  psEntry = entrypoint;
  return *this;
}

//------------------------------------------------------------------------------
BundleOptions& BundleOptions::GeometryShader(const char* filename, const char* entrypoint)
{
  gsShaderFile = filename;
  gsEntry = entrypoint;
  return *this;
}

//------------------------------------------------------------------------------
BundleOptions& BundleOptions::ComputeShader(const char* filename, const char* entrypoint)
{
  csShaderFile = filename;
  csEntry = entrypoint;
  return *this;
}

//------------------------------------------------------------------------------
BundleOptions& BundleOptions::VertexFlags(u32 flags)
{
  vertexFlags = flags;
  return *this;
}

//------------------------------------------------------------------------------
BundleOptions& BundleOptions::Topology(D3D11_PRIMITIVE_TOPOLOGY topologyIn)
{
  topology = topologyIn;
  return *this;
}

//------------------------------------------------------------------------------
BundleOptions& BundleOptions::InputElement(const CD3D11_INPUT_ELEMENT_DESC& elem)
{
  inputElements.push_back(elem);
  return *this;
}

//------------------------------------------------------------------------------
BundleOptions& BundleOptions::InputElements(const vector<D3D11_INPUT_ELEMENT_DESC>& elems)
{
  inputElements = elems;
  return *this;
}

//------------------------------------------------------------------------------
BundleOptions& BundleOptions::DynamicVb(int numElements, int elementSize)
{
  flags.Set(BundleOptions::OptionFlag::DynamicVb);
  vbNumElems = numElements;
  vbElemSize = elementSize;
  return *this;
}

//------------------------------------------------------------------------------
BundleOptions& BundleOptions::DynamicIb(int numElements, int elementSize)
{
  flags.Set(BundleOptions::OptionFlag::DynamicIb);
  ibNumElems = numElements;
  ibElemSize = elementSize;
  return *this;
}

//------------------------------------------------------------------------------
BundleOptions& BundleOptions::StaticVb(int numElements, int elementSize, void* data)
{
  flags.Set(BundleOptions::OptionFlag::StaticVb);
  vbNumElems = numElements;
  vbElemSize = elementSize;
  staticVb = data;
  return *this;
}

//------------------------------------------------------------------------------
BundleOptions& BundleOptions::StaticIb(int numElements, int elementSize, void* data)
{
  flags.Set(BundleOptions::OptionFlag::StaticIb);
  ibNumElems = numElements;
  ibElemSize = elementSize;
  staticIb = data;
  return *this;
}
