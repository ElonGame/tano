#include "fullscreen_effect.hpp"
#include "graphics_context.hpp"
#include "init_sequence.hpp"

using namespace tano;

//------------------------------------------------------------------------------
FullscreenEffect::FullscreenEffect(GraphicsContext* ctx) : _ctx(ctx)
{
}

//------------------------------------------------------------------------------
bool FullscreenEffect::Init()
{
  BEGIN_INIT_SEQUENCE();

  INIT(_cb.Create());
  INIT(_cbScaleBias.Create());
  INIT(_cbBlur.Create());

  // clang-format off
  INIT(_defaultBundle.Create(BundleOptions()
  .DepthStencilDesc(depthDescDepthDisabled)
  .RasterizerDesc(rasterizeDescCullNone)
  .VertexShader("shaders/out/quad", "VsMain")));

  INIT(_scaleBiasBundle.Create(BundleOptions()
    .DepthStencilDesc(depthDescDepthDisabled)
    .RasterizerDesc(rasterizeDescCullNone)
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/common.scale", "PsScaleBias")));

  INIT(_scaleBiasSecondaryBundle.Create(BundleOptions()
    .DepthStencilDesc(depthDescDepthDisabled)
    .RasterizerDesc(rasterizeDescCullNone)
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/common.scale", "PsScaleBiasSecondary")));

  INIT(_copyBundle.Create(BundleOptions()
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/common", "PsCopy")));

  INIT(_addBundle.Create(BundleOptions()
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/common", "PsAdd")));

  INIT(_renderTextureBundle.Create(BundleOptions()
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/common", "PsCopy")));

  // clang-format on

  // blur setup
  INIT_RESOURCE(_csBlurTranspose, GRAPHICS.LoadComputeShaderFromFile("shaders/out/blur", "BlurTranspose"));
  INIT_RESOURCE(_csCopyTranspose, GRAPHICS.LoadComputeShaderFromFile("shaders/out/blur", "CopyTranspose"));
  INIT_RESOURCE(_csBlurX, GRAPHICS.LoadComputeShaderFromFile("shaders/out/blur", "BoxBlurX"));
  INIT_RESOURCE(_csBlurY, GRAPHICS.LoadComputeShaderFromFile("shaders/out/blur", "BoxBlurY"));

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
void FullscreenEffect::Execute(ObjectHandle input,
    ObjectHandle output,
    const RenderTargetDesc& outputDesc,
    ObjectHandle depthStencil,
    ObjectHandle shader,
    bool releaseOutput,
    bool setBundle,
    const Color* clearColor)
{
  Execute(&input, 1, output, outputDesc, depthStencil, shader, releaseOutput, setBundle, clearColor);
}

//------------------------------------------------------------------------------
void FullscreenEffect::Execute(const ObjectHandle* inputs,
    int numInputs,
    ObjectHandle output,
    const RenderTargetDesc& outputDesc,
    ObjectHandle depthStencil,
    ObjectHandle shader,
    bool releaseOutput,
    bool setBundle,
    const Color* clearColor)
{
  assert(output.IsValid());

  _ctx->SetLayout(ObjectHandle());

  if (setBundle)
  {
    _ctx->SetGpuState(_defaultBundle.state);
    _ctx->SetGpuStateSamplers(_defaultBundle.state, ShaderType::PixelShader);
    _ctx->SetGpuStateSamplers(_defaultBundle.state, ShaderType::ComputeShader);
    _ctx->SetGpuObjects(_defaultBundle.objects);
  }
  else
  {
    _ctx->SetVertexShader(_defaultBundle.objects._vs);
  }

  _ctx->SetRenderTarget(output, depthStencil, clearColor);
  _ctx->SetShaderResources(inputs, numInputs, ShaderType::PixelShader);

  CD3D11_VIEWPORT viewport = CD3D11_VIEWPORT(0.f, 0.f, (float)outputDesc.width, (float)outputDesc.height);
  _ctx->SetViewports(1, viewport);

  _ctx->SetPixelShader(shader);
  _ctx->Draw(6, 0);

  if (releaseOutput)
    _ctx->UnsetRenderTargets(0, 1);

  _ctx->UnsetShaderResources(0, numInputs, ShaderType::PixelShader);
}

//------------------------------------------------------------------------------
void FullscreenEffect::Copy(ObjectHandle inputBuffer,
    ObjectHandle outputBuffer,
    const RenderTargetDesc& outputDesc,
    bool releaseOutput)
{
  Execute(inputBuffer, outputBuffer, outputDesc, ObjectHandle(), _copyBundle.objects._ps, releaseOutput);
}

//------------------------------------------------------------------------------
inline CD3D11_VIEWPORT ViewportFromDesc(const RenderTargetDesc& desc)
{
  return CD3D11_VIEWPORT(0.f, 0.f, (float)desc.width, (float)desc.height);
}

//------------------------------------------------------------------------------
void FullscreenEffect::ScaleBias(
    ObjectHandle input, ObjectHandle output, const RenderTargetDesc& outputDesc, float scale, float bias)
{
  assert(output.IsValid());

  _ctx->SetLayout(ObjectHandle());
  _ctx->SetBundleWithSamplers(_scaleBiasBundle, ShaderType::PixelShader);

  _ctx->SetRenderTarget(output, ObjectHandle(), nullptr);

  _ctx->SetShaderResource(input, ShaderType::PixelShader);
  _cbScaleBias.scaleBias = Vector4(scale, bias, 0, 0);
  _ctx->SetConstantBuffer(_cbScaleBias, ShaderType::PixelShader, 0);

  _ctx->SetViewports(1, ViewportFromDesc(outputDesc));

  _ctx->Draw(6, 0);

  _ctx->UnsetRenderTargets(0, 1);
  _ctx->UnsetShaderResources(0, 1, ShaderType::PixelShader);
}

//------------------------------------------------------------------------------
void FullscreenEffect::ScaleBiasSecondary(ObjectHandle input0,
    ObjectHandle input1,
    ObjectHandle output,
    const RenderTargetDesc& outputDesc,
    float scale,
    float bias)
{
  assert(output.IsValid());

  _ctx->SetLayout(ObjectHandle());
  _ctx->SetBundleWithSamplers(_scaleBiasSecondaryBundle, ShaderType::PixelShader);

  _ctx->SetRenderTarget(output, ObjectHandle(), nullptr);

  ObjectHandle inputs[] = {input0, input1};
  _ctx->SetShaderResources(inputs, 2, ShaderType::PixelShader);
  _cbScaleBias.scaleBias = Vector4(scale, bias, 0, 0);
  _ctx->SetConstantBuffer(_cbScaleBias, ShaderType::PixelShader, 0);

  _ctx->SetViewports(1, ViewportFromDesc(outputDesc));

  _ctx->Draw(6, 0);

  _ctx->UnsetRenderTargets(0, 1);
  _ctx->UnsetShaderResources(0, 2, ShaderType::PixelShader);
}

//------------------------------------------------------------------------------
void FullscreenEffect::Blur(
    ObjectHandle input, ObjectHandle output, const RenderTargetDesc& outputDesc, float radius, int scale)
{
  int w = outputDesc.width / scale;
  int h = outputDesc.height / scale;
  BufferFlags f = BufferFlags(BufferFlag::CreateSrv | BufferFlag::CreateUav);

  ObjectHandle scaledRt;
  if (scale > 1)
  {
    scaledRt = GRAPHICS.GetTempRenderTarget(w, h, DXGI_FORMAT_R16G16B16A16_FLOAT, f);
    Copy(input, scaledRt, RenderTargetDesc(w, h, DXGI_FORMAT_R16G16B16A16_FLOAT), true);
  }

  ScopedRenderTarget hscratch0(w, h, DXGI_FORMAT_R16G16B16A16_FLOAT, f);
  ScopedRenderTarget hscratch1(w, h, DXGI_FORMAT_R16G16B16A16_FLOAT, f);

  ScopedRenderTarget vscratch0(h, w, DXGI_FORMAT_R16G16B16A16_FLOAT, f);
  ScopedRenderTarget vscratch1(h, w, DXGI_FORMAT_R16G16B16A16_FLOAT, f);

  // set constant buffers
  _cbBlur.inputSize.x = (float)w;
  _cbBlur.inputSize.y = (float)h;
  _cbBlur.radius = radius;
  _ctx->SetConstantBuffer(_cbBlur, ShaderType::ComputeShader, 0);

  // set constant buffers
  ObjectHandle srcDst[] = {
      // horiz
      input,
      hscratch0,
      hscratch0,
      hscratch1,
      hscratch1,
      vscratch0,
      // vert
      vscratch0,
      vscratch1,
      vscratch1,
      vscratch0,
      vscratch0,
      output,
  };

  if (scale > 1)
  {
    srcDst[0] = scaledRt;
    srcDst[11] = scaledRt;
  }

  int numThreads = 256;

  // horizontal blur (ends up in scratch0)
  for (int i = 0; i < 3; ++i)
  {
    _ctx->SetShaderResource(srcDst[i * 2 + 0], ShaderType::ComputeShader);
    _ctx->SetUnorderedAccessView(srcDst[i * 2 + 1], nullptr);

    _ctx->SetComputeShader(i < 2 ? _csBlurX : _csBlurTranspose);
    _ctx->Dispatch(h / numThreads + 1, 1, 1);

    _ctx->UnsetUnorderedAccessViews(0, 1);
    _ctx->UnsetShaderResources(0, 1, ShaderType::ComputeShader);
  }

  // "vertical" blur, ends up in scratch 0
  _cbBlur.inputSize.x = (float)h;
  _cbBlur.inputSize.y = (float)w;
  _ctx->SetConstantBuffer(_cbBlur, ShaderType::ComputeShader, 0);

  for (int i = 0; i < 3; ++i)
  {
    _ctx->SetShaderResources(&srcDst[6 + i * 2 + 0], 1, ShaderType::ComputeShader);
    _ctx->SetUnorderedAccessView(srcDst[6 + i * 2 + 1], nullptr);

    _ctx->SetComputeShader(i < 2 ? _csBlurX : _csBlurTranspose);
    _ctx->Dispatch(w / numThreads + 1, 1, 1);

    _ctx->UnsetUnorderedAccessViews(0, 1);
    _ctx->UnsetShaderResources(0, 1, ShaderType::ComputeShader);
  }

  if (scale > 1)
  {
    Copy(scaledRt, output, outputDesc, true);
    GRAPHICS.ReleaseTempRenderTarget(scaledRt);
  }
}

//------------------------------------------------------------------------------
void FullscreenEffect::BlurHoriz(
    ObjectHandle input, ObjectHandle output, const RenderTargetDesc& outputDesc, float radius, int scale)
{
  int w = outputDesc.width / scale;
  int h = outputDesc.height / scale;
  BufferFlags f = BufferFlags(BufferFlag::CreateSrv | BufferFlag::CreateUav);

  ObjectHandle scaledRt;
  if (scale > 1)
  {
    scaledRt = GRAPHICS.GetTempRenderTarget(w, h, DXGI_FORMAT_R16G16B16A16_FLOAT, f);
    Copy(input, scaledRt, RenderTargetDesc(w, h, DXGI_FORMAT_R16G16B16A16_FLOAT), true);
  }

  ScopedRenderTarget scratch0(w, h, DXGI_FORMAT_R16G16B16A16_FLOAT, f);
  ScopedRenderTarget scratch1(w, h, DXGI_FORMAT_R16G16B16A16_FLOAT, f);

  // set constant buffers
  _cbBlur.inputSize.x = (float)w;
  _cbBlur.inputSize.y = (float)h;
  _cbBlur.radius = radius;
  _ctx->SetConstantBuffer(_cbBlur, ShaderType::ComputeShader, 0);

  // set constant buffers
  ObjectHandle srcDst[] = {
      // horiz
      input,
      scratch0,
      scratch0,
      scratch1,
      scratch1,
      output,
  };

  if (scale > 1)
  {
    srcDst[0] = scaledRt;
  }

  int numThreads = 256;

  // horizontal blur (ends up in output)
  for (int i = 0; i < 3; ++i)
  {
    _ctx->SetShaderResource(srcDst[i * 2 + 0], ShaderType::ComputeShader);
    _ctx->SetUnorderedAccessView(srcDst[i * 2 + 1], nullptr);

    _ctx->SetComputeShader(_csBlurX);
    _ctx->Dispatch(h / numThreads + 1, 1, 1);

    _ctx->UnsetUnorderedAccessViews(0, 1);
    _ctx->UnsetShaderResources(0, 1, ShaderType::ComputeShader);
  }
}

//------------------------------------------------------------------------------
void FullscreenEffect::BlurVertCustom(ObjectHandle input,
    ObjectHandle output,
    const RenderTargetDesc& outputDesc,
    ObjectHandle shader,
    float radius,
    int scale)
{
  int w = outputDesc.width / scale;
  int h = outputDesc.height / scale;
  BufferFlags f = BufferFlags(BufferFlag::CreateSrv | BufferFlag::CreateUav);

  ObjectHandle scaledRt;
  if (scale > 1)
  {
    scaledRt = GRAPHICS.GetTempRenderTarget(w, h, DXGI_FORMAT_R16G16B16A16_FLOAT, f);
    Copy(input, scaledRt, RenderTargetDesc(w, h, DXGI_FORMAT_R16G16B16A16_FLOAT), true);
  }

  ScopedRenderTarget scratch0(w, h, DXGI_FORMAT_R16G16B16A16_FLOAT, f);
  ScopedRenderTarget scratch1(w, h, DXGI_FORMAT_R16G16B16A16_FLOAT, f);

  // set constant buffers
  _cbBlur.inputSize.x = (float)w;
  _cbBlur.inputSize.y = (float)h;
  _cbBlur.radius = radius;
  _ctx->SetConstantBuffer(_cbBlur, ShaderType::ComputeShader, 0);

  // set constant buffers
  ObjectHandle srcDst[] = {
      input, scratch0, scratch0, scratch1, scratch1, output,
  };

  if (scale > 1)
  {
    srcDst[0] = scaledRt;
  }

  int numThreads = 256;

  for (int i = 0; i < 3; ++i)
  {
    _ctx->SetShaderResource(srcDst[i * 2 + 0], ShaderType::ComputeShader);
    _ctx->SetUnorderedAccessView(srcDst[i * 2 + 1], nullptr);

    _ctx->SetComputeShader(shader);
    _ctx->Dispatch(w / numThreads + 1, 1, 1);

    _ctx->UnsetUnorderedAccessViews(0, 1);
    _ctx->UnsetShaderResources(0, 1, ShaderType::ComputeShader);
  }
}

//------------------------------------------------------------------------------
void FullscreenEffect::BlurVert(
    ObjectHandle input, ObjectHandle output, const RenderTargetDesc& outputDesc, float radius, int scale)
{
  BlurVertCustom(input, output, outputDesc, _csBlurY, radius, scale);
}

//------------------------------------------------------------------------------
void FullscreenEffect::RenderTexture(ObjectHandle texture,
  const vec2& topLeft,
  const vec2& bottomRight)
{
}
