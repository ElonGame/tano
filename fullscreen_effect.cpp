#include "fullscreen_effect.hpp"
#include "graphics_context.hpp"
#include "init_sequence.hpp"

using namespace tano;

//------------------------------------------------------------------------------
FullscreenEffect::FullscreenEffect(GraphicsContext* ctx)
  : _ctx(ctx)
{
}

//------------------------------------------------------------------------------
bool FullscreenEffect::Init()
{
  BEGIN_INIT_SEQUENCE();

  INIT(_cb.Create());
  INIT(_cbScaleBias.Create());
  INIT(_cbBlur.Create());

  INIT(_defaultBundle.Create(BundleOptions()
    .DepthStencilDesc(depthDescDepthDisabled)
    .RasterizerDesc(rasterizeDescCullNone)
    .VertexShader("shaders/out/quad", "VsMain")));

  INIT(_scaleBiasBundle.Create(BundleOptions()
    .DepthStencilDesc(depthDescDepthDisabled)
    .RasterizerDesc(rasterizeDescCullNone)
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/common", "PsScaleBias")));

  INIT(_scaleBiasSecondaryBundle.Create(BundleOptions()
    .DepthStencilDesc(depthDescDepthDisabled)
    .RasterizerDesc(rasterizeDescCullNone)
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/common", "PsScaleBiasSecondary")));

  INIT(_copyBundle.Create(BundleOptions()
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/common", "PsCopy")));

  INIT(_addBundle.Create(BundleOptions()
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/common", "PsAdd")));

  // blur setup
  INIT_RESOURCE(_csBlurX, GRAPHICS.LoadComputeShaderFromFile("shaders/out/blur", "BoxBlurX"));

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
void FullscreenEffect::Execute(
  ObjectHandle input,
  ObjectHandle output,
  const RenderTargetDesc& outputDesc,
  ObjectHandle depthStencil,
  ObjectHandle shader,
  bool releaseOutput,
  const Color* clearColor)
{
  Execute(&input, 1, output, outputDesc, depthStencil, shader, releaseOutput, clearColor);
}

//------------------------------------------------------------------------------
void FullscreenEffect::Execute(
  const ObjectHandle* inputs,
  int numInputs,
  ObjectHandle output,
  const RenderTargetDesc& outputDesc,
  ObjectHandle depthStencil,
  ObjectHandle shader,
  bool releaseOutput,
  const Color* clearColor)
{
  assert(output.IsValid());

  _ctx->SetLayout(ObjectHandle());
  _ctx->SetGpuState(_defaultBundle.state);
  _ctx->SetGpuStateSamplers(_defaultBundle.state, ShaderType::PixelShader);
  _ctx->SetGpuStateSamplers(_defaultBundle.state, ShaderType::ComputeShader);
  _ctx->SetGpuObjects(_defaultBundle.objects);

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
void FullscreenEffect::Copy(
  ObjectHandle inputBuffer,
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
  ObjectHandle input,
  ObjectHandle output,
  const RenderTargetDesc& outputDesc,
  float scale, 
  float bias)
{
  assert(output.IsValid());

  _ctx->SetLayout(ObjectHandle());
  _ctx->SetBundleWithSamplers(_scaleBiasBundle, ShaderType::PixelShader);

  _ctx->SetRenderTarget(output, ObjectHandle(), nullptr);

  _ctx->SetShaderResource(input, ShaderType::PixelShader);
  _cbScaleBias.scaleBias = Vector4(scale, bias, 0, 0);
  _ctx->SetConstantBuffer(_cbScaleBias, ShaderType::PixelShader, 1);

  _ctx->SetViewports(1, ViewportFromDesc(outputDesc));

  _ctx->Draw(6, 0);

  _ctx->UnsetRenderTargets(0, 1);
  _ctx->UnsetShaderResources(0, 1, ShaderType::PixelShader);
}

//------------------------------------------------------------------------------
void FullscreenEffect::ScaleBiasSecondary(
  ObjectHandle input0,
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

  ObjectHandle inputs[] = { input0, input1 };
  _ctx->SetShaderResources(inputs, 2, ShaderType::PixelShader);
  _cbScaleBias.scaleBias = Vector4(scale, bias, 0, 0);
  _ctx->SetConstantBuffer(_cbScaleBias, ShaderType::PixelShader, 1);

  _ctx->SetViewports(1, ViewportFromDesc(outputDesc));

  _ctx->Draw(6, 0);

  _ctx->UnsetRenderTargets(0, 1);
  _ctx->UnsetShaderResources(0, 2, ShaderType::PixelShader);
}

//------------------------------------------------------------------------------
void FullscreenEffect::Blur(ObjectHandle input, ObjectHandle output, float radius)
{
  static Color black(0, 0, 0, 0);

  int w, h;
  GRAPHICS.GetBackBufferSize(&w, &h);
  int s = max(w, h);

  BufferFlags f = BufferFlags(BufferFlag::CreateSrv | BufferFlag::CreateUav);
  ScopedRenderTarget scratch0(s * 4, s, DXGI_FORMAT_R32_FLOAT, f);
  ScopedRenderTarget scratch1(s * 4, s, DXGI_FORMAT_R32_FLOAT, f);
  ScopedRenderTarget scratch2(s, s, DXGI_FORMAT_R16G16B16A16_FLOAT, f);

  // set constant buffers
  _cbBlur.inputSize.x = (float)w;
  _cbBlur.inputSize.y = (float)h;
  _cbBlur.radius = radius;
  _ctx->SetConstantBuffer(_cbBlur, ShaderType::ComputeShader, 0);

  int numThreads = 256;

  // horizontal
  _ctx->SetShaderResource(input, ShaderType::ComputeShader);
  ObjectHandle inputViews[] = { scratch0, scratch1, scratch2 };
  _ctx->SetUnorderedAccessViews(inputViews, 3, nullptr);

  _ctx->SetComputeShader(_csBlurX);
  _ctx->Dispatch(h / numThreads + 1, 1, 1);

  _ctx->UnsetUnorderedAccessViews(0, 3);
  _ctx->UnsetShaderResources(0, 1, ShaderType::ComputeShader);

  // vertical
  _cbBlur.inputSize.x = (float)h;
  _cbBlur.inputSize.y = (float)w;
  _ctx->SetConstantBuffer(_cbBlur, ShaderType::ComputeShader, 0);

  _ctx->SetShaderResource(scratch2._rtHandle, ShaderType::ComputeShader);
  ObjectHandle inputViews2[] = { scratch0, scratch1, output };
  _ctx->SetUnorderedAccessViews(inputViews2, 3, nullptr);

  _ctx->SetComputeShader(_csBlurX);
  _ctx->Dispatch(w / numThreads + 1, 1, 1);

  _ctx->UnsetUnorderedAccessViews(0, 3);
  _ctx->UnsetShaderResources(0, 1, ShaderType::ComputeShader);
}
