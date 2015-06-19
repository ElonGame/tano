#include "post_process.hpp"
#include "graphics_context.hpp"
#include "init_sequence.hpp"

using namespace tano;

//------------------------------------------------------------------------------
PostProcess::PostProcess(GraphicsContext* ctx)
  : _ctx(ctx)
{
}

//------------------------------------------------------------------------------
bool PostProcess::Init()
{
  BEGIN_INIT_SEQUENCE();

  INIT(_cb.Create());
  INIT(_cbScaleBias.Create());

  INIT(_defaultBundle.Create(BundleOptions()
    .DepthStencilDesc(depthDescDepthDisabled)
    .RasterizerDesc(rasterizeDescCullNone)
    .ShaderFile("shaders/out/quad")
    .VsEntry("VsMain")));

  INIT(_defaultBundle.Create(BundleOptions()
    .DepthStencilDesc(depthDescDepthDisabled)
    .RasterizerDesc(rasterizeDescCullNone)
    .ShaderFile("shaders/out/common")
    .VsEntry("VsQuad")
    .PsEntry("PsScaleBias")));

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
void PostProcess::Execute(
    const vector<ObjectHandle>& input,
    ObjectHandle output,
    ObjectHandle depthStencil,
    ObjectHandle shader,
    bool releaseOutput,
    const Color* clearColor)
{
  Execute(input.data(), (int)input.size(), output, depthStencil, shader, releaseOutput, clearColor);
}

//------------------------------------------------------------------------------
void PostProcess::Execute(
  ObjectHandle input,
  ObjectHandle output,
  ObjectHandle depthStencil,
  ObjectHandle shader,
  bool releaseOutput,
  const Color* clearColor)
{
  Execute(&input, 1, output, depthStencil, shader, releaseOutput, clearColor);
}

//------------------------------------------------------------------------------
void PostProcess::Execute(
  const ObjectHandle* inputs,
  int numInputs,
  ObjectHandle output,
  ObjectHandle depthStencil,
  ObjectHandle shader,
  bool releaseOutput,
  const Color* clearColor)
{
  _ctx->SetLayout(ObjectHandle());
  _ctx->SetGpuState(_defaultBundle.state);
  _ctx->SetGpuStateSamplers(_defaultBundle.state, ShaderType::PixelShader);
  _ctx->SetGpuStateSamplers(_defaultBundle.state, ShaderType::ComputeShader);
  _ctx->SetGpuObjects(_defaultBundle.objects);

  assert(output.IsValid());
  _ctx->SetRenderTarget(output, depthStencil, clearColor);

  _ctx->SetShaderResources(inputs, numInputs, ShaderType::PixelShader);

  // TODO: This should really be moved in an input parameter
  u32 outputX, outputY;
  GRAPHICS.GetTextureSize(output, &outputX, &outputY);

  CD3D11_VIEWPORT viewport = CD3D11_VIEWPORT(0.f, 0.f, (float)outputX, (float)outputY);
  _ctx->SetViewports(1, viewport);

  _ctx->SetPixelShader(shader);
  _ctx->Draw(6, 0);

  if (releaseOutput)
    _ctx->UnsetRenderTargets(0, 1);

  _ctx->UnsetShaderResources(0, numInputs, ShaderType::PixelShader);
}

//------------------------------------------------------------------------------
void PostProcess::ScaleBias(ObjectHandle input, ObjectHandle output, float scale, float bias)
{
  _ctx->SetLayout(ObjectHandle());
  _ctx->SetBundleWithSamplers(_scaleBiasBundle, ShaderType::PixelShader);

  assert(output.IsValid());
  _ctx->SetRenderTarget(output, ObjectHandle(), nullptr);

  _ctx->SetShaderResource(input, ShaderType::PixelShader);
  _ctx->SetConstantBuffer(_cbScaleBias, ShaderType::PixelShader, 1);

  // TODO: This should really be moved in an input parameter
  u32 outputX, outputY;
  GRAPHICS.GetTextureSize(output, &outputX, &outputY);

  CD3D11_VIEWPORT viewport = CD3D11_VIEWPORT(0.f, 0.f, (float)outputX, (float)outputY);
  _ctx->SetViewports(1, viewport);

  _ctx->Draw(6, 0);

  _ctx->UnsetRenderTargets(0, 1);
  _ctx->UnsetShaderResources(0, 1, ShaderType::PixelShader);
}
