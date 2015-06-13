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

  CD3D11_DEPTH_STENCIL_DESC dsDesc = CD3D11_DEPTH_STENCIL_DESC(CD3D11_DEFAULT());
  dsDesc.DepthEnable = FALSE;
  dsDesc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ZERO;
  dsDesc.DepthFunc = D3D11_COMPARISON_LESS_EQUAL;

  CD3D11_RASTERIZER_DESC rsDesc = CD3D11_RASTERIZER_DESC(CD3D11_DEFAULT());
  rsDesc.CullMode = D3D11_CULL_NONE;

  INIT(_gpuState.Create(&dsDesc, nullptr, &rsDesc));
  INIT(_gpuObjects.LoadShadersFromFile("shaders/out/quad", "VsMain", nullptr, nullptr));

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
  _ctx->SetLayout(ObjectHandle());
  _ctx->SetGpuState(_gpuState);
  _ctx->SetGpuStateSamplers(_gpuState, ShaderType::PixelShader);
  _ctx->SetGpuStateSamplers(_gpuState, ShaderType::ComputeShader);
  _ctx->SetGpuObjects(_gpuObjects);

  if (output.IsValid())
    _ctx->SetRenderTarget(output, depthStencil, clearColor);

  _ctx->SetShaderResources(input, ShaderType::PixelShader);

  u32 outputX, outputY;
  GRAPHICS.GetTextureSize(output, &outputX, &outputY);

  CD3D11_VIEWPORT viewport = CD3D11_VIEWPORT(0.f, 0.f, (float)outputX, (float)outputY);
  _ctx->SetViewports(1, viewport);

  _ctx->SetPixelShader(shader);
  _ctx->Draw(6, 0);

  if (output.IsValid() && releaseOutput)
    _ctx->UnsetRenderTargets(0, 1);

  _ctx->UnsetShaderResources(0, (u32)input.size(), ShaderType::PixelShader);
}
