#include "blur_post_process.hpp"
#include "init_sequence.hpp"
#include "graphics_context.hpp"

using namespace tano;

//------------------------------------------------------------------------------
bool BlurPostProcess::Init(GraphicsContext* ctx, float blurRadius)
{
  _ctx = ctx;

  BEGIN_INIT_SEQUENCE();
  // blur setup
  INIT(GRAPHICS.LoadComputeShadersFromFile("shaders/out/blur", &_csBlurTranspose, "BlurTranspose"));
  INIT(GRAPHICS.LoadComputeShadersFromFile("shaders/out/blur", &_csCopyTranspose, "CopyTranspose"));
  INIT(GRAPHICS.LoadComputeShadersFromFile("shaders/out/blur", &_csBlurX, "BoxBlurX"));

  INIT(_cbBlur.Create());
  _cbBlur.radius = blurRadius;

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
void BlurPostProcess::Apply(ObjectHandle inputBuffer, ObjectHandle outputBuffer)
{
  static Color black(0, 0, 0, 0);

  int w, h;
  GRAPHICS.GetBackBufferSize(&w, &h);
  int s = max(w, h);

  BufferFlags f = BufferFlags(BufferFlag::CreateSrv | BufferFlag::CreateUav);
  ScopedRenderTarget scratch0(s, s, DXGI_FORMAT_R16G16B16A16_FLOAT, f);
  ScopedRenderTarget scratch1(s, s, DXGI_FORMAT_R16G16B16A16_FLOAT, f);

  // set constant buffers
  _cbBlur.inputSize.x = (float)w;
  _cbBlur.inputSize.y = (float)h;
  _ctx->SetConstantBuffer(_cbBlur, ShaderType::ComputeShader, 0);

  // set constant buffers
  ObjectHandle srcDst[] =
  {
    // horiz
    inputBuffer, scratch0._rtHandle, scratch0._rtHandle, scratch1._rtHandle, scratch1._rtHandle, scratch0._rtHandle,
    // vert
    scratch1._rtHandle, scratch0._rtHandle, scratch0._rtHandle, scratch1._rtHandle, scratch1._rtHandle, scratch0._rtHandle,
  };

  // horizontal blur (ends up in scratch0)
  for (int i = 0; i < 3; ++i)
  {
    _ctx->SetShaderResources({ srcDst[i * 2 + 0] }, ShaderType::ComputeShader);
    _ctx->SetUnorderedAccessView(srcDst[i * 2 + 1], &black);

    _ctx->SetComputeShader(_csBlurX);
    _ctx->Dispatch(h / 32 + 1, 1, 1);

    _ctx->UnsetUnorderedAccessViews(0, 1);
    _ctx->UnsetShaderResources(0, 1, ShaderType::ComputeShader);
  }

  // copy/transpose from scratch0 -> scratch1
  _ctx->SetShaderResources({ scratch0._rtHandle }, ShaderType::ComputeShader);
  _ctx->SetUnorderedAccessView(scratch1._rtHandle, &black);

  _ctx->SetComputeShader(_csCopyTranspose);
  _ctx->Dispatch(h / 32 + 1, 1, 1);

  _ctx->UnsetUnorderedAccessViews(0, 1);
  _ctx->UnsetShaderResources(0, 1, ShaderType::ComputeShader);

  // "vertical" blur, ends up in scratch 0

  _cbBlur.inputSize.x = (float)h;
  _cbBlur.inputSize.y = (float)w;
  _ctx->SetConstantBuffer(_cbBlur, ShaderType::ComputeShader, 0);

  for (int i = 0; i < 3; ++i)
  {
    _ctx->SetShaderResources({ srcDst[6 + i * 2 + 0] }, ShaderType::ComputeShader);
    _ctx->SetUnorderedAccessView(srcDst[6 + i * 2 + 1], &black);

    _ctx->SetComputeShader(_csBlurX);
    _ctx->Dispatch(w / 32 + 1, 1, 1);

    _ctx->UnsetUnorderedAccessViews(0, 1);
    _ctx->UnsetShaderResources(0, 1, ShaderType::ComputeShader);
  }

  // copy/transpose from scratch0 -> blur1
  _ctx->SetShaderResources({ scratch0._rtHandle }, ShaderType::ComputeShader);
  _ctx->SetUnorderedAccessView(outputBuffer, &black);

  _ctx->SetComputeShader(_csCopyTranspose);
  _ctx->Dispatch(w / 32 + 1, 1, 1);

  _ctx->UnsetUnorderedAccessViews(0, 1);
  _ctx->UnsetShaderResources(0, 1, ShaderType::ComputeShader);
}
