#pragma once
#include "object_handle.hpp"
#include "gpu_objects.hpp"

namespace tano
{
  class GraphicsContext;

  class FullscreenEffect
  {
  public:
    FullscreenEffect(GraphicsContext* ctx);

    bool Init();

    void Execute(
      ObjectHandle input,
      ObjectHandle output,
      const RenderTargetDesc& outputDesc,
      ObjectHandle depthStencil,
      ObjectHandle shader,
      bool releaseOutput = true,
      const Color* clearColor = nullptr);

    void Execute(
      const ObjectHandle* inputs,
      int numInputs, 
      ObjectHandle output,
      const RenderTargetDesc& outputDesc,
      ObjectHandle depthStencil,
      ObjectHandle shader,
      bool releaseOutput = true,
      const Color* clearColor = nullptr);

    void ScaleBias(
      ObjectHandle input,
      ObjectHandle output,
      const RenderTargetDesc& outputDesc,
      float scale,
      float bias);

    void ScaleBiasSecondary(
      ObjectHandle input0,
      ObjectHandle input1, 
      ObjectHandle output, 
      const RenderTargetDesc& outputDesc, 
      float scale, 
      float bias);

    void Blur(ObjectHandle inputBuffer, ObjectHandle outputBuffer, float radius, int scale);
    void Copy(ObjectHandle inputBuffer, ObjectHandle outputBuffer, const RenderTargetDesc& outputDesc, bool releaseOutput = true);
    void Add(ObjectHandle input0, ObjectHandle input1, ObjectHandle outputBuffer, float scale0, float scale1);

  private:
    GraphicsContext* _ctx;

    struct CBufferPerFrame
    {
      Vector2 inputSize;
      Vector2 outputSize;
    };
    ConstantBuffer<CBufferPerFrame> _cb;

    struct CBufferScaleBias
    {
      Vector4 scaleBias = Vector4(1.0, 0.5, 0, 0);
    };
    ConstantBuffer<CBufferScaleBias> _cbScaleBias;

    struct CBufferBlur
    {
      Vector2 inputSize;
      float radius = 10;
    };

    ConstantBuffer<CBufferBlur> _cbBlur;
    ObjectHandle _csBlurX;
    ObjectHandle _csBlurTranspose;
    ObjectHandle _csCopyTranspose;

    GpuBundle _addBundle;
    GpuBundle _copyBundle;

    GpuBundle _defaultBundle;
    GpuBundle _scaleBiasBundle;
    GpuBundle _scaleBiasSecondaryBundle;
  };
}