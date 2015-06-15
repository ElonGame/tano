#pragma once
#include "gpu_objects.hpp"
namespace tano
{
  struct BlurPostProcess
  {
    bool Init(GraphicsContext* ctx, float blurRadius);
    void Apply(ObjectHandle inputBuffer, ObjectHandle outputBuffer);

    struct CBufferBlur
    {
      Vector2 inputSize;
      float radius = 10;
    };

    ConstantBuffer<CBufferBlur> _cbBlur;
    ObjectHandle _csBlurX;
    ObjectHandle _csBlurTranspose;
    ObjectHandle _csCopyTranspose;

    GraphicsContext* _ctx = nullptr;
  };
}