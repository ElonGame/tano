#pragma once
#include "object_handle.hpp"
#include "gpu_objects.hpp"

namespace tano
{
  class GraphicsContext;

  struct PostProcess
  {
    PostProcess(GraphicsContext* ctx);

    bool Init();
    void Setup();

    void Execute(
      const vector<ObjectHandle>& input,
      ObjectHandle output,
      ObjectHandle shader,
      const Color* clearColor,
      WCHAR* name);

    GraphicsContext* _ctx;

    struct CBufferPS
    {
      Vector2 inputSize;
      Vector2 outputSize;
    };
    ConstantBuffer<CBufferPS> _cb;

    GpuState _gpuState;
    GpuObjects _gpuObjects;

    ObjectHandle _vsQuad;
  };
}