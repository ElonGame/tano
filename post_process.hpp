#pragma once
#include "object_handle.hpp"
#include "gpu_objects.hpp"

namespace tano
{
  class GraphicsContext;

  class PostProcess
  {
  public:
    PostProcess(GraphicsContext* ctx);

    bool Init();

    // Render shader to a temp render target with the given format
    ObjectHandle Execute(
      const vector<ObjectHandle>& input, 
      DXGI_FORMAT outputFormat, 
      const GpuObjects& gpuObjects,
      const Color* clearColor = nullptr);

    // Render shader to the given output
    void Execute(
      const vector<ObjectHandle>& input,
      ObjectHandle output,
      ObjectHandle shader,
      bool releaseOutput = true,
      const Color* clearColor = nullptr);

  private:
    GraphicsContext* _ctx;

    struct CBufferPS
    {
      Vector2 inputSize;
      Vector2 outputSize;
    };
    ConstantBuffer<CBufferPS> _cb;

    GpuState _gpuState;
    GpuObjects _gpuObjects;
  };
}