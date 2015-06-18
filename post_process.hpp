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

    // TODO: Render shader to a temp render target with the given format

    // Render shader to the given output
    void Execute(
      const vector<ObjectHandle>& input,
      ObjectHandle output,
      ObjectHandle depthStencil,
      ObjectHandle shader,
      bool releaseOutput = true,
      const Color* clearColor = nullptr);

    void Execute(
      ObjectHandle input,
      ObjectHandle output,
      ObjectHandle depthStencil,
      ObjectHandle shader,
      bool releaseOutput = true,
      const Color* clearColor = nullptr);

    void Execute(
      const ObjectHandle* inputs,
      int numInputs, 
      ObjectHandle output,
      ObjectHandle depthStencil,
      ObjectHandle shader,
      bool releaseOutput = true,
      const Color* clearColor = nullptr);

  private:
    GraphicsContext* _ctx;

    struct CBufferPerFrame
    {
      Vector2 inputSize;
      Vector2 outputSize;
    };
    ConstantBuffer<CBufferPerFrame> _cb;

    GpuState _gpuState;
    GpuObjects _gpuObjects;
  };
}