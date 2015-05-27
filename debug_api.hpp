#pragma once
#include "gpu_objects.hpp"

namespace tano
{
  struct DebugApi
  {
    static DebugApi& Instance();
    static void Create();
    static void Destroy();

    bool Init();

    void BeginFrame();
    void EndFrame();

    void SetTransform(const Matrix& worldViewProj);
    void AddDebugLine(const Vector3& start, const Vector3& end, const Color& color);

    u32 AddDebugAnchor();
    void UpdateDebugAnchor(u32 id);

    struct CBufferPerFrame
    {
      Matrix world;
      Matrix view;
      Matrix proj;
      Matrix viewProj;
    };
    ConstantBuffer<CBufferPerFrame> _cbPerFrame;

    static DebugApi* _instance;

    GpuState _gpuState;
    GpuObjects _gpuObjects;
  };

#define DEBUG_API DebugApi::Instance()

}
