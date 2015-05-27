#pragma once
#include "gpu_objects.hpp"

namespace tano
{
  class GraphicsContext;

  struct DebugApi
  {
    static DebugApi& Instance();
    static void Create();
    static void Destroy();

    bool Init(GraphicsContext* ctx);

    void BeginFrame();
    void EndFrame();

    void SetTransform(const Matrix& world, const Matrix& viewProj);
    void AddDebugLine(const Vector3& start, const Vector3& end, const Color& color);

    u32 AddDebugAnchor();
    void UpdateDebugAnchor(u32 id);

    struct LineChunk
    {
      Matrix mtxWorld;
      Matrix mtxViewProj;
      int startOfs;
      int numVertices;
    };

    vector<LineChunk> _lineChunks;

    static const int MAX_VERTS = 128 * 1024;
    bristol::PosCol _vertices[MAX_VERTS];
    int _numVerts = 0;

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

    GraphicsContext* _ctx = nullptr;
  };

#define DEBUG_API DebugApi::Instance()

}
