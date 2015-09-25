#pragma once
#include "gpu_objects.hpp"

namespace tano
{
  class GraphicsContext;

  struct DebugApi
  {
    static DebugApi& Instance();
    static bool Create(GraphicsContext* ctx);
    static void Destroy();

    void BeginFrame();
    void EndFrame();

    void SetTransform(const Matrix& world, const Matrix& viewProj);
    void AddDebugLine(const vec3& start, const vec3& end, const Color& color);
    void AddDebugLine(const vec3& start, const vec3& end, const Color& startColor, const Color& endColor);
    void AddDebugSphere(const vec3& center, float radius, const Color& color);
    void AddDebugCube(const vec3& center, const vec3& extents, const Color& color);

    u32 AddDebugAnchor();
    void UpdateDebugAnchor(u32 id);

    void CreateDebugGeometry();

    bool Init(GraphicsContext* ctx);

    struct LineChunk
    {
      Matrix mtxWorld;
      Matrix mtxViewProj;
      int startOfs;
      int numVertices;
    };

    vector<LineChunk> _lineChunks;
    vector<vec3> _unitSphere;

    static const int MAX_VERTS = 128 * 1024;

    struct PosCol
    {
      vec3 pos;
      Color col;
    };

    PosCol _vertices[MAX_VERTS];
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

    GpuState _gpuStateDepthTestDisabled;
    GpuState _gpuStateDepthTestEnabled;
    GpuObjects _gpuObjects;

    GraphicsContext* _ctx = nullptr;
  };

#define DEBUG_API DebugApi::Instance()

}
