#include "debug_api.hpp"
#include "init_sequence.hpp"
#include "graphics_context.hpp"

using namespace tano;
using namespace bristol;

DebugApi* DebugApi::_instance;

//------------------------------------------------------------------------------
DebugApi& DebugApi::Instance()
{
  return *_instance;
}

//------------------------------------------------------------------------------
void DebugApi::Create()
{
  _instance = new DebugApi();
}

//------------------------------------------------------------------------------
void DebugApi::Destroy()
{
  delete exch_null(_instance);
}

//------------------------------------------------------------------------------
bool DebugApi::Init(GraphicsContext* ctx)
{
  _ctx = ctx;

  BEGIN_INIT_SEQUENCE();

  CD3D11_RASTERIZER_DESC rasterDesc = CD3D11_RASTERIZER_DESC(CD3D11_DEFAULT());
  rasterDesc.CullMode = D3D11_CULL_NONE;
  INIT(_gpuState.Create(nullptr, nullptr, &rasterDesc));

  INIT(_cbPerFrame.Create());

  INIT(_gpuObjects.LoadShadersFromFile("shaders/out/basic", "VsPosColor", nullptr, "PsPosColor", VF_POS | VF_COLOR));
  INIT(_gpuObjects.CreateDynamicVb(MAX_VERTS * sizeof(PosCol), sizeof(PosCol)));
  _gpuObjects._topology = D3D11_PRIMITIVE_TOPOLOGY_LINELIST;

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
void DebugApi::BeginFrame()
{
  _lineChunks.clear();
  _numVerts = 0;
}

//------------------------------------------------------------------------------
void DebugApi::EndFrame()
{
  PosCol* vtx = _ctx->MapWriteDiscard<PosCol>(_gpuObjects._vb);
  memcpy(vtx, _vertices, _numVerts * sizeof(PosCol));
  _ctx->Unmap(_gpuObjects._vb);

  _ctx->SetGpuObjects(_gpuObjects);
  _ctx->SetGpuState(_gpuState);

  for (const LineChunk& chunk : _lineChunks)
  {
    _cbPerFrame.world = chunk.mtxWorld.Transpose();
    _cbPerFrame.viewProj = chunk.mtxViewProj.Transpose();
    _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::VertexShader, 0);
    _ctx->Draw(chunk.numVertices, chunk.startOfs);
  }

}

//------------------------------------------------------------------------------
void DebugApi::SetTransform(const Matrix& world, const Matrix& viewProj)
{
  _lineChunks.push_back({world, viewProj, _numVerts, 0});
}

//------------------------------------------------------------------------------
void DebugApi::AddDebugLine(const Vector3& start, const Vector3& end, const Color& color)
{
  // if no line chunk is present, then there was no call to SetTransform, and we can bail
  assert(!_lineChunks.empty());

  LineChunk& chunk = _lineChunks.back();
  chunk.numVertices += 2;

  _vertices[_numVerts + 0].pos = start;
  _vertices[_numVerts + 0].col = color;

  _vertices[_numVerts + 1].pos = end;
  _vertices[_numVerts + 1].col = color;

  _numVerts += 2;
}
