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
bool DebugApi::Create(GraphicsContext* ctx)
{
  _instance = new DebugApi();
  return _instance->Init(ctx);
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

  CD3D11_DEPTH_STENCIL_DESC dsDesc = CD3D11_DEPTH_STENCIL_DESC(CD3D11_DEFAULT());
  dsDesc.DepthEnable = FALSE;

  CD3D11_RASTERIZER_DESC rasterDesc = CD3D11_RASTERIZER_DESC(CD3D11_DEFAULT());
  rasterDesc.CullMode = D3D11_CULL_NONE;
  INIT(_gpuStateDepthTestDisabled.Create(&dsDesc, nullptr, &rasterDesc));

  dsDesc.DepthEnable = TRUE;
  INIT(_gpuStateDepthTestEnabled.Create(nullptr, nullptr, &rasterDesc));

  INIT(_cbPerFrame.Create());

  INIT(_gpuObjects.LoadShadersFromFile("shaders/out/basic", "VsPosColor", nullptr, "PsPosColor", VF_POS | VF_COLOR));
  INIT(_gpuObjects.CreateDynamicVb(MAX_VERTS * sizeof(PosCol), sizeof(PosCol)));
  _gpuObjects._topology = D3D11_PRIMITIVE_TOPOLOGY_LINELIST;

  CreateDebugGeometry();

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
void AddVertex(vector<Vector3>* verts, const Vector3& v)
{
  // project point onto the until sphere
  float len = v.Length();
  verts->push_back(v / len);
}

void ExpandTriangle(vector<Vector3>* expanded, const vector<Vector3>& verts, int a, int b, int c)
{
  expanded->push_back(verts[a]);
  expanded->push_back(verts[b]);
  expanded->push_back(verts[b]);
  expanded->push_back(verts[c]);
  expanded->push_back(verts[c]);
  expanded->push_back(verts[a]);
}

void DebugApi::CreateDebugGeometry()
{
  float t = (1.f + sqrtf(5)) / 2;
  vector<Vector3> verts;
  AddVertex(&verts, { -1, +t, 0 });
  AddVertex(&verts, { +1, +t, 0 });
  AddVertex(&verts, { -1, -t, 0 });
  AddVertex(&verts, { +1, -t, 0 });

  AddVertex(&verts, { 0, -1, +t });
  AddVertex(&verts, { 0, +1, +t });
  AddVertex(&verts, { 0, -1, -t });
  AddVertex(&verts, { 0, +1, -t });

  AddVertex(&verts, { +t, 0, -1 });
  AddVertex(&verts, { +t, 0, +1 });
  AddVertex(&verts, { -t, 0, -1 });
  AddVertex(&verts, { -t, 0, +1 });

  vector<Vector3> expanded;
  ExpandTriangle(&expanded, verts, 0, 11, 5);
  ExpandTriangle(&expanded, verts, 0, 5, 1);
  ExpandTriangle(&expanded, verts, 0, 1, 7);
  ExpandTriangle(&expanded, verts, 0, 7, 10);
  ExpandTriangle(&expanded, verts, 0, 10, 11);

  ExpandTriangle(&expanded, verts, 1, 5, 9);
  ExpandTriangle(&expanded, verts, 5, 11, 4);
  ExpandTriangle(&expanded, verts, 11, 10, 2);
  ExpandTriangle(&expanded, verts, 10, 7, 6);
  ExpandTriangle(&expanded, verts, 7, 1, 8);

  ExpandTriangle(&expanded, verts, 3, 9, 4);
  ExpandTriangle(&expanded, verts, 3, 4, 2);
  ExpandTriangle(&expanded, verts, 3, 2, 6);
  ExpandTriangle(&expanded, verts, 3, 6, 8);
  ExpandTriangle(&expanded, verts, 3, 8, 9);

  ExpandTriangle(&expanded, verts, 4, 9, 5);
  ExpandTriangle(&expanded, verts, 2, 4, 11);
  ExpandTriangle(&expanded, verts, 6, 2, 10);
  ExpandTriangle(&expanded, verts, 8, 6, 7);
  ExpandTriangle(&expanded, verts, 9, 8, 1);

  _unitSphere = expanded;
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
  _ctx->SetGpuState(_gpuStateDepthTestDisabled);

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
  // if the transform is the same as the previous one, just append to it
  if (_lineChunks.empty() || _lineChunks.back().mtxWorld != world || _lineChunks.back().mtxViewProj != viewProj)
  {
    _lineChunks.push_back({ world, viewProj, _numVerts, 0 });
  }
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

//------------------------------------------------------------------------------
void DebugApi::AddDebugSphere(const Vector3& center, float radius, const Color& color)
{
  assert(!_lineChunks.empty());
  LineChunk& chunk = _lineChunks.back();

  int i = _numVerts;
  for (const Vector3& p : _unitSphere)
  {
    _vertices[i].pos = center + radius * p;
    _vertices[i].col = color;
    i++;
  }

  chunk.numVertices += (int)_unitSphere.size();
  _numVerts += (int)_unitSphere.size();
}

//------------------------------------------------------------------------------
void DebugApi::AddDebugCube(const Vector3& center, const Vector3& extents, const Color& color)
{
  assert(!_lineChunks.empty());
  LineChunk& chunk = _lineChunks.back();
}
