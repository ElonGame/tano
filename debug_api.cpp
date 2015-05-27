#include "debug_api.hpp"
#include "init_sequence.hpp"

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
bool DebugApi::Init()
{
  BEGIN_INIT_SEQUENCE();

  CD3D11_RASTERIZER_DESC rasterDesc = CD3D11_RASTERIZER_DESC(CD3D11_DEFAULT());
  rasterDesc.CullMode = D3D11_CULL_NONE;
  INIT(_gpuState.Create(nullptr, nullptr, &rasterDesc));

  INIT(_cbPerFrame.Create());

  INIT(_gpuObjects.LoadShadersFromFile("shaders/out/basic", "VsPosColor", nullptr, "PsPosColor", VF_POS));

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
void DebugApi::BeginFrame()
{
}

//------------------------------------------------------------------------------
void DebugApi::EndFrame()
{
}

//------------------------------------------------------------------------------
void DebugApi::SetTransform(const Matrix& worldViewProj)
{
}

//------------------------------------------------------------------------------
void DebugApi::AddDebugLine(const Vector3& start, const Vector3& end, const Color& color)
{
}
