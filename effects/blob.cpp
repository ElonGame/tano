#include "blob.hpp"
#include "../tano.hpp"
#include "../graphics.hpp"
#include "../graphics_context.hpp"
#include "../demo_engine.hpp"
#include "../resource_manager.hpp"
#include "../init_sequence.hpp"
#include "../generated/demo.parse.hpp"
#include "../arena_allocator.hpp"
#include "../stop_watch.hpp"
#include "../blackboard.hpp"
#include "../generated/demo.parse.hpp"
#include "../generated/input_buffer.hpp"
#include "../generated/output_buffer.hpp"

using namespace tano;
using namespace bristol;

//------------------------------------------------------------------------------
Blob::Blob(const string& name, const string& config, u32 id) : BaseEffect(name, config, id)
{
}

//------------------------------------------------------------------------------
Blob::~Blob()
{
}

//------------------------------------------------------------------------------
bool Blob::OnConfigChanged(const vector<char>& buf)
{
  return ParseBlobSettings(InputBuffer(buf), &_settings);
}

//------------------------------------------------------------------------------
bool Blob::Init()
{
  BEGIN_INIT_SEQUENCE();

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool Blob::Update(const UpdateState& state)
{
  UpdateCameraMatrix(state);
  return true;
}

//------------------------------------------------------------------------------
bool Blob::FixedUpdate(const FixedUpdateState& state)
{
  _camera.Update(state);
  return true;
}

//------------------------------------------------------------------------------
void Blob::UpdateCameraMatrix(const UpdateState& state)
{
  Matrix view = _camera._view;
  Matrix proj = _camera._proj;

  Matrix viewProj = view * proj;
}

//------------------------------------------------------------------------------
bool Blob::Render()
{
  rmt_ScopedCPUSample(Blob_Render);

  static Color black(0, 0, 0, 0);

  _ctx->SetSwapChain(GRAPHICS.DefaultSwapChain(), black);

  RenderTargetDesc desc = GRAPHICS.GetBackBufferDesc();

  return true;
}

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Blob::RenderParameterSet()
{
  if (ImGui::Button("Reset"))
    Reset();
}
#endif

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Blob::SaveParameterSet()
{
  SaveSettings(_settings);
}
#endif

//------------------------------------------------------------------------------
void Blob::Reset()
{
  _camera._pos = Vector3(0.f, 0.f, 0.f);
  _camera._pitch = _camera._yaw = _camera._roll = 0.f;
}

//------------------------------------------------------------------------------
bool Blob::Close()
{
  return true;
}

//------------------------------------------------------------------------------
BaseEffect* Blob::Create(const char* name, const char* config, u32 id)
{
  return new Blob(name, config, id);
}

//------------------------------------------------------------------------------
const char* Blob::Name()
{
  return "blob";
}

//------------------------------------------------------------------------------
void Blob::Register()
{
  DEMO_ENGINE.RegisterFactory(Name(), Blob::Create);
}
