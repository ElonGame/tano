#include "sample.hpp"
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
Sample::Sample(const string &name, const string& config, u32 id)
  : BaseEffect(name, config, id)
{
#if WITH_IMGUI
  PROPERTIES.Register(Name(),
    bind(&Sample::RenderParameterSet, this),
    bind(&Sample::SaveParameterSet, this));

  PROPERTIES.SetActive(Name());
#endif
}

//------------------------------------------------------------------------------
Sample::~Sample()
{
}

//------------------------------------------------------------------------------
bool Sample::OnConfigChanged(const vector<char>& buf)
{
  return ParseSampleSettings(InputBuffer(buf), &_settings);
}

//------------------------------------------------------------------------------
bool Sample::Init()
{
  BEGIN_INIT_SEQUENCE();

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool Sample::Update(const UpdateState& state)
{
  UpdateCameraMatrix(state);
  return true;
}

//------------------------------------------------------------------------------
bool Sample::FixedUpdate(const FixedUpdateState& state)
{
  _camera.Update(state);
  return true;
}

//------------------------------------------------------------------------------
void Sample::UpdateCameraMatrix(const UpdateState& state)
{
  Matrix view = _camera._view;
  Matrix proj = _camera._proj;

  Matrix viewProj = view * proj;
}

//------------------------------------------------------------------------------
bool Sample::Render()
{
  rmt_ScopedCPUSample(Sample_Render);

  static Color black(0, 0, 0, 0);

  _ctx->SetSwapChain(GRAPHICS.DefaultSwapChain(), black);

  RenderTargetDesc desc = GRAPHICS.GetBackBufferDesc();

  return true;
}

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Sample::RenderParameterSet()
{
  if (ImGui::Button("Reset"))
    Reset();
}
#endif

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Sample::SaveParameterSet()
{
  SaveSettings(_settings);
}
#endif

//------------------------------------------------------------------------------
void Sample::Reset()
{
  _camera._pos = Vector3(0.f, 0.f, 0.f);
  _camera._pitch = _camera._yaw = _camera._roll = 0.f;
}

//------------------------------------------------------------------------------
bool Sample::Close()
{
  return true;
}

//------------------------------------------------------------------------------
BaseEffect* Sample::Create(const char* name, const char* config, u32 id)
{
  return new Sample(name, config, id);
}

//------------------------------------------------------------------------------
const char* Sample::Name()
{
  return "sample";
}

//------------------------------------------------------------------------------
void Sample::Register()
{
  DEMO_ENGINE.RegisterFactory(Name(), Sample::Create);
}
