#include "particle_trail.hpp"
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
ParticleTrail::ParticleTrail(const string &name, const string& config, u32 id)
  : BaseEffect(name, config, id)
{
#if WITH_IMGUI
  PROPERTIES.Register(Name(),
    bind(&ParticleTrail::RenderParameterSet, this),
    bind(&ParticleTrail::SaveParameterSet, this));

  PROPERTIES.SetActive(Name());
#endif
}

//------------------------------------------------------------------------------
ParticleTrail::~ParticleTrail()
{
}

//------------------------------------------------------------------------------
bool ParticleTrail::OnConfigChanged(const vector<char>& buf)
{
  return ParseParticleTrailSettings(InputBuffer(buf), &_settings);
}

//------------------------------------------------------------------------------
bool ParticleTrail::Init()
{
  BEGIN_INIT_SEQUENCE();

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool ParticleTrail::Update(const UpdateState& state)
{
  UpdateCameraMatrix(state);
  return true;
}

//------------------------------------------------------------------------------
bool ParticleTrail::FixedUpdate(const FixedUpdateState& state)
{
  _camera.Update(state);
  return true;
}

//------------------------------------------------------------------------------
void ParticleTrail::UpdateCameraMatrix(const UpdateState& state)
{
  Matrix view = _camera._view;
  Matrix proj = _camera._proj;

  Matrix viewProj = view * proj;
}

//------------------------------------------------------------------------------
bool ParticleTrail::Render()
{
  rmt_ScopedCPUSample(ParticleTrail_Render);

  static Color black(0, 0, 0, 0);

  _ctx->SetSwapChain(GRAPHICS.DefaultSwapChain(), black);

  RenderTargetDesc desc = GRAPHICS.GetBackBufferDesc();

  return true;
}

//------------------------------------------------------------------------------
#if WITH_IMGUI
void ParticleTrail::RenderParameterSet()
{
  if (ImGui::Button("Reset"))
    Reset();
}
#endif

//------------------------------------------------------------------------------
#if WITH_IMGUI
void ParticleTrail::SaveParameterSet()
{
  SaveSettings(_settings);
}
#endif

//------------------------------------------------------------------------------
void ParticleTrail::Reset()
{
  _camera._pos = Vector3(0.f, 0.f, 0.f);
  _camera._pitch = _camera._yaw = _camera._roll = 0.f;
}

//------------------------------------------------------------------------------
bool ParticleTrail::Close()
{
  return true;
}

//------------------------------------------------------------------------------
BaseEffect* ParticleTrail::Create(const char* name, const char* config, u32 id)
{
  return new ParticleTrail(name, config, id);
}

//------------------------------------------------------------------------------
const char* ParticleTrail::Name()
{
  return "particle_trail";
}

//------------------------------------------------------------------------------
void ParticleTrail::Register()
{
  DEMO_ENGINE.RegisterFactory(Name(), ParticleTrail::Create);
}
