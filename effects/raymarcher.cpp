#include "raymarcher.hpp"
#include "../tano.hpp"
#include "../graphics.hpp"
#include "../graphics_context.hpp"
#include "../demo_engine.hpp"
#include "../resource_manager.hpp"
#include "../init_sequence.hpp"
#include "../generated/demo.parse.hpp"

using namespace tano;
using namespace bristol;

//------------------------------------------------------------------------------
RayMarcher::RayMarcher(const string& name, const string& config, u32 id) : BaseEffect(name, config, id)
{
}

//------------------------------------------------------------------------------
RayMarcher::~RayMarcher()
{
}

//------------------------------------------------------------------------------
bool RayMarcher::OnConfigChanged(const vector<char>& buf)
{
  _settings = ParseRayMarcherSettings(InputBuffer(buf));
  return true;
}

//------------------------------------------------------------------------------
bool RayMarcher::Init()
{
  BEGIN_INIT_SEQUENCE();

  INIT(_raymarcherGpuObjects.LoadVertexShader("shaders/out/common", "VsQuad"));
  INIT(_raymarcherGpuObjects.LoadPixelShader("shaders/out/raymarcher", "PsRaymarcher"));
  INIT(_raymarcherState.Create());

  int w, h;
  INIT(_cbPerFrame.Create());
  GRAPHICS.GetBackBufferSize(&w, &h);
  _cbPerFrame.dim.x = (float)w;
  _cbPerFrame.dim.y = (float)h;

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool RayMarcher::Update(const UpdateState& state)
{
  rmt_ScopedCPUSample(RayMarcher_Update);

  _cbPerFrame.time = state.localTime.TotalMilliseconds() / 1000.f;

  return true;
}

//------------------------------------------------------------------------------
bool RayMarcher::Render()
{
  rmt_ScopedCPUSample(RayMarcher_Render);

  _ctx->SetSwapChain(GRAPHICS.DefaultSwapChain(), Color(0.1f, 0.1f, 0.1f, 0));
  _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::PixelShader, 0);

  _ctx->SetGpuObjects(_raymarcherGpuObjects);
  _ctx->SetGpuState(_raymarcherState);
  _ctx->Draw(3, 0);

  return true;
}

//------------------------------------------------------------------------------
#if WITH_IMGUI
void RayMarcher::RenderParameterSet()
{
  if (ImGui::Button("Reset"))
    Reset();
}
#endif

//------------------------------------------------------------------------------
#if WITH_IMGUI
void RayMarcher::SaveParameterSet(bool inc)
{
  SaveSettings(_settings, inc);
}
#endif

//------------------------------------------------------------------------------
void RayMarcher::Reset()
{
}

//------------------------------------------------------------------------------
bool RayMarcher::Close()
{
  return true;
}

//------------------------------------------------------------------------------
BaseEffect* RayMarcher::Create(const char* name, const char* config, u32 id)
{
  return new RayMarcher(name, config, id);
}

//------------------------------------------------------------------------------
const char* RayMarcher::Name()
{
  return "raymarcher";
}

//------------------------------------------------------------------------------
void RayMarcher::Register()
{
  DEMO_ENGINE.RegisterFactory(Name(), RayMarcher::Create);
}
