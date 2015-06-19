#include "raymarcher.hpp"
#include "../tano.hpp"
#include "../graphics.hpp"
#include "../graphics_context.hpp"
#include "../demo_engine.hpp"
#include "../resource_manager.hpp"
#include "../init_sequence.hpp"
#include "../generated/demo.parse.hpp"
#include "../generated/input_buffer.hpp"
#include "../generated/output_buffer.hpp"

using namespace tano;
using namespace bristol;

//------------------------------------------------------------------------------
RayMarcher::RayMarcher(const string &name, u32 id)
  : Effect(name, id)
{
#if WITH_IMGUI
  PROPERTIES.Register("ray marcher",
    bind(&RayMarcher::RenderParameterSet, this),
    bind(&RayMarcher::SaveParameterSet, this));
#endif
}

//------------------------------------------------------------------------------
RayMarcher::~RayMarcher()
{
}

//------------------------------------------------------------------------------
bool RayMarcher::Init(const char* configFile)
{
  BEGIN_INIT_SEQUENCE();

  _configName = configFile;
  vector<char> buf;
  if (!RESOURCE_MANAGER.LoadFile(configFile, &buf))
    return false;

  INIT(ParseRayMarcherSettings(InputBuffer(buf), &_settings));

  INIT(_raymarcherGpuObjects.LoadVertexShader("shaders/out/raymarcher", "VsQuad"));
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

  if (state.numTicks == 0)
    return true;

  return true;
}

//------------------------------------------------------------------------------
bool RayMarcher::Render()
{
  _ctx->SetSwapChain(GRAPHICS.DefaultSwapChain(), Color(0.1f, 0.1f, 0.1f, 0));
  _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::PixelShader, 0);

  rmt_ScopedCPUSample(RayMarcher_Render);

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
void RayMarcher::SaveParameterSet()
{
  OutputBuffer buf;
  Serialize(buf, _settings);
  if (FILE* f = fopen(_configName.c_str(), "wt"))
  {
    fwrite(buf._buf.data(), 1, buf._ofs, f);
    fclose(f);
  }
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
Effect* RayMarcher::Create(const char* name, u32 id)
{
  return new RayMarcher(name, id);
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
