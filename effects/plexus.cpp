#include "Plexus.hpp"
#include "../tano.hpp"
#include "../graphics.hpp"
#include "../graphics_context.hpp"
#include "../demo_engine.hpp"
#include "../resource_manager.hpp"
#include "../init_sequence.hpp"
#include "../generated/demo.parse.hpp"
#include "../generated/input_buffer.hpp"
#include "../generated/output_buffer.hpp"
#include "../mesh_utils.hpp"
#include "../fullscreen_effect.hpp"

using namespace tano;
using namespace bristol;

//------------------------------------------------------------------------------
Plexus::Plexus(const string &name, u32 id)
  : BaseEffect(name, id)
{
#if WITH_IMGUI
  PROPERTIES.Register(Name(),
    bind(&Plexus::RenderParameterSet, this),
    bind(&Plexus::SaveParameterSet, this));

  PROPERTIES.SetActive(Name());
#endif
}

//------------------------------------------------------------------------------
Plexus::~Plexus()
{
}

//------------------------------------------------------------------------------
bool Plexus::Init(const char* configFile)
{
  BEGIN_INIT_SEQUENCE();

  _configName = configFile;
  vector<char> buf;
  INIT_FATAL(RESOURCE_MANAGER.LoadFile(configFile, &buf));

  INIT(ParsePlexusSettings(InputBuffer(buf), &_settings));
  _camera._pitch = _settings.camera.pitch;
  _camera._yaw = _settings.camera.yaw;
  _camera._roll = _settings.camera.roll;
  _camera._pos = _settings.camera.pos;

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool Plexus::Update(const UpdateState& state)
{
  UpdateCameraMatrix(state);
  return true;
}

//------------------------------------------------------------------------------
void Plexus::UpdateCameraMatrix(const UpdateState& state)
{
  _camera.Update(state);

  Matrix view = _camera._view;
  Matrix proj = _camera._proj;

  Matrix viewProj = view * proj;

  _cbPerFrame.world = Matrix::Identity();
  _cbPerFrame.view = view.Transpose();
  _cbPerFrame.proj = proj.Transpose();
  _cbPerFrame.viewProj = viewProj.Transpose();
  _cbPerFrame.cameraPos = _camera._pos;
  _cbPerFrame.cameraLookAt = _camera._target;
  _cbPerFrame.cameraUp = _camera._up;
}

//------------------------------------------------------------------------------
bool Plexus::Render()
{
  rmt_ScopedCPUSample(Plexus_Render);

  static Color black(.1f, .1f, .1f, 0);

  _ctx->SetSwapChain(GRAPHICS.DefaultSwapChain(), black);

  return true;
}

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Plexus::RenderParameterSet()
{
  if (ImGui::Button("Reset"))
    Reset();
}
#endif

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Plexus::SaveParameterSet()
{
  OutputBuffer buf;
  _settings.camera.pos = _camera._pos;
  _settings.camera.yaw = _camera._yaw;
  _settings.camera.pitch = _camera._pitch;
  _settings.camera.roll = _camera._roll;
  Serialize(buf, _settings);
  if (FILE* f = fopen(_configName.c_str(), "wt"))
  {
    fwrite(buf._buf.data(), 1, buf._ofs, f);
    fclose(f);
  }
}
#endif

//------------------------------------------------------------------------------
void Plexus::Reset()
{
  _camera._pos = Vector3(0.f, 0.f, 0.f);
  _camera._pitch = _camera._yaw = _camera._roll = 0.f;
}

//------------------------------------------------------------------------------
bool Plexus::Close()
{
  return true;
}

//------------------------------------------------------------------------------
BaseEffect* Plexus::Create(const char* name, u32 id)
{
  return new Plexus(name, id);
}

//------------------------------------------------------------------------------
const char* Plexus::Name()
{
  return "Plexus";
}

//------------------------------------------------------------------------------
void Plexus::Register()
{
  DEMO_ENGINE.RegisterFactory(Name(), Plexus::Create);
}
