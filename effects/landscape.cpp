#include "landscape.hpp"
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


using namespace tano;
using namespace bristol;

namespace
{
  float angle = 0;
  float height = 200;
  float distance = 500;
}

//------------------------------------------------------------------------------
Landscape::Landscape(const string &name, u32 id)
  : Effect(name, id)
  , _blinkFace("blinkface")
{
#if WITH_IMGUI
  PROPERTIES.Register(Name(),
    bind(&Landscape::RenderParameterSet, this),
    bind(&Landscape::SaveParameterSet, this));

  PROPERTIES.SetActive(Name());
#endif
}

//------------------------------------------------------------------------------
Landscape::~Landscape()
{
}

//------------------------------------------------------------------------------
bool Landscape::Init(const char* configFile)
{
  auto sx = ToSpherical({ -1, 0, 0 });
  auto vx = FromSpherical(sx);

  auto sy = ToSpherical({ 0, -1, 0 });
  auto vy = FromSpherical(sy);

  auto sz = ToSpherical({ 0, 0, -1 });
  auto vz = FromSpherical(sz);

  BEGIN_INIT_SEQUENCE();

  _configName = configFile;
  vector<char> buf;
  INIT_FATAL(RESOURCE_MANAGER.LoadFile(configFile, &buf));

  INIT(ParseLandscapeSettings(InputBuffer(buf), &_settings));

  INIT_FATAL(RESOURCE_MANAGER.LoadFile("gfx/landscape1.png", &buf));
  int x, y, n;
  u8* data = stbi_load_from_memory((const u8*)buf.data(), (int)buf.size(), &x, &y, &n, 4);

  // create mesh from landscape
  u32 vertexFlags = VF_POS | VF_NORMAL;
  INIT(CreateBuffersFromBitmapFaceted(data, x, y, Vector3(10, 20, 10), &vertexFlags, &_landscapeGpuObjects));

  INIT(_landscapeGpuObjects.LoadShadersFromFile("shaders/out/Landscape", "VsLandscape", nullptr, "PsLandscape", vertexFlags));
  INIT(_landscapeState.Create());

  int w, h;
  INIT(_cbPerFrame.Create());
  GRAPHICS.GetBackBufferSize(&w, &h);

//  _camera._pos = Vector3(0, 200, 500);

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool Landscape::Update(const UpdateState& state)
{
  rmt_ScopedCPUSample(Landscape_Update);

  UpdateCameraMatrix();

  return true;
}

//------------------------------------------------------------------------------
void Landscape::UpdateCameraMatrix()
{
  float x = distance * sin(angle);
  float y = height;
  float z = distance * cos(angle);

  Vector3 pos = Vector3(x, y, z);
  Vector3 target = Vector3(0, 0, 0);
  Vector3 dir = target - pos;
  dir.Normalize();

  Matrix view = Matrix::CreateLookAt(pos, Vector3(0, 0, 0), Vector3(0, 1, 0));
  _camera.Update();
  view = _camera._view;
  Matrix proj = Matrix::CreatePerspectiveFieldOfView(XMConvertToRadians(45), 16/10.f, 0.1f, 2000.f);
  Matrix viewProj = view * proj;

  _cbPerFrame.world = Matrix::Identity();
  _cbPerFrame.view = view.Transpose();
  _cbPerFrame.proj = proj.Transpose();
  _cbPerFrame.viewProj = viewProj.Transpose();
}

//------------------------------------------------------------------------------
bool Landscape::Render()
{
  _ctx->SetSwapChain(GRAPHICS.DefaultSwapChain(), Color(0.1f, 0.1f, 0.1f, 0));
  _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::VertexShader, 0);

  rmt_ScopedCPUSample(Landscape_Render);

  _ctx->SetGpuObjects(_landscapeGpuObjects);
  _ctx->SetGpuState(_landscapeState);
  _ctx->Draw(_landscapeGpuObjects._numVerts, 0);

  return true;
}

//------------------------------------------------------------------------------
bool Landscape::InitAnimatedParameters()
{
  return true;
}

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Landscape::RenderParameterSet()
{

  ImGui::SliderAngle("camera xz-plane", &angle);
  ImGui::SliderFloat("camera distance", &distance, -1000, 1000);
  ImGui::SliderFloat("camera height", &height, -200, 200);

  if (ImGui::Button("Reset"))
    Reset();
}
#endif

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Landscape::SaveParameterSet()
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
void Landscape::Reset()
{
}

//------------------------------------------------------------------------------
bool Landscape::Close()
{
  return true;
}

//------------------------------------------------------------------------------
Effect* Landscape::Create(const char* name, u32 id)
{
  return new Landscape(name, id);
}

//------------------------------------------------------------------------------
const char* Landscape::Name()
{
  return "landscape";
}

//------------------------------------------------------------------------------
void Landscape::Register()
{
  DEMO_ENGINE.RegisterFactory(Name(), Landscape::Create);
}
