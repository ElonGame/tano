#include "cloth.hpp"
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
#include "../post_process.hpp"

using namespace tano;
using namespace bristol;

//------------------------------------------------------------------------------
Cloth::Cloth(const string &name, u32 id)
: Effect(name, id)
{
#if WITH_IMGUI
  PROPERTIES.Register(Name(),
    bind(&Cloth::RenderParameterSet, this),
    bind(&Cloth::SaveParameterSet, this));

  PROPERTIES.SetActive(Name());
#endif
}

//------------------------------------------------------------------------------
Cloth::~Cloth()
{
}

//------------------------------------------------------------------------------
bool Cloth::Init(const char* configFile)
{
  BEGIN_INIT_SEQUENCE();

  _configName = configFile;
  vector<char> buf;
  INIT_FATAL(RESOURCE_MANAGER.LoadFile(configFile, &buf));

  INIT(ParseClothSettings(InputBuffer(buf), &_settings));
  _camera._pitch = _settings.camera.pitch;
  _camera._yaw = _settings.camera.yaw;
  _camera._roll = _settings.camera.roll;
  _camera._pos = _settings.camera.pos;

  CD3D11_RASTERIZER_DESC rasterDesc = CD3D11_RASTERIZER_DESC(CD3D11_DEFAULT());
  rasterDesc.CullMode = D3D11_CULL_NONE;
  INIT(_clothState.Create(nullptr, nullptr, &rasterDesc));

  INIT(_cbPerFrame.Create());
  int w, h;
  GRAPHICS.GetBackBufferSize(&w, &h);
  _cbPerFrame.dim.x = (float)w;
  _cbPerFrame.dim.y = (float)h;

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool Cloth::Update(const UpdateState& state)
{
  UpdateCameraMatrix();
  return true;
}

//------------------------------------------------------------------------------
void Cloth::UpdateCameraMatrix()
{
  _camera.Update();

  int w, h;
  GRAPHICS.GetBackBufferSize(&w, &h);
  float aspectRatio = (float)w / h;

  Matrix proj = _camera._proj;
  Matrix view = _camera._view;

  Matrix viewProj = view * proj;

  _cbPerFrame.world = Matrix::Identity();
  _cbPerFrame.view = view.Transpose();
  _cbPerFrame.proj = proj.Transpose();
  _cbPerFrame.viewProj = viewProj.Transpose();
  _cbPerFrame.cameraPos = _camera._pos;
//  _cbPerFrame.cameraLookAt = target;
//  _cbPerFrame.cameraUp = up;
}

//------------------------------------------------------------------------------
bool Cloth::Render()
{
  rmt_ScopedCPUSample(Cloth_Render);

  static Color black(.1f, .1f, .1f, 0);

  _ctx->SetSwapChain(GRAPHICS.DefaultSwapChain(), black);
  _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::PixelShader, 0);

  _ctx->SetGpuState(_clothState);

  _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::VertexShader, 0);
  _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::PixelShader, 0);

  return true;
}

//------------------------------------------------------------------------------
bool Cloth::InitAnimatedParameters()
{
  return true;
}

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Cloth::RenderParameterSet()
{
  if (ImGui::Button("Reset"))
    Reset();
}
#endif

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Cloth::SaveParameterSet()
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
void Cloth::Reset()
{
  _camera._pos = Vector3(0.f, 0.f, 0.f);
  _camera._pitch = _camera._yaw = _camera._roll = 0.f;
}

//------------------------------------------------------------------------------
bool Cloth::Close()
{
  return true;
}

//------------------------------------------------------------------------------
Effect* Cloth::Create(const char* name, u32 id)
{
  return new Cloth(name, id);
}

//------------------------------------------------------------------------------
const char* Cloth::Name()
{
  return "cloth";
}

//------------------------------------------------------------------------------
void Cloth::Register()
{
  DEMO_ENGINE.RegisterFactory(Name(), Cloth::Create);
}
