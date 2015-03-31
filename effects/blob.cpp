#include "blob.hpp"
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
Blob::Blob(const string &name, u32 id)
  : Effect(name, id)
{
#if WITH_IMGUI
  PROPERTIES.Register(Name(),
    bind(&Blob::RenderParameterSet, this),
    bind(&Blob::SaveParameterSet, this));

  PROPERTIES.SetActive(Name());
#endif
}

//------------------------------------------------------------------------------
Blob::~Blob()
{
}

//------------------------------------------------------------------------------
bool Blob::Init(const char* configFile)
{
  BEGIN_INIT_SEQUENCE();

  _configName = configFile;
  vector<char> buf;
  INIT_FATAL(RESOURCE_MANAGER.LoadFile(configFile, &buf));

  INIT(ParseBlobSettings(InputBuffer(buf), &_settings));
  _camera._pitch = _settings.camera.pitch;
  _camera._yaw = _settings.camera.yaw;
  _camera._roll = _settings.camera.roll;
  _camera._pos = _settings.camera.pos;

  // load the blob mesh
  MeshLoader loader;
  INIT(loader.Load("gfx/blob1.boba"));
  u32 blobVertexFlags = 0;
  INIT(CreateBuffersFromMesh(loader, "Blob", &blobVertexFlags, &_blobGpuObjects));
  INIT(_blobGpuObjects.LoadShadersFromFile("shaders/out/blob", "VsMesh", nullptr, "PsMesh", blobVertexFlags));
  INIT(_blobState.Create());

  INIT(_cbPerFrame.Create());
  int w, h;
  GRAPHICS.GetBackBufferSize(&w, &h);
  _cbPerFrame.dim.x = (float)w;
  _cbPerFrame.dim.y = (float)h;

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool Blob::Update(const UpdateState& state)
{
  UpdateCameraMatrix();
  return true;
}

//------------------------------------------------------------------------------
void Blob::UpdateCameraMatrix()
{
  _camera.Update();
  Matrix view = _camera._view;
  Matrix proj = _camera._proj;
  Matrix viewProj = view * proj;

  // compute size of frustum
  float farW = _camera._farPlane * tan(_camera._fov);
  Vector3 v0(-farW, 0, _camera._farPlane);
  Vector3 v1(+farW, 0, _camera._farPlane);

  float nearW = _camera._nearPlane * tan(_camera._fov);
  Vector3 v2(-nearW, 0, _camera._nearPlane);
  Vector3 v3(+nearW, 0, _camera._nearPlane);

  v0 = Vector3::Transform(v0 + _camera._pos, _camera._mtx);
  v1 = Vector3::Transform(v1 + _camera._pos, _camera._mtx);
  v2 = Vector3::Transform(v2 + _camera._pos, _camera._mtx);
  v3 = Vector3::Transform(v3 + _camera._pos, _camera._mtx);

  _cbPerFrame.world = Matrix::Identity();
  _cbPerFrame.view = view.Transpose();
  _cbPerFrame.proj = proj.Transpose();
  _cbPerFrame.viewProj = viewProj.Transpose();
  _cbPerFrame.cameraPos = _camera._pos;
  _cbPerFrame.cameraLookAt = _camera._target;
  _cbPerFrame.cameraUp = _camera._up;
}

//------------------------------------------------------------------------------
bool Blob::Render()
{
  rmt_ScopedCPUSample(Blob_Render);

  static Color black(0, 0, 0, 0);

  _ctx->SetSwapChain(GRAPHICS.DefaultSwapChain(), black);
  _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::VertexShader, 0);

  _ctx->SetGpuObjects(_blobGpuObjects);
  _ctx->SetGpuState(_blobState);
  _ctx->DrawIndexed(_blobGpuObjects._numIndices, 0, 0);

  return true;
}

//------------------------------------------------------------------------------
bool Blob::InitAnimatedParameters()
{
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
Effect* Blob::Create(const char* name, u32 id)
{
  return new Blob(name, id);
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
