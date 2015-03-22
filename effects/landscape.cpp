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
#include "../post_process.hpp"

using namespace tano;
using namespace bristol;

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
  BEGIN_INIT_SEQUENCE();

  _configName = configFile;
  vector<char> buf;
  INIT_FATAL(RESOURCE_MANAGER.LoadFile(configFile, &buf));

  INIT(ParseLandscapeSettings(InputBuffer(buf), &_settings));
  _camera._pitch = _settings.pitch;
  _camera._yaw = _settings.yaw;
  _camera._pos = _settings.pos;

  INIT_FATAL(RESOURCE_MANAGER.LoadFile("gfx/landscape1.png", &buf));
  int x, y, n;
  u8* data = stbi_load_from_memory((const u8*)buf.data(), (int)buf.size(), &x, &y, &n, 4);

  // create mesh from landscape
  u32 vertexFlags = VF_POS | VF_NORMAL;
  INIT(CreateBuffersFromBitmapFaceted(data, x, y, Vector3(10, 30, 10), &vertexFlags, &_landscapeGpuObjects));

  INIT(_landscapeGpuObjects.LoadShadersFromFile("shaders/out/landscape", "VsLandscape", nullptr, "PsLandscape", vertexFlags));
  INIT(_landscapeState.Create());

  INIT(_edgeGpuObjects.LoadShadersFromFile("shaders/out/landscape", "VsQuad", nullptr, "PsEdgeDetect"));
  INIT(_skyGpuObjects.LoadShadersFromFile("shaders/out/landscape", "VsQuad", nullptr, "PsSky"));

  INIT(_compositeGpuObjects.LoadShadersFromFile("shaders/out/landscape", "VsQuad", nullptr, "PsComposite"));
  INIT(_compositeState.Create());

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
  _camera.Update();
  Matrix view = _camera._view;
  Matrix proj = Matrix::CreatePerspectiveFieldOfView(XMConvertToRadians(45), 16/10.f, 0.1f, 2000.f);
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
bool Landscape::Render()
{
  rmt_ScopedCPUSample(Landscape_Render);
  static Color black(0, 0, 0, 0);
  PostProcess* postProcess = GRAPHICS.GetPostProcess();

  ScopedRenderTarget rt(DXGI_FORMAT_R16G16B16A16_FLOAT, BufferFlags(BufferFlag::CreateSrv) | BufferFlag::CreateDepthBuffer);

  u32 dimX, dimY;
  GRAPHICS.GetTextureSize(rt._handle, &dimX, &dimY);
  _cbPerFrame.dim.x = (float)dimX;
  _cbPerFrame.dim.y = (float)dimY;
  _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::VertexShader, 0);
  _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::PixelShader, 0);

  // Render the sky
  _ctx->SetRenderTarget(rt._handle, &black);
  _ctx->SetGpuObjects(_skyGpuObjects);
  _ctx->Draw(3, 0);

  // Render the background
  _ctx->SetRenderTarget(rt._handle, nullptr);
  _ctx->SetGpuObjects(_landscapeGpuObjects);
  _ctx->SetGpuState(_landscapeState);
  _ctx->Draw(_landscapeGpuObjects._numVerts, 0);

  // outline
  ScopedRenderTarget rtOutline(DXGI_FORMAT_R16G16B16A16_FLOAT);
  postProcess->Execute({ rt._handle }, rtOutline._handle, _edgeGpuObjects._ps, false);

  postProcess->Execute({ rt._handle, rtOutline._handle }, 
    GRAPHICS.GetBackBuffer(), _compositeGpuObjects._ps, false);

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
  if (ImGui::Button("Reset"))
    Reset();
}
#endif

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Landscape::SaveParameterSet()
{
  OutputBuffer buf;
  _settings.pos = _camera._pos;
  _settings.yaw = _camera._yaw;
  _settings.pitch = _camera._pitch;
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
