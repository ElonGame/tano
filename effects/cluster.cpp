#include "cluster.hpp"
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
#include "../tano_math.hpp"

using namespace tano;
using namespace bristol;

namespace
{
  float angle = 0;
  float height = 0;
  float distance = 500;
}

//------------------------------------------------------------------------------
Cluster::Cluster(const string &name, u32 id)
  : BaseEffect(name, id)
  , _blinkFace("blinkface")
{
#if WITH_IMGUI
  PROPERTIES.Register("cluster",
    bind(&Cluster::RenderParameterSet, this),
    bind(&Cluster::SaveParameterSet, this));

  PROPERTIES.SetActive("cluster");
#endif
}

//------------------------------------------------------------------------------
Cluster::~Cluster()
{
}

//------------------------------------------------------------------------------
bool Cluster::Init(const char* configFile)
{
  BEGIN_INIT_SEQUENCE();

  _configName = configFile;
  vector<char> buf;
  if (!RESOURCE_MANAGER.LoadFile(configFile, &buf))
    return false;

  INIT(ParseClusterSettings(InputBuffer(buf), &_settings));

  INIT(_meshLoader.Load("meshes/blob1.boba"));
  u32 vertexFlags;
  INIT(CreateBuffersFromMeshFaceted(_meshLoader, "Cube", &vertexFlags, &_clusterGpuObjects));
  INIT(_clusterGpuObjects.LoadVertexShader("shaders/out/cluster", "VsMesh", vertexFlags));
  INIT(_clusterGpuObjects.LoadPixelShader("shaders/out/cluster", "PsMesh"));
  INIT(_clusterState.Create());

  int w, h;
  INIT(_cbPerFrame.Create());
  GRAPHICS.GetBackBufferSize(&w, &h);

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool Cluster::Update(const UpdateState& state)
{
  rmt_ScopedCPUSample(Cluster_Update);

  UpdateCameraMatrix(state);

  return true;
}

//------------------------------------------------------------------------------
void Cluster::UpdateCameraMatrix(const UpdateState& state)
{
  float x = distance * sin(angle);
  float y = height;
  float z = distance * cos(angle);

  Vector3 pos = Vector3(x, y, z);
  Vector3 target = Vector3(0, 0, 0);
  Vector3 dir = Normalize(target - pos);

  Matrix view = Matrix::CreateLookAt(pos, Vector3(0, 0, 0), Vector3(0, 1, 0));
  Matrix proj = Matrix::CreatePerspectiveFieldOfView(XMConvertToRadians(45), 16/10.f, 0.1f, 2000.f);
  Matrix viewProj = view * proj;

  _cbPerFrame.world = Matrix::Identity();
  _cbPerFrame.view = view.Transpose();
  _cbPerFrame.proj = proj.Transpose();
  _cbPerFrame.viewProj = viewProj.Transpose();
}

//------------------------------------------------------------------------------
bool Cluster::Render()
{
  _ctx->SetSwapChain(GRAPHICS.DefaultSwapChain(), Color(0.1f, 0.1f, 0.1f, 0));
  _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::VertexShader, 0);

  rmt_ScopedCPUSample(Cluster_Render);

  _ctx->SetGpuObjects(_clusterGpuObjects);
  _ctx->SetGpuState(_clusterState);
  _ctx->DrawIndexed(_clusterGpuObjects._numIndices, 0, 0);

  return true;
}

//------------------------------------------------------------------------------
bool Cluster::InitAnimatedParameters()
{
  return true;
}

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Cluster::RenderParameterSet()
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
void Cluster::SaveParameterSet()
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
void Cluster::Reset()
{
}

//------------------------------------------------------------------------------
bool Cluster::Close()
{
  return true;
}

//------------------------------------------------------------------------------
BaseEffect* Cluster::Create(const char* name, u32 id)
{
  return new Cluster(name, id);
}

//------------------------------------------------------------------------------
const char* Cluster::Name()
{
  return "cluster";
}

//------------------------------------------------------------------------------
void Cluster::Register()
{
  DEMO_ENGINE.RegisterFactory(Name(), Cluster::Create);
}
