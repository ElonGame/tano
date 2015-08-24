#include "blob.hpp"
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
#include "../mesh_loader.hpp"
#include "../mesh_utils.hpp"

using namespace tano;
using namespace bristol;

//------------------------------------------------------------------------------
Blob::Blob(const string& name, const string& config, u32 id) : BaseEffect(name, config, id)
{
}

//------------------------------------------------------------------------------
Blob::~Blob()
{
}

//------------------------------------------------------------------------------
bool Blob::OnConfigChanged(const vector<char>& buf)
{
  return ParseBlobSettings(InputBuffer(buf), &_settings);
}

//------------------------------------------------------------------------------
bool Blob::Init()
{
  BEGIN_INIT_SEQUENCE();

  _camera.FromProtocol(_settings.camera);

  MeshLoader meshLoader;
  INIT(meshLoader.Load("gfx/displace_blob.boba"));
  CreateScene(meshLoader,
    SceneOptions().TransformToWorldSpace(),
    &_scene);

    // clang-format off
    INIT(_meshBundle.Create(BundleOptions()
      .RasterizerDesc(rasterizeDescCullNone)
      .InputElement(CD3D11_INPUT_ELEMENT_DESC("POSITION", DXGI_FORMAT_R32G32B32_FLOAT))
      .InputElement(CD3D11_INPUT_ELEMENT_DESC("NORMAL", DXGI_FORMAT_R32G32B32_FLOAT))
      .VertexShader("shaders/out/blob.mesh", "VsMesh")
      .PixelShader("shaders/out/blob.mesh", "PsMesh")));
    // clang-format on

  INIT(_cbMesh.Create());

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool Blob::Update(const UpdateState& state)
{
  UpdateCameraMatrix(state);
  return true;
}

//------------------------------------------------------------------------------
bool Blob::FixedUpdate(const FixedUpdateState& state)
{
  _camera.Update(state);
  return true;
}

//------------------------------------------------------------------------------
void Blob::UpdateCameraMatrix(const UpdateState& state)
{
  Matrix view = _camera._view;
  Matrix proj = _camera._proj;

  Matrix viewProj = view * proj;

  _cbMesh.vs0.viewProj = viewProj.Transpose();
  _cbMesh.vs1.objWorld = Matrix::Identity();
}

//------------------------------------------------------------------------------
bool Blob::Render()
{
  rmt_ScopedCPUSample(Blob_Render);

  static Color black(0, 0, 0, 0);

  _ctx->SetSwapChain(GRAPHICS.DefaultSwapChain(), black);
  RenderTargetDesc desc = GRAPHICS.GetBackBufferDesc();

  _ctx->SetBundle(_meshBundle);
  _cbMesh.Set(_ctx, 0);

  for (scene::MeshBuffer* buf : _scene.meshBuffers)
  {
    _ctx->SetVertexBuffer(buf->vb);
    _ctx->SetIndexBuffer(buf->ib);

    for (scene::Mesh* mesh : buf->meshes)
    {
      _cbMesh.Set(_ctx, 1);
      _ctx->DrawIndexed(mesh->indexCount, mesh->startIndexLocation, mesh->baseVertexLocation);
    }
  }

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
void Blob::SaveParameterSet(bool inc)
{
  _camera.ToProtocol(&_settings.camera);
  SaveSettings(_settings, inc);
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
BaseEffect* Blob::Create(const char* name, const char* config, u32 id)
{
  return new Blob(name, config, id);
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
