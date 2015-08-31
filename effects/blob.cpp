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
#include "../fullscreen_effect.hpp"

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
  INIT_FATAL(meshLoader.Load("gfx/displace_blob.boba"));
  CreateScene(meshLoader,
    SceneOptions().TransformToWorldSpace(),
    &_scene);

    // clang-format off
  INIT_FATAL(_compositeBundle.Create(BundleOptions()
    //.BlendDesc(blendDescWeightedBlendResolve)
    .DepthStencilDesc(depthDescDepthDisabled)
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/blob.compose", "PsComposite")));

  INIT_FATAL(_meshBundle.Create(BundleOptions()
    .DepthStencilDesc(depthDescDepthWriteDisabled)
    .BlendDesc(blendDescWeightedBlend)
    .InputElement(CD3D11_INPUT_ELEMENT_DESC("POSITION", DXGI_FORMAT_R32G32B32_FLOAT))
    .InputElement(CD3D11_INPUT_ELEMENT_DESC("NORMAL", DXGI_FORMAT_R32G32B32_FLOAT))
    .VertexShader("shaders/out/blob.mesh", "VsMesh")
    .PixelShader("shaders/out/blob.mesh", "PsMesh")));
  // clang-format on

  INIT(_cbMesh.Create());
  INIT(_cbComposite.Create());

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

  _cbMesh.vs0.view = view.Transpose();
  _cbMesh.vs0.viewProj = viewProj.Transpose();
  _cbMesh.ps0.cameraPos = _camera._pos;

  _cbMesh.vs1.objWorld = Matrix::Identity();
}

//------------------------------------------------------------------------------
bool Blob::Render()
{
  rmt_ScopedCPUSample(Blob_Render);

  static Color black(0, 0, 0, 0);
  FullscreenEffect* fullscreen = GRAPHICS.GetFullscreenEffect();

  _ctx->SetSwapChain(GRAPHICS.DefaultSwapChain(), black);
  RenderTargetDesc desc = GRAPHICS.GetBackBufferDesc();

  ScopedRenderTarget rtColor(DXGI_FORMAT_R16G16B16A16_FLOAT);
  _ctx->SetRenderTarget(rtColor, GRAPHICS.GetDepthStencil(), &black);

  ScopedRenderTarget rtOpacity(DXGI_FORMAT_R32G32B32A32_FLOAT);
  ScopedRenderTarget rtRevealage(DXGI_FORMAT_R32G32B32A32_FLOAT);

  const Color* clearColors[] = {&Color(0, 0, 0, 0), &Color(1, 1, 1, 1)};
  ObjectHandle targets[] = {rtOpacity, rtRevealage};
  _ctx->SetRenderTargets(targets, 2, GRAPHICS.GetDepthStencil(), clearColors);

  {
    // meshes
    _ctx->SetBundle(_meshBundle);
    _cbMesh.Set(_ctx, 0);

    for (scene::MeshBuffer* buf : _scene.meshBuffers)
    {
      _ctx->SetVertexBuffer(buf->vb);
      _ctx->SetIndexBuffer(buf->ib);

      int e = (int)buf->meshes.size();
      //int e = 1;
      for (int i = 0; i < e; ++i)
      {
        scene::Mesh* mesh = buf->meshes[i];

        // colors are pre-mulitplied alpha
        Vector4 meshColors[] = { Vector4(0, 0, 0.25f, 0.25f), Vector4(0, 0.5f, 0, 0.5f), Vector4(0.75f, 0, 0, 0.75f) };
        _cbMesh.ps1.col = meshColors[i % 3];
        _cbMesh.Set(_ctx, 1);
        _ctx->DrawIndexed(mesh->indexCount, mesh->startIndexLocation, mesh->baseVertexLocation);
      }
    }
  }
  _ctx->UnsetRenderTargets(0, 2);

  {
    // composite
    _ctx->SetBundleWithSamplers(_compositeBundle, ShaderType::PixelShader);

    _cbComposite.ps0.tonemap = Vector2(_settings.tonemap.exposure, _settings.tonemap.min_white);
    _cbComposite.Set(_ctx, 0);

    ObjectHandle inputs[] = { rtOpacity, rtRevealage };
    fullscreen->Execute(inputs,
      2,
      GRAPHICS.GetBackBuffer(),
      GRAPHICS.GetBackBufferDesc(),
      GRAPHICS.GetDepthStencil(),
      _compositeBundle.objects._ps,
      false,
      false,
      &Color(0.1f,0.1f,0.1f,0));
  }


  return true;
}

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Blob::RenderParameterSet()
{
  ImGui::SliderFloat("Exposure", &_settings.tonemap.exposure, 0.1f, 2.0f);
  ImGui::SliderFloat("Min White", &_settings.tonemap.min_white, 0.1f, 2.0f);
  ImGui::Separator();

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
