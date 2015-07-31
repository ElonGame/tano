#include "particle_trail.hpp"
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
#include "../fullscreen_effect.hpp"

using namespace tano;
using namespace bristol;

static int MAX_NUM_PARTICLES = 32 * 1024;

//------------------------------------------------------------------------------
ParticleTrail::ParticleTrail(const string &name, const string& config, u32 id)
  : BaseEffect(name, config, id)
{
#if WITH_IMGUI
  PROPERTIES.Register(Name(),
    bind(&ParticleTrail::RenderParameterSet, this),
    bind(&ParticleTrail::SaveParameterSet, this));

  PROPERTIES.SetActive(Name());
#endif
}

//------------------------------------------------------------------------------
ParticleTrail::~ParticleTrail()
{
}

//------------------------------------------------------------------------------
bool ParticleTrail::OnConfigChanged(const vector<char>& buf)
{
  return ParseParticleTrailSettings(InputBuffer(buf), &_settings);
}

//------------------------------------------------------------------------------
bool ParticleTrail::Init()
{
  BEGIN_INIT_SEQUENCE();

  // clang-format off
  INIT(_particleBundle.Create(BundleOptions()
    .VertexShader("shaders/out/trail.particle", "VsParticle")
    .GeometryShader("shaders/out/trail.particle", "GsParticle")
    .PixelShader("shaders/out/trail.particle", "PsParticle")
    .InputElement(CD3D11_INPUT_ELEMENT_DESC("POSITION", DXGI_FORMAT_R32G32B32_FLOAT))
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_POINTLIST)
    .DynamicVb(MAX_NUM_PARTICLES, sizeof(V3))
    .DepthStencilDesc(depthDescDepthDisabled)
    .BlendDesc(blendDescBlendOneOne)
    .RasterizerDesc(rasterizeDescCullNone)));

  INIT(_compositeBundle.Create(BundleOptions()
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/trail.composite", "PsComposite")));

  // clang-format on

  INIT_RESOURCE(_particleTexture, RESOURCE_MANAGER.LoadTexture("gfx/particle3.png"));

  INIT(_cbParticle.Create());
  INIT(_cbComposite.Create());

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool ParticleTrail::Update(const UpdateState& state)
{
  UpdateCameraMatrix(state);

  ObjectHandle handle = _particleBundle.objects._vb;
  V3* vtx = _ctx->MapWriteDiscard<V3>(handle);

  float h = 0.01f;

  float a = _settings.lorenz_a;
  float b = _settings.lorenz_b;
  float c = _settings.lorenz_c;

  float x0 = 0.1f;
  float y0 = 0;
  float z0 = 0;
  for (int i = 0; i < MAX_NUM_PARTICLES+100; i++)
  {
    float x1 = x0 + h * a * (y0 - x0);
    float y1 = y0 + h * (x0 * (b - z0) - y0);
    float z1 = z0 + h * (x0 * y0 - c * z0);
    x0 = x1;
    y0 = y1;
    z0 = z1;

    if (i >= 100)
    {
      *vtx = V3(x0, y0, z0);
      ++vtx;
    }
  }

  _ctx->Unmap(handle);
  return true;
}

//------------------------------------------------------------------------------
bool ParticleTrail::FixedUpdate(const FixedUpdateState& state)
{
  _camera.Update(state);
  return true;
}

//------------------------------------------------------------------------------
void ParticleTrail::UpdateCameraMatrix(const UpdateState& state)
{
  Matrix view = _camera._view;
  Matrix proj = _camera._proj;

  Matrix viewProj = view * proj;

  _cbParticle.gs0.world = Matrix::Identity();
  _cbParticle.gs0.viewProj = viewProj.Transpose();
  _cbParticle.gs0.cameraPos = _camera._pos;

}

//------------------------------------------------------------------------------
bool ParticleTrail::Render()
{
  rmt_ScopedCPUSample(ParticleTrail_Render);

  static Color black(0, 0, 0, 0);
  FullscreenEffect* fullscreen = GRAPHICS.GetFullscreenEffect();

  ScopedRenderTarget rtColor(DXGI_FORMAT_R16G16B16A16_FLOAT);
  {
    // particle
    _ctx->SetRenderTarget(rtColor, &black);

    _cbParticle.Set(_ctx, 0);
    _ctx->SetBundleWithSamplers(_particleBundle, ShaderType::PixelShader);
    _ctx->SetShaderResource(_particleTexture);
    _ctx->Draw(MAX_NUM_PARTICLES, 0);
  }

  {
    // composite
    _cbComposite.ps0.tonemap = Vector2(_settings.tonemap.exposure, _settings.tonemap.min_white);
    _cbComposite.Set(_ctx, 0);

    ObjectHandle inputs[] = { rtColor };
    fullscreen->Execute(inputs,
      1,
      GRAPHICS.GetBackBuffer(),
      GRAPHICS.GetBackBufferDesc(),
      GRAPHICS.GetDepthStencil(),
      _compositeBundle.objects._ps,
      false);
  }

  return true;
}

//------------------------------------------------------------------------------
#if WITH_IMGUI
void ParticleTrail::RenderParameterSet()
{
  ImGui::SliderFloat("a", &_settings.lorenz_a, -20, 20);
  ImGui::SliderFloat("b", &_settings.lorenz_b, -50, 50);
  ImGui::SliderFloat("c", &_settings.lorenz_c, -10, 10);

  ImGui::Separator();
  ImGui::SliderFloat("Exposure", &_settings.tonemap.exposure, 0.1f, 20.0f);
  ImGui::SliderFloat("Min White", &_settings.tonemap.min_white, 0.1f, 20.0f);

  if (ImGui::Button("Reset"))
    Reset();
}
#endif

//------------------------------------------------------------------------------
#if WITH_IMGUI
void ParticleTrail::SaveParameterSet()
{
  SaveSettings(_settings);
}
#endif

//------------------------------------------------------------------------------
void ParticleTrail::Reset()
{
  _camera._pos = Vector3(0.f, 0.f, 0.f);
  _camera._pitch = _camera._yaw = _camera._roll = 0.f;
}

//------------------------------------------------------------------------------
bool ParticleTrail::Close()
{
  return true;
}

//------------------------------------------------------------------------------
BaseEffect* ParticleTrail::Create(const char* name, const char* config, u32 id)
{
  return new ParticleTrail(name, config, id);
}

//------------------------------------------------------------------------------
const char* ParticleTrail::Name()
{
  return "particle_trail";
}

//------------------------------------------------------------------------------
void ParticleTrail::Register()
{
  DEMO_ENGINE.RegisterFactory(Name(), ParticleTrail::Create);
}
