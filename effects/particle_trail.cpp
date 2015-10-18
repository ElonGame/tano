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
#include "../fullscreen_effect.hpp"

using namespace tano;
using namespace bristol;

static int MAX_NUM_PARTICLES = 32 * 1024;

//------------------------------------------------------------------------------
void Taily::AddPos(const vec3& pos)
{
  // copy out old cur
  tail[writePos] = cur;
  writePos = (writePos + 1) % MAX_TAIL_LENGTH;
  tailLength = min((int)MAX_TAIL_LENGTH, tailLength + 1);

  cur = pos;
}

//------------------------------------------------------------------------------
vec3* Taily::CopyOut(vec3* buf)
{
  if (tailLength < MAX_TAIL_LENGTH)
  {
    memcpy((void*)buf, tail, tailLength * sizeof(vec3));
    buf += tailLength;
    *buf++ = cur;
    return buf;
  }

  // the buffer has looped, so we need 2 copies:
  // - first from write pos -> end
  // - then from start -> write pos

  // 8 9 3 4 5 6 7
  // writePos = 2
  // MAX_TAIL_LENGTH = 7
  int n = MAX_TAIL_LENGTH - writePos;
  memcpy(buf, tail + writePos, n * sizeof(vec3));
  memcpy(buf + n, tail, writePos * sizeof(vec3));
  *(buf + MAX_TAIL_LENGTH) = cur;
  return buf + MAX_TAIL_LENGTH + 1;
}

//------------------------------------------------------------------------------
ParticleTrail::ParticleTrail(const string& name, const string& config, u32 id) : BaseEffect(name, config, id)
{
}

//------------------------------------------------------------------------------
ParticleTrail::~ParticleTrail()
{
}

//------------------------------------------------------------------------------
bool ParticleTrail::OnConfigChanged(const vector<char>& buf)
{
  _settings = ParseParticleTrailSettings(InputBuffer(buf));
  _camera.FromProtocol(_settings.camera);
  return true;
}

//------------------------------------------------------------------------------
bool ParticleTrail::Init()
{
  BEGIN_INIT_SEQUENCE();

  // clang-format off
  INIT(_backgroundBundle.Create(BundleOptions()
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/trail.background", "PsBackground")));

  INIT(_particleBundle.Create(BundleOptions()
    .VertexShader("shaders/out/trail.particle", "VsParticle")
    .GeometryShader("shaders/out/trail.particle", "GsParticle")
    .PixelShader("shaders/out/trail.particle", "PsParticle")
    .InputElement(CD3D11_INPUT_ELEMENT_DESC("POSITION", DXGI_FORMAT_R32G32B32_FLOAT))
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_POINTLIST)
    .DynamicVb(MAX_NUM_PARTICLES, sizeof(vec3))
    .DepthStencilDesc(depthDescDepthDisabled)
    .BlendDesc(blendDescBlendOneOne)
    .RasterizerDesc(rasterizeDescCullNone)));

  INIT(_compositeBundle.Create(BundleOptions()
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/trail.composite", "PsComposite")));

  INIT(_lineBundle.Create(BundleOptions()
    .VertexShader("shaders/out/trail.lines", "VsLines")
    .GeometryShader("shaders/out/trail.lines", "GsLines")
    .PixelShader("shaders/out/trail.lines", "PsLines")
    .InputElement(CD3D11_INPUT_ELEMENT_DESC("POSITION", DXGI_FORMAT_R32G32B32_FLOAT))
    .RasterizerDesc(rasterizeDescCullNone)
    .BlendDesc(blendDescBlendOneOne)
    .DepthStencilDesc(depthDescDepthDisabled)
    .DynamicVb(128 * 1024, sizeof(vec3))
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_LINESTRIP)));

  // clang-format on

  INIT_RESOURCE(_particleTexture, RESOURCE_MANAGER.LoadTexture("gfx/particle3.png"));

  INIT(_cbParticle.Create());
  INIT(_cbComposite.Create());
  INIT(_cbPlexus.Create());
  INIT(_cbBackground.Create());

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
vec3 LorenzUpdate(float a, float b, float c, float h, const vec3& prev)
{
  return vec3{prev.x + h * a * (prev.y - prev.x),
      prev.y + h * (prev.x * (b - prev.z) - prev.y),
      prev.z + h * (prev.x * prev.y - c * prev.z)};
}

//------------------------------------------------------------------------------
bool ParticleTrail::Update(const UpdateState& state)
{
  UpdateCameraMatrix(state);

  {
    ObjectHandle handle = _particleBundle.objects._vb;
    vec3* vtx = _ctx->MapWriteDiscard<vec3>(handle);
    _taily.CopyOut(vtx);
    _ctx->Unmap(handle);
  }

  {
    ObjectHandle handle = _lineBundle.objects._vb;
    vec3* vtx = _ctx->MapWriteDiscard<vec3>(handle);
    _taily.CopyOut(vtx);
    _ctx->Unmap(handle);
  }

  _cbBackground.ps0.upper = g_Blackboard->GetVec4Var("particle_trail.upper");
  _cbBackground.ps0.lower = g_Blackboard->GetVec4Var("particle_trail.lower");
  return true;
}

//------------------------------------------------------------------------------
bool ParticleTrail::FixedUpdate(const FixedUpdateState& state)
{
  float h = state.delta;
  float a = _settings.lorenz_a;
  float b = _settings.lorenz_b;
  float c = _settings.lorenz_c;

  _taily.AddPos(LorenzUpdate(a, b, c, h, _taily.cur));

  _camera.Update(state.delta);
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

  _cbPlexus.gs0.world = Matrix::Identity();
  _cbPlexus.gs0.viewProj = viewProj.Transpose();
  _cbPlexus.gs0.cameraPos = _camera._pos;
}

//------------------------------------------------------------------------------
bool ParticleTrail::Render()
{
  rmt_ScopedCPUSample(ParticleTrail_Render);

  static Color black(0, 0, 0, 0);
  FullscreenEffect* fullscreen = g_Graphics->GetFullscreenEffect();

  ScopedRenderTarget rtColor(DXGI_FORMAT_R16G16B16A16_FLOAT);

  _ctx->SetRenderTarget(rtColor, &black);

  {
    // Render the background
    _cbBackground.Set(_ctx, 0);
    _ctx->SetRenderTarget(rtColor, g_Graphics->GetDepthStencil(), &black);
    _ctx->SetBundle(_backgroundBundle);
    _ctx->Draw(3, 0);
  }

#if 0
  {
    // particle
    _cbParticle.gs0.numParticles.x = (float)_taily.tailLength + 1;
    _cbParticle.Set(_ctx, 0);
    _ctx->SetBundleWithSamplers(_particleBundle, ShaderType::PixelShader);
    _ctx->SetShaderResource(_particleTexture);
    _ctx->Draw(_taily.tailLength + 1, 0);
  }
#else
    {
      // lines
      RenderTargetDesc desc = g_Graphics->GetBackBufferDesc();
      _cbPlexus.gs0.dim = vec4((float)desc.width, (float)desc.height, 0, 0);
      vec3 params = g_Blackboard->GetVec3Var("particle_trail.lineParams");
      _cbPlexus.ps0.lineParams = vec4(params.x, params.y, params.z, 1);
      _cbPlexus.Set(_ctx, 0);
      _ctx->SetBundle(_lineBundle);
      _ctx->Draw(_taily.tailLength-1, 0);
    }
#endif

  {
    // composite
    _cbComposite.ps0.tonemap = vec2(_settings.tonemap.exposure, _settings.tonemap.min_white);
    _cbComposite.Set(_ctx, 0);

    ObjectHandle inputs[] = {rtColor};
    fullscreen->Execute(inputs,
        1,
        g_Graphics->GetBackBuffer(),
        g_Graphics->GetBackBufferDesc(),
        g_Graphics->GetDepthStencil(),
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
  ImGui::SliderFloat("Exposure", &_settings.tonemap.exposure, 0.1f, 2.0f);
  ImGui::SliderFloat("Min White", &_settings.tonemap.min_white, 0.1f, 2.0f);

  if (ImGui::Button("Reset"))
    Reset();
}
#endif

//------------------------------------------------------------------------------
#if WITH_IMGUI
void ParticleTrail::SaveParameterSet(bool inc)
{
  _camera.ToProtocol(&_settings.camera);
  SaveSettings(_settings, inc);
}
#endif

//------------------------------------------------------------------------------
void ParticleTrail::Reset()
{
  _camera._pos = vec3(0.f, 0.f, 0.f);
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
  g_DemoEngine->RegisterFactory(Name(), ParticleTrail::Create);
}
