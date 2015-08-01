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

void Taily::AddPos(const V3& pos)
{
  // copy out old cur
  tail[writePos] = cur;
  writePos = (writePos + 1) % MAX_TAIL_LENGTH;
  tailLength = min((int)MAX_TAIL_LENGTH, tailLength + 1);

  cur = pos;
}

V3* Taily::CopyOut(V3* buf)
{
  if (tailLength < MAX_TAIL_LENGTH)
  {
    memcpy((void*)buf, tail, tailLength * sizeof(V3));
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
  memcpy(buf, tail + writePos, n * sizeof(V3));
  memcpy(buf + n, tail, writePos * sizeof(V3));
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
  bool res = ParseParticleTrailSettings(InputBuffer(buf), &_settings);
  _camera.FromProtocol(_settings.camera);
  return res;
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
V3 LorenzUpdate(float a, float b, float c, float h, const V3& prev)
{
  return V3{prev.x + h * a * (prev.y - prev.x),
      prev.y + h * (prev.x * (b - prev.z) - prev.y),
      prev.z + h * (prev.x * prev.y - c * prev.z)};
}

//------------------------------------------------------------------------------
bool ParticleTrail::Update(const UpdateState& state)
{
  UpdateCameraMatrix(state);

  ObjectHandle handle = _particleBundle.objects._vb;
  V3* vtx = _ctx->MapWriteDiscard<V3>(handle);

  float h = 0.01f;
  h = state.delta.TotalSecondsAsFloat();
  float a = _settings.lorenz_a;
  float b = _settings.lorenz_b;
  float c = _settings.lorenz_c;

  _taily.AddPos(LorenzUpdate(a, b, c, h, _taily.cur));
  _taily.CopyOut(vtx);

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

    _cbParticle.gs0.numParticles.x = (float)_taily.tailLength + 1;
    _cbParticle.Set(_ctx, 0);
    _ctx->SetBundleWithSamplers(_particleBundle, ShaderType::PixelShader);
    _ctx->SetShaderResource(_particleTexture);
    _ctx->Draw(_taily.tailLength + 1, 0);
  }

  {
    // composite
    _cbComposite.ps0.tonemap = Vector2(_settings.tonemap.exposure, _settings.tonemap.min_white);
    _cbComposite.Set(_ctx, 0);

    ObjectHandle inputs[] = {rtColor};
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
