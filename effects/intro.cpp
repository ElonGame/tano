#include "intro.hpp"
#include "../tano.hpp"
#include "../graphics.hpp"
#include "../graphics_extra.hpp"
#include "../graphics_context.hpp"
#include "../demo_engine.hpp"
#include "../resource_manager.hpp"
#include "../init_sequence.hpp"
#include "../generated/demo.parse.hpp"
#include "../mesh_loader.hpp"
#include "../fullscreen_effect.hpp"
#include "../scheduler.hpp"
#include "../arena_allocator.hpp"
#include "../stop_watch.hpp"
#include "../blackboard.hpp"
#include "../mesh_utils.hpp"
#include "../tano_math_convert.hpp"
#include "../vertex_types.hpp"

using namespace tano;
using namespace tano::scheduler;
using namespace bristol;
using namespace DirectX;

//------------------------------------------------------------------------------
namespace
{
  float angle = 0;
  float height = 0;
  float distance = 1300;
  bool extended = true;

  static int MAX_NUM_PARTICLES = 10000;
  static int VB_FRAMES = 3;
  static int VB_INDEX = 0;
}

//------------------------------------------------------------------------------
Intro::Intro(const string& name, const string& config, u32 id) : BaseEffect(name, config, id)
{
}

//------------------------------------------------------------------------------
Intro::~Intro()
{
  for (int i = 0; i < _particleEmitters.Size(); ++i)
  {
    _particleEmitters[i].Destroy();
  }
}

//------------------------------------------------------------------------------
bool Intro::OnConfigChanged(const vector<char>& buf)
{
  _settings = ParseIntroSettings(InputBuffer(buf));
  return true;
}

//------------------------------------------------------------------------------
vec4 ColorToVector4(const Color& c)
{
  return vec4(c.x, c.y, c.z, c.w);
}

//------------------------------------------------------------------------------
bool Intro::Init()
{
  BEGIN_INIT_SEQUENCE();

  _freeflyCamera.FromProtocol(_settings.camera);

  // clang-format off
  INIT(_backgroundBundle.Create(BundleOptions()
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/intro.background", "PsBackground")));

  INIT(_particleBundle.Create(BundleOptions()
    .VertexShader("shaders/out/intro.particle", "VsParticle")
    .GeometryShader("shaders/out/intro.particle", "GsParticle")
    .PixelShader("shaders/out/intro.particle", "PsParticle")
    .InputElement(CD3D11_INPUT_ELEMENT_DESC("POSITION", DXGI_FORMAT_R32G32B32A32_FLOAT))
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_POINTLIST)
    .DynamicVb(VB_FRAMES * MAX_NUM_PARTICLES, sizeof(vec4))
    .DepthStencilDesc(depthDescDepthDisabled)
    .BlendDesc(blendDescPreMultipliedAlpha)
    .RasterizerDesc(rasterizeDescCullNone)));

  INIT(_compositeBundle.Create(BundleOptions()
    .PixelShader("shaders/out/intro.composite", "PsComposite")));

  INIT(_textBundle.Create(BundleOptions()
    .PixelShader("shaders/out/intro.text", "PsIntroTextDistort")));

  // clang-format on

  INIT_RESOURCE_FATAL(_particleTexture, RESOURCE_MANAGER.LoadTexture(_settings.texture.c_str()));
  INIT_RESOURCE_FATAL(
      _csParticleBlur, g_Graphics->LoadComputeShaderFromFile("shaders/out/intro.blur", "BoxBlurY"));

  // Create default emitter
  for (int i = 0; i < 20; ++i)
  {
    _particleEmitters.Append(RadialParticleEmitter())
        .Create(vec3(0, 0, 0), 25.f * (i + 1), _settings.num_particles);
  }

  // Generic setup
  INIT(_cbBackground.Create());
  INIT(_cbComposite.Create());
  INIT(_cbText.Create());
  INIT(_cbParticle.Create());
  _cbBackground.ps0.inner = ColorToVector4(_settings.inner_color);
  _cbBackground.ps0.outer = ColorToVector4(_settings.outer_color);

  INIT_RESOURCE_FATAL(_introTexture[0].h,
    RESOURCE_MANAGER.LoadTexture("gfx/intro_1.png", false, &_introTexture[0].info));
  INIT_RESOURCE_FATAL(_introTexture[1].h,
    RESOURCE_MANAGER.LoadTexture("gfx/intro_2.png", false, &_introTexture[1].info));
  INIT_RESOURCE_FATAL(_introTexture[2].h,
    RESOURCE_MANAGER.LoadTexture("gfx/intro_3.png", false, &_introTexture[2].info));

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
void Intro::UpdateCameraMatrix(const UpdateState& state)
{
  Matrix view = _freeflyCamera._view;
  Matrix proj = _freeflyCamera._proj;
  Matrix viewProj = view * proj;

  _cbParticle.gs0.world = Matrix::Identity();
  _cbParticle.gs0.viewProj = viewProj.Transpose();

  {
    float x = distance * sin(angle);
    float z = -distance * cos(angle);
    vec3 pos = vec3(x, height, z);
    vec3 target = vec3(0, 0, 0);
    vec3 dir = Normalize(target - pos);
   
    int w, h;
    g_Graphics->GetBackBufferSize(&w, &h);
    float aspect = (float)w / h;
    Matrix view = Matrix::CreateLookAt(ToVector3(pos), Vector3(0, 0, 0), Vector3(0, 1, 0));
    Matrix proj = Matrix::CreatePerspectiveFieldOfView(XMConvertToRadians(45), aspect, 0.1f, 3000.f);
    Matrix viewProj = view * proj;
  }
}

//------------------------------------------------------------------------------
bool Intro::Update(const UpdateState& state)
{
  rmt_ScopedCPUSample(Intro_Update);

  static AvgStopWatch stopWatch;
  stopWatch.Start();

  UpdateParticleEmitters(state.delta.TotalSecondsAsFloat());

  _curTime = state.localTime.TotalMicroseconds() / (float)1e6;

  UpdateCameraMatrix(state);

  float ms = state.localTime.TotalMicroseconds() / (float)1e6;

  _cbText.ps0.time = ms;

  _cbComposite.ps0.time.x = ms;
  _cbComposite.ps0.tonemap = vec4(1, 1, 0, 0);

  CopyOutParticleEmitters();

  double avg = stopWatch.Stop();
#if WITH_IMGUI
  TANO.AddPerfCallback([=]()
      {
        ImGui::Text("Update time: %.3fms", 1000 * avg);
      });
#endif
  return true;
}

//------------------------------------------------------------------------------
void Intro::UpdateParticleEmitters(float dt)
{
  rmt_ScopedCPUSample(Intro_UpdateParticleEmitters);

  SimpleAppendBuffer<TaskId, 32> tasks;

  typedef RadialParticleEmitter::EmitterKernelData EmitterKernelData;

  for (int i = 0; i < _particleEmitters.Size(); ++i)
  {
    EmitterKernelData* data = g_ScratchMemory.Alloc<EmitterKernelData>(1);
    *data = EmitterKernelData{&_particleEmitters[i], dt, nullptr};
    KernelData kd;
    kd.data = data;
    kd.size = sizeof(EmitterKernelData);
    tasks.Append(g_Scheduler->AddTask(kd, RadialParticleEmitter::UpdateEmitter));
  }

  for (const TaskId& taskId : tasks)
    g_Scheduler->Wait(taskId);
}

//------------------------------------------------------------------------------
void Intro::CopyOutParticleEmitters()
{
  rmt_ScopedCPUSample(Intro_CopyOutParticleEmitters);

  typedef RadialParticleEmitter::EmitterKernelData EmitterKernelData;

  ObjectHandle vb = _particleBundle.objects._vb;

  D3D11_MAPPED_SUBRESOURCE res;
  D3D11_MAP m = D3D11_MAP_WRITE_NO_OVERWRITE;
  if (VB_INDEX == VB_FRAMES)
  {
    VB_INDEX = 0;
    m = D3D11_MAP_WRITE_DISCARD;
  }

  HRESULT hr = _ctx->Map(_particleBundle.objects._vb, 0, m, 0, &res);
  //vec4* vtx = _ctx->MapWriteDiscard<vec4>(_particleBundle.objects._vb);
  vec4* vtx = (vec4*)res.pData;//  _ctx->MapWriteDiscard<vec4>(_particleBundle.objects._vb);

  SimpleAppendBuffer<TaskId, 32> tasks;

  _numSpawnedParticles = 0;
  for (int i = 0; i < _particleEmitters.Size(); ++i)
  {
    EmitterKernelData* data = g_ScratchMemory.Alloc<EmitterKernelData>(1);
    *data = EmitterKernelData{&_particleEmitters[i],
        0,
        vtx + VB_INDEX * MAX_NUM_PARTICLES + i * _particleEmitters[i]._spawnedParticles};
    KernelData kd;
    kd.data = data;
    kd.size = sizeof(ParticleEmitter::EmitterKernelData);

    tasks.Append(g_Scheduler->AddTask(kd, RadialParticleEmitter::CopyOutEmitter));

    _numSpawnedParticles += _particleEmitters[i]._spawnedParticles;
  }

  for (const TaskId& taskId : tasks)
    g_Scheduler->Wait(taskId);

  _ctx->Unmap(vb);
}

//------------------------------------------------------------------------------
bool Intro::FixedUpdate(const FixedUpdateState& state)
{
  rmt_ScopedCPUSample(Intro_FixedUpdate);

  float ms = state.localTime.TotalMicroseconds() / (float)1e6;

  float dt = state.delta;

  float zz = -1000;
  _textCamera._pos.z = zz;
  _textCamera._fov = atan(100.f / fabsf(zz));
  _textCamera.Update(state.delta);

  return true;
}

//------------------------------------------------------------------------------
void SetQuadCoords(Pos4Tex* vtx, const vec2& topLeft, const vec2& bottomRight)
{
  // convert from 0..1 -> -1..1
  vec2 tl = { -1 + 2 * topLeft.x, 1 - 2 * topLeft.y };
  vec2 br = { -1 + 2 * bottomRight.x, 1 - 2 * bottomRight.y };

  // 0--1
  // 2 -3
  Pos4Tex v0 = { vec4{ tl.x, tl.y, 0, 1 }, vec2{ 0, 0 } };
  Pos4Tex v1 = { vec4{ br.x, tl.y, 0, 1 }, vec2{ 1, 0 } };
  Pos4Tex v2 = { vec4{ tl.x, br.y, 0, 1 }, vec2{ 0, 1 } };
  Pos4Tex v3 = { vec4{ br.x, br.y, 0, 1 }, vec2{ 1, 1 } };

  // 0, 1, 2
  // 2, 1, 3
  *vtx++ = v0;
  *vtx++ = v1;
  *vtx++ = v2;

  *vtx++ = v2;
  *vtx++ = v1;
  *vtx++ = v3;
}

//------------------------------------------------------------------------------
bool Intro::Render()
{
  rmt_ScopedCPUSample(Intro_Render);

  static Color black(0, 0, 0, 0);

  FullscreenEffect* fullscreen = g_Graphics->GetFullscreenEffect();

  ScopedRenderTarget rtColor(DXGI_FORMAT_R11G11B10_FLOAT);
  {
    // Render the background
     rmt_ScopedD3D11Sample(Background);

    _cbBackground.Set(_ctx, 0);
    _ctx->SetRenderTarget(rtColor, g_Graphics->GetDepthStencil(), &black);
    _ctx->SetBundle(_backgroundBundle);
    _ctx->Draw(3, 0);
  }

  {
    // Render particles
    rmt_ScopedD3D11Sample(Particles);

    _cbParticle.Set(_ctx, 0);
    _ctx->SetBundleWithSamplers(_particleBundle, PixelShader);
    _ctx->SetShaderResource(_particleTexture);
    _ctx->Draw(_numSpawnedParticles, VB_INDEX * MAX_NUM_PARTICLES);
  }

  //ScopedRenderTarget rtText(DXGI_FORMAT_R16G16B16A16_FLOAT);
  //ScopedRenderTarget rtTextDistort(DXGI_FORMAT_R16G16B16A16_FLOAT);
  ScopedRenderTarget rtText(DXGI_FORMAT_R11G11B10_FLOAT); // 16G16B16A16_FLOAT);
  ScopedRenderTarget rtTextDistort(DXGI_FORMAT_R11G11B10_FLOAT);
  if (true) {
    // text
    rmt_ScopedD3D11Sample(Text);

    _ctx->SetRenderTarget(rtText, g_Graphics->GetDepthStencil(), &black);

    float right = g_Blackboard->GetFloatVar("intro.textRight");
    float s = g_Blackboard->GetFloatVar("intro.textSize");
    float y0 = g_Blackboard->GetFloatVar("intro.newText0Pos");
    float y1 = g_Blackboard->GetFloatVar("intro.newText1Pos");
    float y2 = g_Blackboard->GetFloatVar("intro.newText2Pos");

    float ar0 = _introTexture[0].info.Width / (float)_introTexture[0].info.Height;
    float ar1 = _introTexture[1].info.Width / (float)_introTexture[1].info.Height;
    float ar2 = _introTexture[2].info.Width / (float)_introTexture[2].info.Height;

    bool rightAlign = true;
    if (rightAlign)
    {
      eval::Environment env;
      env.constants["t"] = _curTime;
      float bb = g_Blackboard->GetExpr("intro.text0Brightness", &env);
      fullscreen->RenderTexture(
          _introTexture[0].h, vec2{right - ar0 * s, y0 - s}, vec2{right, y0}, bb);

      fullscreen->RenderTexture(
          _introTexture[1].h, vec2{right - ar1 * s, y1 - s}, vec2{right, y1}, bb);

      fullscreen->RenderTexture(
          _introTexture[2].h, vec2{right - ar2 * s, y2 - s}, vec2{right, y2}, bb);

      _cbText.ps0.brightness = g_Blackboard->GetExpr("intro.distortBrightness", &env);
      _cbText.Set(_ctx, 0);
      ObjectHandle inputs[] = { rtText };
      fullscreen->Execute(inputs,
        1,
        rtTextDistort,
        rtTextDistort._desc,
        ObjectHandle(),
        _textBundle.objects._ps,
        true,
        true,
        &black);
    }
    else
    {
      // center
      float w0 = ar0 * s;
      fullscreen->RenderTexture(
        _introTexture[0].h, vec2{ 0.5f - w0 / 2, 0.5f - s / 2 }, vec2{ 0.5f + w0 / 2, 0.5f + s / 2 });
    }

  }

  ScopedRenderTarget rtBlurText(rtText._desc, BufferFlag::CreateSrv | BufferFlag::CreateUav);

  float beatHi = g_Blackboard->GetFloatVar("Beat-Hi", _curTime);

  {
    // blur
    _ctx->UnsetRenderTargets(0, 1);
    fullscreen->Blur(rtTextDistort, rtBlurText, rtBlurText._desc, 20, 1);
  }

  ScopedRenderTarget rtBlur2(rtColor._desc, BufferFlag::CreateSrv | BufferFlag::CreateUav);
  {
    // blur
    _ctx->UnsetRenderTargets(0, 1);
    fullscreen->BlurVertCustom(rtColor, rtBlur2, rtBlur2._desc, _csParticleBlur, 20, 1);
  }

  ScopedRenderTarget rtLines(DXGI_FORMAT_R16G16B16A16_FLOAT);
  ScopedRenderTarget rtBlur(
    DXGI_FORMAT_R16G16B16A16_FLOAT, BufferFlags(BufferFlag::CreateSrv | BufferFlag::CreateUav));

  ScopedRenderTarget rtCompose(DXGI_FORMAT_R16G16B16A16_FLOAT, BufferFlag::CreateSrv);
  {
    // composite
    _cbComposite.ps0.tonemap = vec4(_settings.tonemap.exposure, _settings.tonemap.min_white, 0, 0);
    _cbComposite.Set(_ctx, 0);
    ObjectHandle inputs[] = {rtColor, rtLines, rtBlur, rtBlur2, rtTextDistort, rtBlurText };
    fullscreen->Execute(inputs,
        6,
        g_Graphics->GetBackBuffer(),
        g_Graphics->GetBackBufferDesc(),
        g_Graphics->GetDepthStencil(),
        _compositeBundle.objects._ps,
        true,
        true,
        &black);
  }

  VB_INDEX += 1;
  return true;
}

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Intro::RenderParameterSet()
{
  ImGui::Checkbox("plexus", &extended);
  if (extended)
  {
    ImGui::InputInt("# particles", &_settings.num_particles, 25, 100);
    ImGui::SliderFloat("blur", &_settings.blur_radius, 1, 100);
  }
  else
  {
    ImGui::SliderAngle("camera xz-plane", &angle);
    ImGui::SliderFloat("camera distance", &distance, 1, 2000);
    ImGui::SliderFloat("camera height", &height, -100, 100);
  }

  ImGui::SliderFloat("Exposure", &_settings.tonemap.exposure, 0.1f, 20.0f);
  ImGui::SliderFloat("Min White", &_settings.tonemap.min_white, 0.1f, 20.0f);

  if (ImGui::Button("Reset"))
    Reset();
}
#endif

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Intro::SaveParameterSet(bool inc)
{
  _freeflyCamera.ToProtocol(&_settings.camera);
  SaveSettings(_settings, inc);
}
#endif

//------------------------------------------------------------------------------
void Intro::Reset()
{
}

//------------------------------------------------------------------------------
bool Intro::Close()
{
  return true;
}

//------------------------------------------------------------------------------
BaseEffect* Intro::Create(const char* name, const char* config, u32 id)
{
  return new Intro(name, config, id);
}

//------------------------------------------------------------------------------
const char* Intro::Name()
{
  return "intro";
}

//------------------------------------------------------------------------------
void Intro::Register()
{
  g_DemoEngine->RegisterFactory(Name(), Intro::Create);
}
