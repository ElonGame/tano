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

using namespace tano;
using namespace tano::scheduler;
using namespace bristol;
using namespace DirectX;

namespace
{
  float angle = 0;
  float height = 0;
  float distance = 1300;
  bool extended = true;
}

template <typename T>
void* BlockAllocate(int headerSize, int numBlocks, int dataOffset)
{
  char* mem = g_ScratchMemory.Alloc(numBlocks * (headerSize + dataSize));
  for (int i = 0; i < numBlocks; ++i)
  {
  }
}

//------------------------------------------------------------------------------
void CreateTriangles(const vec3* vtx, int numVerts, vector<u32>* tris)
{
  vec3 minBounds = vtx[0];
  vec3 maxBounds = vtx[1];
  for (int i = 0; i < numVerts; ++i)
  {
    minBounds = Min(minBounds, vtx[i]);
    maxBounds = Max(maxBounds, vtx[i]);
  }

  // create a number of buckets between min/max
  int numBuckets = 10;
  struct Bucket
  {
    vec3 center;
    int cnt;
    int* vtx;
  };

  int vtxSizeInBytes = sizeof(int) * numVerts;
  int bucketSizeInBytes = sizeof(Bucket) + vtxSizeInBytes;
  Bucket* buckets = (Bucket*)g_ScratchMemory.Alloc(numBuckets * bucketSizeInBytes);
  char* ptr = (char*)buckets;
  int* vtxPtr = (int*)(ptr + numBuckets * sizeof(Bucket));
  for (int i = 0; i < numBuckets; ++i)
  {
    Bucket* b = &buckets[i];
    b->center = lerp(minBounds, maxBounds, i / (float)(numBuckets - 1));
    b->cnt = 0;
    b->vtx = vtxPtr;
    vtxPtr += numVerts;
  }

  // assign each vtx to a bucket
  for (int i = 0; i < numVerts; ++i)
  {
    float bucketDist = DistanceSquared(buckets[0].center, vtx[i]);
    int bucketIdx = 0;
    for (int j = 1; j < numBuckets; ++j)
    {
      float cand = DistanceSquared(buckets[j].center, vtx[i]);
      if (cand < bucketDist)
      {
        bucketIdx = j;
        bucketDist = cand;
      }
    }

    Bucket* b = &buckets[bucketIdx];
    b->vtx[b->cnt++] = i;
  }

  struct SortedEdge
  {
    float dist;
    int a, b;
  };

  SortedEdge* sortedEdges = g_ScratchMemory.Alloc<SortedEdge>(numVerts * numVerts);

  for (int k = 0; k < numBuckets; ++k)
  {
    Bucket* b = &buckets[k];
    int* verts = b->vtx;
    int vertexCount = b->cnt;
    int cnt = 0;
    for (int i = 0; i < vertexCount; i += 5)
    {
      for (int j = i + 1; j < vertexCount; j += 5)
      {
        int ii = verts[i];
        int jj = verts[j];
        sortedEdges[cnt++] = SortedEdge{DistanceSquared(vtx[ii], vtx[jj]), ii, jj};
      }
    }

    sort(sortedEdges,
        sortedEdges + cnt,
        [](const SortedEdge& lhs, const SortedEdge& rhs)
        {
          return lhs.dist < rhs.dist;
        });
  }

  int a = 10;
}

static int MAX_NUM_PARTICLES = 128 * 1024;

//------------------------------------------------------------------------------
template <typename T>
void BlurLine(const T* src, T* dst, int size, float r)
{
  // Blur the data
  float scale = 1.f / (2.0f * r + 1.f);
  int m = (int)r;
  float alpha = r - m;

  // set up initial pixel

  T sum = src[0];
  for (int i = 0; i < m; i++)
    sum += src[IntMod(-i, size)] + src[IntMod(i, size)];
  sum += alpha * (src[IntMod(-m, size)] + src[IntMod(m, size)]);

  for (int i = 0; i < size; ++i)
  {
    dst[i] = sum * scale;
    sum += lerp(src[IntMod(i + m + 1, size)], src[IntMod(i + m + 2, size)], alpha);
    sum -= lerp(src[IntMod(i - m, size)], src[IntMod(i - m - 1, size)], alpha);
  }
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
    .DynamicVb(MAX_NUM_PARTICLES, sizeof(vec4))
    .DepthStencilDesc(depthDescDepthDisabled)
    .BlendDesc(blendDescPreMultipliedAlpha)
    .RasterizerDesc(rasterizeDescCullNone)));

  INIT(_compositeBundle.Create(BundleOptions()
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/intro.composite", "PsComposite")));
  // clang-format on

  INIT_RESOURCE_FATAL(_particleTexture, RESOURCE_MANAGER.LoadTexture(_settings.texture.c_str()));
  INIT_RESOURCE_FATAL(
      _csParticleBlur, GRAPHICS.LoadComputeShaderFromFile("shaders/out/intro.blur", "BoxBlurY"));

  // Create default emitter
  for (int i = 0; i < 20; ++i)
  {
    _particleEmitters.Append(RadialParticleEmitter())
        .Create(vec3(0, 0, 0), 25.f * (i + 1), _settings.num_particles);
  }

  // Generic setup
  INIT(_cbBackground.Create());
  INIT(_cbComposite.Create());
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
    GRAPHICS.GetBackBufferSize(&w, &h);
    float aspect = (float)w / h;
    Matrix view = Matrix::CreateLookAt(ToVector3(pos), Vector3(0, 0, 0), Vector3(0, 1, 0));
    Matrix proj = Matrix::CreatePerspectiveFieldOfView(XMConvertToRadians(45), aspect, 0.1f, 3000.f);
    Matrix viewProj = view * proj;

    _cbPlexus.gs0.viewProj = viewProj.Transpose();
    _cbPlexus.gs0.cameraPos = pos;

    _cbTextPoly.vs0.viewProj = viewProj.Transpose();
    _cbTextPoly.vs0.objWorld = Matrix::Identity();
  }
}

//------------------------------------------------------------------------------
bool Intro::Update(const UpdateState& state)
{
  static AvgStopWatch stopWatch;
  stopWatch.Start();

  UpdateParticleEmitters(state.delta.TotalSecondsAsFloat());

  _curTime = state.localTime.TotalMicroseconds() / (float)1e6;

  rmt_ScopedCPUSample(ParticleTunnel_Update);

  UpdateCameraMatrix(state);

  float ms = state.localTime.TotalMicroseconds() / (float)1e6;

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
  rmt_ScopedCPUSample(Particles_Update);

  SimpleAppendBuffer<TaskId, 32> tasks;

  typedef RadialParticleEmitter::EmitterKernelData EmitterKernelData;

  for (int i = 0; i < _particleEmitters.Size(); ++i)
  {
    EmitterKernelData* data = g_ScratchMemory.Alloc<EmitterKernelData>(1);
    *data = EmitterKernelData{&_particleEmitters[i], dt, nullptr};
    KernelData kd;
    kd.data = data;
    kd.size = sizeof(EmitterKernelData);
    tasks.Append(SCHEDULER.AddTask(kd, RadialParticleEmitter::UpdateEmitter));
  }

  for (const TaskId& taskId : tasks)
    SCHEDULER.Wait(taskId);
}

//------------------------------------------------------------------------------
void Intro::CopyOutParticleEmitters()
{
  rmt_ScopedCPUSample(Particles_CopyOut);

  typedef RadialParticleEmitter::EmitterKernelData EmitterKernelData;

  ObjectHandle vb = _particleBundle.objects._vb;
  vec4* vtx = _ctx->MapWriteDiscard<vec4>(_particleBundle.objects._vb);

  SimpleAppendBuffer<TaskId, 32> tasks;

  _numSpawnedParticles = 0;
  for (int i = 0; i < _particleEmitters.Size(); ++i)
  {
    EmitterKernelData* data = g_ScratchMemory.Alloc<EmitterKernelData>(1);
    *data = EmitterKernelData{&_particleEmitters[i], 0, vtx + i * _particleEmitters[i]._spawnedParticles};
    KernelData kd;
    kd.data = data;
    kd.size = sizeof(ParticleEmitter::EmitterKernelData);

    tasks.Append(SCHEDULER.AddTask(kd, RadialParticleEmitter::CopyOutEmitter));

    _numSpawnedParticles += _particleEmitters[i]._spawnedParticles;
  }

  for (const TaskId& taskId : tasks)
    SCHEDULER.Wait(taskId);

  _ctx->Unmap(vb);
}

//------------------------------------------------------------------------------
bool Intro::FixedUpdate(const FixedUpdateState& state)
{
  float ms = state.localTime.TotalMicroseconds() / (float)1e6;

  rmt_ScopedCPUSample(Particles_Update);
  float dt = state.delta;

  float zz = -1000;
  _textCamera._pos.z = zz;
  _textCamera._fov = atan(100.f / fabsf(zz));
  _textCamera.Update(state.delta);

  return true;
}

//------------------------------------------------------------------------------
bool Intro::Render()
{
  static Color black(0, 0, 0, 0);

  FullscreenEffect* fullscreen = GRAPHICS.GetFullscreenEffect();

  ScopedRenderTarget rtColor(DXGI_FORMAT_R16G16B16A16_FLOAT);
  {
    // Render the background
    _cbBackground.Set(_ctx, 0);
    _ctx->SetRenderTarget(rtColor, GRAPHICS.GetDepthStencil(), &black);
    _ctx->SetBundle(_backgroundBundle);
    _ctx->Draw(3, 0);
  }

  {
    // Render particles
    _cbParticle.Set(_ctx, 0);
    _ctx->SetBundleWithSamplers(_particleBundle, PixelShader);
    _ctx->SetShaderResource(_particleTexture);
    _ctx->Draw(_numSpawnedParticles, 0);
  }

  ScopedRenderTarget rtText(DXGI_FORMAT_R16G16B16A16_FLOAT);
  {
    // text
    _ctx->SetRenderTarget(rtText, GRAPHICS.GetDepthStencil(), &black);
    // 656 x 40

    float right = BLACKBOARD.GetFloatVar("intro.textRight");
    float s = BLACKBOARD.GetFloatVar("intro.textSize");
    float y0 = BLACKBOARD.GetFloatVar("intro.newText0Pos");
    float y1 = BLACKBOARD.GetFloatVar("intro.newText1Pos");
    float y2 = BLACKBOARD.GetFloatVar("intro.newText2Pos");

    float ar0 = _introTexture[0].info.Width / (float)_introTexture[0].info.Height;
    float ar1 = _introTexture[1].info.Width / (float)_introTexture[1].info.Height;
    float ar2 = _introTexture[2].info.Width / (float)_introTexture[2].info.Height;

    bool rightAlign = true;
    if (rightAlign)
    {
      fullscreen->RenderTexture(
        _introTexture[0].h, vec2{ right - ar0 * s, y0 - s }, vec2{ right, y0 });

      fullscreen->RenderTexture(
        _introTexture[1].h, vec2{ right - ar1 * s, y1 - s }, vec2{ right, y1 });

      fullscreen->RenderTexture(
        _introTexture[2].h, vec2{ right - ar2 * s, y2 - s }, vec2{ right, y2 });
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

  float beatHi = BLACKBOARD.GetFloatVar("Beat-Hi", _curTime);

  {
    // blur
    _ctx->UnsetRenderTargets(0, 1);
    fullscreen->BlurVertCustom(rtText, rtBlurText, rtBlurText._desc, _csParticleBlur, 20 * beatHi, 1);
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
    ObjectHandle inputs[] = {rtColor, rtLines, rtBlur, rtBlur2, rtText, rtBlurText};
    fullscreen->Execute(inputs,
        6,
        GRAPHICS.GetBackBuffer(),
        GRAPHICS.GetBackBufferDesc(),
        GRAPHICS.GetDepthStencil(),
        _compositeBundle.objects._ps,
        true,
        true,
        &black);
  }

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
  DEMO_ENGINE.RegisterFactory(Name(), Intro::Create);
}
