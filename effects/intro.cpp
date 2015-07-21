#include "intro.hpp"
#include "../tano.hpp"
#include "../graphics.hpp"
#include "../graphics_extra.hpp"
#include "../graphics_context.hpp"
#include "../demo_engine.hpp"
#include "../resource_manager.hpp"
#include "../init_sequence.hpp"
#include "../generated/demo.parse.hpp"
#include "../generated/input_buffer.hpp"
#include "../generated/output_buffer.hpp"
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

extern "C" float stb_perlin_noise3(float x, float y, float z);

namespace
{
  bool wireframe = false;
  float angle = 0;
  float height = 0;
  float distance = 1300;
  bool extended = false;
}

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
int CalcLines(V3* vtx, const V3* points, int num, int* neighbours, const PlexusObject& config)
{
  u8* connected = g_ScratchMemory.Alloc<u8>(num*num);
  memset(connected, 0, num*num);

  struct DistEntry { float dist; int idx; };
  DistEntry* dist = g_ScratchMemory.Alloc<DistEntry>(num);
  int* idx = g_ScratchMemory.Alloc<int>(num);
  u8* degree = g_ScratchMemory.Alloc<u8>(num);
  memset(degree, 0, num);

  V3* orgVtx = vtx;

  float minDist = config.min_dist;
  float maxDist = config.max_dist;
  int numNeighbours = config.num_neighbours;

  float eps = config.eps;

  auto fnSort = [&](int a, int b) {
    float da = dist[a].dist;
    float db = dist[b].dist;
    float d = da - db;
    if (d < 0) d *= -1;
    if (d < eps)
      return degree[a] < degree[b];
    return da < db;
  };

  for (int i = 0; i < num; ++i)
    idx[i] = i;

  for (int i = 0; i < num; ++i)
  {
    int numValid = 0;
    for (int j = 0; j < numNeighbours; ++j)
    {
      int curIdx = neighbours[i*num + j];
      if (curIdx == -1)
        break;

      float d = Distance(points[i], points[curIdx]);

      if (curIdx == i || d < minDist || d > maxDist || connected[i*num + curIdx] || connected[curIdx*num + i])
        continue;

      idx[numValid] = numValid;
      dist[numValid].dist = d;
      dist[numValid].idx = curIdx;
      numValid++;
    }

    sort(idx, idx + numValid, fnSort);

    int left = config.num_nearest;
    for (int j = 0; j < numValid; ++j)
    {
      int curIdx = dist[idx[j]].idx;

      vtx[0] = points[i];
      vtx[1] = points[curIdx];
      vtx += 2;

      connected[i*num + curIdx] = 1;
      degree[i]++;
      degree[curIdx]++;

      if (--left == 0)
        break;
    }
  }

  return (int)(vtx - orgVtx);
}

//------------------------------------------------------------------------------
void DistortVerts(
  V3* dst, const V3* src, int num, 
  const V3* randomPoints, float scale, float strength,
  const V3 ptOfs, float ptScale
  )
{
  for (int i = 0; i < num; ++i)
  {
    V3 pt(ptScale * (src[i] + ptOfs));
    float s = fabs(1024 * stb_perlin_noise3(pt.x / scale, pt.y / scale, pt.z / scale));
    V3 v = randomPoints[IntMod((int)s, 1024)];
    dst[i] = pt + strength * v;
  }
}

//------------------------------------------------------------------------------
static void CalcTextNeighbours(int num, const vector<int>& tris, int* neighbours)
{
  unordered_map<int, vector<int>> tmp;

  int numTris = (int)tris.size() / 3;
  for (int i = 0; i < numTris; ++i)
  {
    int a = tris[i * 3 + 0];
    int b = tris[i * 3 + 1];
    int c = tris[i * 3 + 2];

    tmp[a].push_back(b);
    tmp[a].push_back(c);

    tmp[b].push_back(a);
    tmp[b].push_back(c);

    tmp[c].push_back(a);
    tmp[c].push_back(b);
  }

  for (int i = 0; i < num; ++i)
  {
    const vector<int>& n = tmp[i];
    int cnt = (u32)n.size();
    for (int j = 0; j < cnt; ++j)
    {
      neighbours[i*num + j] = n[j];
    }
    // terminate
    neighbours[i*num + cnt] = -1;
  }

}

//------------------------------------------------------------------------------
void Intro::GenRandomPoints(float kernelSize)
{
  V3* tmp = g_ScratchMemory.Alloc<V3>(_randomPoints.Capacity());
  for (int i = 0; i < _randomPoints.Capacity(); ++i)
  {
    V3 v(randf(-1.f, 1.f), randf(-1.f, 1.f), randf(-1.f, 1.f));
    v = Normalize(v);
    tmp[i] = v;
  }

  _randomPoints.Resize(_randomPoints.Capacity());

  BlurLine(tmp, _randomPoints.Data(), _randomPoints.Capacity(), kernelSize);
}

//------------------------------------------------------------------------------
void Intro::ParticleEmitter::Create(const V3& center, int numParticles)
{
  _center = center;
  _numParticles = numParticles;
  pos = new XMVECTOR[_numParticles];
  vel = new XMVECTOR[_numParticles];
  _deadParticles = new int[_numParticles];

  float s = 100;
  for (int i = 0; i < _numParticles; ++i)
  {
    CreateParticle(i, s);
  }

  for (int i = 0; i < 1000; ++i)
  {
    Update(0.01f);
  }
}

//------------------------------------------------------------------------------
void Intro::ParticleEmitter::CreateParticle(int idx, float s)
{
  // lifetime and lifetime decay is stored in the w-component
  XMFLOAT4 p(
    _center.x + randf(-s, s),
    _center.y + randf(-s, s),
    _center.z + randf(1500.f, 2000.f),
    0.f);

  XMFLOAT4 v(
    randf(-s, s),
    randf(-s, s),
    -randf(10.f, 200.f),
    0);

  pos[idx] = XMLoadFloat4(&p);
  vel[idx] = XMLoadFloat4(&v);
}

//------------------------------------------------------------------------------
void Intro::ParticleEmitter::Destroy()
{
  SAFE_ADELETE(pos);
  SAFE_ADELETE(vel);
  SAFE_ADELETE(_deadParticles);
}

//------------------------------------------------------------------------------
void Intro::ParticleEmitter::Update(float dt)
{
  int numDead = 0;
  int* dead = _deadParticles;

  XMVECTOR zClip = XMVectorReplicate(-1500);

  for (int i = 0, e = _numParticles; i < e; ++i)
  {
    XMVECTOR scaledVel = XMVectorScale(vel[i], dt);
    pos[i] = XMVectorAdd(pos[i], scaledVel);

    XMVECTOR res = XMVectorLess(pos[i], zClip);
    u32 rr[4];
    // rr[2] is 0xffffffff if pos[i] < zClip
    XMStoreInt4(rr, res);
    if (rr[2])
    {
      *dead++ = i;
      numDead++;
    }
  }

  float s = 100;
  for (int i = 0; i < numDead; ++i)
  {
    CreateParticle(_deadParticles[i], s);
  }
}

//------------------------------------------------------------------------------
void Intro::ParticleEmitter::CopyToBuffer(ParticleType* vtx)
{
  memcpy(vtx, pos, _numParticles * sizeof(ParticleType));
}

//------------------------------------------------------------------------------
Intro::Intro(const string &name, const string& config, u32 id)
  : BaseEffect(name, config, id)
{
#if WITH_IMGUI
  PROPERTIES.Register("particle tunnel", 
    bind(&Intro::RenderParameterSet, this),
    bind(&Intro::SaveParameterSet, this));
#endif
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
  return ParseIntroSettings(InputBuffer(buf), &_settings);
}

Vector4 ToVector4(const Color& c)
{
  return Vector4(c.x, c.y, c.z, c.w);
}

//------------------------------------------------------------------------------
bool Intro::Init()
{
  BEGIN_INIT_SEQUENCE();

  // Background state setup
  INIT(_backgroundBundle.Create(BundleOptions()
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/intro.background", "PsBackground")));

  vector<D3D11_INPUT_ELEMENT_DESC> inputs = {
    CD3D11_INPUT_ELEMENT_DESC("POSITION", DXGI_FORMAT_R32G32B32A32_FLOAT)
  };

  INIT(_particleBundle.Create(BundleOptions()
    .VertexShader("shaders/out/intro.particle", "VsParticle")
    .GeometryShader("shaders/out/intro.particle", "GsParticle")
    .PixelShader("shaders/out/intro.particle", "PsParticle")
    .InputElements(inputs)
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_POINTLIST)
    .DynamicVb(24 * _settings.num_particles, sizeof(ParticleType))
    .DepthStencilDesc(depthDescDepthDisabled)
    .BlendDesc(blendDescPreMultipliedAlpha)
    .RasterizerDesc(rasterizeDescCullNone)));

  INIT_RESOURCE(_particleTexture, RESOURCE_MANAGER.LoadTexture(_settings.texture.c_str()));
  
  // Create default emitter
  for (int i = 0; i < 10; ++i)
  {
    _particleEmitters.Append(ParticleEmitter()).Create(V3(0, 0, 0), _settings.num_particles);
  }
  
  // Composite state setup
  INIT(_compositeBundle.Create(BundleOptions()
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/intro.composite", "PsComposite")));

  GenRandomPoints(_settings.plexus.blur_kernel);

  // Text setup
  INIT(_textWriter.Init("gfx/text1.boba"));
  const char* text[] = {
    "neurotica efs",
    "radio silence",
    "solskogen",
  };

  int maxVerts = 0;
  for (int i = 0; i < ELEMS_IN_ARRAY(text); ++i)
  {
    TextData& t = _textData[i];
    _textWriter.GenerateTris(text[i], TextWriter::TextOutline, &t.outline);
    _textWriter.GenerateTris(text[i], TextWriter::TextCap1, &t.cap);
    _textWriter.GenerateIndexedTris(text[i], TextWriter::TextOutline, &t.verts, &t.indices);
    t.transformedVerts.resize(t.verts.size());
    copy(t.verts.begin(), t.verts.end(), t.transformedVerts.begin());

    int num = (u32)_textData[i].verts.size();
    _textData[i].neighbours = new int[num*num];
    memset(_textData[i].neighbours, 0xff, num*num*sizeof(int));
    CalcTextNeighbours(num, _textData[i].indices, _textData[i].neighbours);

    maxVerts = max(maxVerts, (int)_textData[i].outline.size());
  }

  INIT(_plexusLineBundle.Create(BundleOptions()
    .VertexShader("shaders/out/intro.plexus", "VsLines")
    .GeometryShader("shaders/out/intro.plexus", "GsLines")
    .PixelShader("shaders/out/intro.plexus", "PsLines")
    .VertexFlags(VF_POS)
    .RasterizerDesc(rasterizeDescCullNone)
    .BlendDesc(blendDescBlendOneOne)
    .DepthStencilDesc(depthDescDepthDisabled)
    .DynamicVb(128 * 1024, sizeof(V3))
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST)));

  // Generic setup
  INIT(_cbBackground.Create());
  INIT(_cbComposite.Create());
  INIT(_cbFracture.Create());
  INIT(_cbParticle.Create());
  _cbBackground.ps0.inner = ToVector4(_settings.inner_color);
  _cbBackground.ps0.outer = ToVector4(_settings.outer_color);

  INIT(_cbPlexus.Create());

  int w, h;
  GRAPHICS.GetBackBufferSize(&w, &h);
  Vector4 dim((float)w, (float)h, 0, 0);
  _cbPlexus.gs0.dim = dim;
  _cbPlexus.gs0.world = Matrix::Identity();

  MeshLoader meshLoader;
  INIT(meshLoader.Load("gfx/shatter_plane1.boba"));
  CreateScene(meshLoader, sizeof(FracturePiece), &_scene);

  for (scene::Mesh* mesh : _scene.meshes)
  {
    FracturePiece* p = (FracturePiece*)mesh->userData;
    p->dir = PointOnHemisphere(V3(0,0,-1));
    p->rot = RandomVector();
  }

  {
    vector<D3D11_INPUT_ELEMENT_DESC> inputs = {
      CD3D11_INPUT_ELEMENT_DESC("SV_POSITION", DXGI_FORMAT_R32G32B32_FLOAT),
      CD3D11_INPUT_ELEMENT_DESC("NORMAL", DXGI_FORMAT_R32G32B32_FLOAT),
      CD3D11_INPUT_ELEMENT_DESC("TEXCOORD", DXGI_FORMAT_R32G32_FLOAT),
    };

    INIT(_fractureBundle.Create(BundleOptions()
      .RasterizerDesc(rasterizeDescCullNone)
      .VertexShader("shaders/out/intro.fracture", "VsFracture")
      .InputElements(inputs)
      .PixelShader("shaders/out/intro.fracture", "PsFracture")));
  }

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
void Intro::UpdateCameraMatrix(const UpdateState& state)
{
  float x = distance * sin(angle);
  float z = -distance * cos(angle);
  Vector3 pos = Vector3(x, height, z);
  Vector3 target = Vector3(0, 0, 0);
  Vector3 dir = target - pos;
  dir.Normalize();

  int w, h;
  GRAPHICS.GetBackBufferSize(&w, &h);
  float aspect = (float)w / h;
  Matrix view = Matrix::CreateLookAt(pos, Vector3(0, 0, 0), Vector3(0, 1, 0));
  Matrix proj = Matrix::CreatePerspectiveFieldOfView(XMConvertToRadians(45), aspect, 0.1f, 3000.f);
  Matrix viewProj = view * proj;

  _cbParticle.gs0.world = Matrix::Identity();
  _cbParticle.gs0.viewProj = viewProj.Transpose();

  _cbPlexus.gs0.viewProj = viewProj.Transpose();
  _cbPlexus.gs0.cameraPos = pos;

  float zz = -100;
  _fixedCamera._pos.z = zz;
  _fixedCamera._fov = atan(100.f / fabsf(zz));
  view = _fixedCamera._view;
  proj = _fixedCamera._proj;
  viewProj = view * proj;
  _cbFracture.vs0.viewProj = viewProj.Transpose();
}

//------------------------------------------------------------------------------
void Intro::MemCpy(const scheduler::TaskData& data)
{
  MemCpyKernelData* memcpyData = (MemCpyKernelData*)data.kernelData.data;
  memcpy(memcpyData->dst, memcpyData->src, memcpyData->size);
}

//------------------------------------------------------------------------------
void Intro::UpdateEmitter(const TaskData& data)
{
  EmitterKernelData* emitterData = (EmitterKernelData*)data.kernelData.data;
  ParticleEmitter* emitter = emitterData->emitter;
  emitter->Update(emitterData->dt);
}

//------------------------------------------------------------------------------
void Intro::CopyOutEmitter(const scheduler::TaskData& data)
{
  EmitterKernelData* emitterData = (EmitterKernelData*)data.kernelData.data;
  ParticleEmitter* emitter = emitterData->emitter;
  emitter->CopyToBuffer(emitterData->vtx);
}

//------------------------------------------------------------------------------
bool Intro::Update(const UpdateState& state)
{
  static AvgStopWatch stopWatch;
  stopWatch.Start();

  _curTime = state.localTime.TotalMicroseconds() / (float)1e6;

  auto fnCalcFade = [](float ms, const char* s, const char* e, bool invert)
  {
    float ss = BLACKBOARD.GetFloatVar(s);
    float ee = BLACKBOARD.GetFloatVar(e);
    if (Inside(ms, ss, ee))
    {
      float dd = (ms - ss) / (ee - ss);;
      return invert ? 1 - dd : dd;
    }

    return 1.0f;
  };

  rmt_ScopedCPUSample(ParticleTunnel_Update);

  UpdateCameraMatrix(state);

  if (g_KeyUpTrigger.IsTriggered('1'))
  {
    float s = 500;
    V3 center(randf(-s, s), randf(-s, s), 0);
    _particleEmitters.Append(ParticleEmitter()).Create(center, _settings.num_particles);
  }

  float ms = state.localTime.TotalMicroseconds() / (float)1e6;
  
  TextData* t = nullptr;
  float scale;
  V3 ptOfs(0,0,0);
  float ptScale = 1;
  BLACKBOARD.SetNamespace("intro");
  if (Inside(ms, BLACKBOARD.GetFloatVar("text0FadeInStart"), BLACKBOARD.GetFloatVar("text0FadeOutEnd")))
  {
    t = &_textData[0];
    scale = ms - BLACKBOARD.GetFloatVar("text0FadeInStart");
    float delta = BLACKBOARD.GetFloatVar("text0FadeInEnd") - BLACKBOARD.GetFloatVar("text0FadeInStart");
    _lineFade = min(
      fnCalcFade(ms, "text0FadeInStart", "text0FadeInEnd", false), 
      fnCalcFade(ms, "text0FadeOutStart", "text0FadeOutEnd", true));
    scale = Clamp(0.f, 1.f, scale / delta);
    ptOfs = BLACKBOARD.GetVec3Var("text0pos");
    ptScale = BLACKBOARD.GetFloatVar("text0Scale");
  }
  else if (Inside(ms, BLACKBOARD.GetFloatVar("text1FadeInStart"), BLACKBOARD.GetFloatVar("text1FadeOutEnd")))
  {
    t = &_textData[1];
    scale = ms - BLACKBOARD.GetFloatVar("text1FadeInStart");
    float delta = BLACKBOARD.GetFloatVar("text1FadeInEnd") - BLACKBOARD.GetFloatVar("text1FadeInStart");
    _lineFade = min(
      fnCalcFade(ms, "text1FadeInStart", "text1FadeInEnd", false),
      fnCalcFade(ms, "text1FadeOutStart", "text1FadeOutEnd", true));
    scale = Clamp(0.f, 1.f, scale / delta);
    ptOfs = BLACKBOARD.GetVec3Var("text1pos");
    ptScale = BLACKBOARD.GetFloatVar("text1Scale");
  }
  else if (Inside(ms, BLACKBOARD.GetFloatVar("text2FadeInStart"), BLACKBOARD.GetFloatVar("text2FadeOutEnd")))
  {
    t = &_textData[2];
    scale = ms - BLACKBOARD.GetFloatVar("text2FadeInStart");
    float delta = BLACKBOARD.GetFloatVar("text2FadeInEnd") - BLACKBOARD.GetFloatVar("text2FadeInStart");
    _lineFade = min(
      fnCalcFade(ms, "text2FadeInStart", "text2FadeInEnd", false),
      fnCalcFade(ms, "text2FadeOutStart", "text2FadeOutStart", true));
    scale = Clamp(0.f, 1.f, scale / delta);
    ptOfs = BLACKBOARD.GetVec3Var("text2pos");
    ptScale = BLACKBOARD.GetFloatVar("text2Scale");
  }

  _drawText = false;
  _curText = nullptr;
  if (t)
  {
    _drawText = true;
    _curText = t;
    scale = (1-scale) * BLACKBOARD.GetFloatVar("maxStrength");

    DistortVerts(
      t->transformedVerts.data(),
      t->verts.data(),
      (int)t->verts.size(),
      _randomPoints.Data(),
      _settings.plexus.perlin_scale,
      scale,
      ptOfs,
      ptScale);
  }

  BLACKBOARD.ClearNamespace();

  _cbComposite.ps0.time.x = ms;
  _cbComposite.ps0.tonemap = Vector4(1, 1, 0, 0);

  rmt_ScopedCPUSample(Particles_Update);

  ObjectHandle vb = _particleBundle.objects._vb;
  ParticleType* vtx = _ctx->MapWriteDiscard<ParticleType>(_particleBundle.objects._vb);

  SimpleAppendBuffer<TaskId, 32> tasks;

  for (int i = 0; i < _particleEmitters.Size(); ++i)
  {
    EmitterKernelData* data = g_ScratchMemory.Alloc<EmitterKernelData>(1);
    *data = EmitterKernelData{ &_particleEmitters[i], 0, vtx + i * _settings.num_particles };
    KernelData kd;
    kd.data = data;
    kd.size = sizeof(EmitterKernelData);
    tasks.Append(SCHEDULER.AddTask(kd, CopyOutEmitter));
  }

  for (const TaskId& taskId : tasks)
    SCHEDULER.Wait(taskId);

  _ctx->Unmap(vb);

  double avg = stopWatch.Stop();
#if WITH_IMGUI
  TANO.AddPerfCallback([=]() {
    ImGui::Text("Update time: %.3fms", 1000 * avg);
  });
#endif
  return true;
}

//------------------------------------------------------------------------------
bool Intro::FixedUpdate(const FixedUpdateState& state)
{
  float ms = state.localTime.TotalMicroseconds() / (float)1e6;

  rmt_ScopedCPUSample(Particles_Update);
  float dt = state.delta;

  ObjectHandle vb = _particleBundle.objects._vb;

  SimpleAppendBuffer<TaskId, 32> tasks;

  for (int i = 0; i < _particleEmitters.Size(); ++i)
  {
    EmitterKernelData* data = g_ScratchMemory.Alloc<EmitterKernelData>(1);
    *data = EmitterKernelData{ &_particleEmitters[i], dt, nullptr };
    KernelData kd;
    kd.data = data;
    kd.size = sizeof(EmitterKernelData);
    tasks.Append(SCHEDULER.AddTask(kd, UpdateEmitter));
  }

  for (const TaskId& taskId : tasks)
    SCHEDULER.Wait(taskId);

  _fixedCamera.Update(state);

  return true;
}

//------------------------------------------------------------------------------
bool Intro::Render()
{
  static Color black(0, 0, 0, 0);

  FullscreenEffect* fullscreen = GRAPHICS.GetFullscreenEffect();

  ScopedRenderTarget rt(DXGI_FORMAT_R16G16B16A16_FLOAT);
  {
    // Render the background
    _cbBackground.Set(_ctx, 0);
    _ctx->SetRenderTarget(rt._rtHandle, GRAPHICS.GetDepthStencil(), &black);
    _ctx->SetBundle(_backgroundBundle);
    _ctx->Draw(3, 0);
  }

  {
    // Render particles
    _cbParticle.Set(_ctx, 0);
    _ctx->SetBundleWithSamplers(_particleBundle, PixelShader);
    _ctx->SetShaderResource(_particleTexture);
    _ctx->Draw(_settings.num_particles * _particleEmitters.Size(), 0);
  }

  ScopedRenderTarget rtLines(DXGI_FORMAT_R16G16B16A16_FLOAT);
  if (_drawText && _curText)
  {
    // Render lines
    _ctx->SetRenderTarget(rtLines, GRAPHICS.GetDepthStencil(), &black);

    RenderTargetDesc desc = GRAPHICS.GetBackBufferDesc();
    V3 params = BLACKBOARD.GetVec3Var("intro.lineParams");
    _cbPlexus.ps0.lineParams = Vector4(params.x, params.y, params.z, _lineFade);
    _cbPlexus.Set(_ctx, 0);

    ObjectHandle vb = _plexusLineBundle.objects._vb;
    V3* vtx = _ctx->MapWriteDiscard<V3>(vb);
    int numLines = CalcLines(
      vtx,
      _curText->transformedVerts.data(),
      (int)_curText->transformedVerts.size(),
      _curText->neighbours,
      _settings.plexus);
    _ctx->Unmap(vb);
    _ctx->SetBundle(_plexusLineBundle);
    _ctx->Draw(numLines, 0);
    _ctx->UnsetRenderTargets(0, 1);
  }

  ScopedRenderTarget rtBlur(DXGI_FORMAT_R16G16B16A16_FLOAT, BufferFlags(BufferFlag::CreateSrv | BufferFlag::CreateUav));
  {
    // blur
    fullscreen->Blur(rtLines._rtHandle, rtBlur._rtHandle, rtBlur._desc, _settings.blur_radius, 1);
  }

  ScopedRenderTarget rtCompose(DXGI_FORMAT_R16G16B16A16_FLOAT, BufferFlag::CreateSrv);
  {
    //composite
    _cbComposite.ps0.tonemap = Vector4(_settings.tonemap.exposure, _settings.tonemap.min_white, 0, 0);
    _cbComposite.Set(_ctx, 0);
    ObjectHandle inputs[] = { rt, rtLines, rtBlur };
    fullscreen->Execute(
      inputs, 3,
      rtCompose, rtCompose._desc,
      GRAPHICS.GetDepthStencil(),
      _compositeBundle.objects._ps,
      true,
      &black);

    _ctx->SetShaderResource(rtCompose._rtHandle);
    _ctx->SetRenderTarget(GRAPHICS.GetBackBuffer(), GRAPHICS.GetDepthStencil(), &black);
  }

  {
    // fracture
    _ctx->SetBundle(_fractureBundle);

    int w, h;
    GRAPHICS.GetBackBufferSize(&w, &h);
    float aspect = (float)w / h;
    Matrix mtxScale = Matrix::CreateScale(aspect, 1, 1);

    float explodeFactor = max(0.f, _curTime - BLACKBOARD.GetFloatVar("intro.explodeTime"));
    float moveSpeed = BLACKBOARD.GetFloatVar("intro.moveSpeed");
    float rotSpeed = BLACKBOARD.GetFloatVar("intro.rotSpeed");

    _cbFracture.Set(_ctx, 0);

    for (scene::MeshBuffer* buf : _scene.meshBuffers)
    {
      _ctx->SetVertexBuffer(buf->vb);
      _ctx->SetIndexBuffer(buf->ib);

      for (scene::Mesh* mesh : buf->meshes)
      {
        FracturePiece* p = (FracturePiece*)mesh->userData;
        Matrix mtx = mesh->mtxGlobal;

        Matrix mtxDir = Matrix::CreateTranslation(explodeFactor * moveSpeed * ToVector3(p->dir));
        Matrix mtxRotX = Matrix::CreateRotationX(explodeFactor * rotSpeed * p->rot.x);
        Matrix mtxRotY = Matrix::CreateRotationX(explodeFactor * rotSpeed * p->rot.y);
        Matrix mtxRotZ = Matrix::CreateRotationX(explodeFactor * rotSpeed * p->rot.z);

        _cbFracture.vs1.objWorld = (mesh->mtxInvLocal * mtxRotX * mtxRotY * mtxRotZ * mesh->mtxLocal * mtxScale * mtxDir).Transpose();
        //_cbFracture.vs1.objWorld = (mesh->mtxInvLocal * mtxRotX * mtxRotY * mtxRotZ * mtxDir * mesh->mtxLocal ).Transpose();
        //_cbFracture.vs1.objWorld = (mesh->mtxInvLocal * mtxRotX * mtxRotY * mtxRotZ * mesh->mtxLocal).Transpose();
        //_cbFracture.vs1.objWorld = Matrix::Identity();
        _cbFracture.Set(_ctx, 1);
        _ctx->DrawIndexed(mesh->indexCount, mesh->startIndexLocation, mesh->baseVertexLocation);
      }
    }
  }

  return true;
}

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Intro::RenderParameterSet()
{
  ImGui::Checkbox("extended", &extended);
  if (extended)
  {
    //if (ImGui::ColorEdit4("Tint", &_settings.tint.x)) _cbPerFrame.tint = _settings.tint;
    //if (ImGui::ColorEdit4("Inner", &_settings.inner_color.x)) _cbPerFrame.inner = _settings.inner_color;
    //if (ImGui::ColorEdit4("Outer", &_settings.outer_color.x)) _cbPerFrame.outer = _settings.outer_color;
    ImGui::Separator();
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
void Intro::SaveParameterSet()
{
  SaveSettings(_settings);
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
