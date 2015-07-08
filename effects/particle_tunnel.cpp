#include "particle_tunnel.hpp"
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
  static AvgStopWatch stopWatch;
  stopWatch.Start();

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

  double avg = stopWatch.Stop();
  TANO.AddPerfCallback([=]() {
    ImGui::Text("Update time: %.3fms", 1000 * avg);
  });

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
void ParticleTunnel::GenRandomPoints(float kernelSize)
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
void ParticleTunnel::ParticleEmitter::Create(const V3& center, int numParticles)
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
void ParticleTunnel::ParticleEmitter::CreateParticle(int idx, float s)
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
void ParticleTunnel::ParticleEmitter::Destroy()
{
  SAFE_ADELETE(pos);
  SAFE_ADELETE(vel);
  SAFE_ADELETE(_deadParticles);
}

//------------------------------------------------------------------------------
void ParticleTunnel::ParticleEmitter::Update(float dt)
{
  //Lifetime* ll = _lifetime;

  int numDead = 0;
  int* dead = _deadParticles;

  XMVECTOR zClip = XMVectorReplicate(-1500);

  for (int i = 0, e = _numParticles; i < e; ++i)
  {
    XMVECTOR scaledVel = XMVectorScale(vel[i], dt);
    pos[i] = XMVectorAdd(pos[i], scaledVel);
    //--ll->left;

    XMVECTOR res = XMVectorLess(pos[i], zClip);
    u32 rr[4];
    // rr[2] is 0xffffffff if pos[i] < zClip
    XMStoreInt4(rr, res);
    if (rr[2])
    {
      *dead++ = i;
      numDead++;
    }

    //ll++;
  }

  float s = 100;
  for (int i = 0; i < numDead; ++i)
  {
    CreateParticle(_deadParticles[i], s);
  }
}

//------------------------------------------------------------------------------
void ParticleTunnel::ParticleEmitter::CopyToBuffer(ParticleType* vtx)
{
  memcpy(vtx, pos, _numParticles * sizeof(ParticleType));
}

//------------------------------------------------------------------------------
ParticleTunnel::ParticleTunnel(const string &name, u32 id)
  : BaseEffect(name, id)
  , _beatTrack("beat")
{
#if WITH_IMGUI
  PROPERTIES.Register("particle tunnel", 
    bind(&ParticleTunnel::RenderParameterSet, this),
    bind(&ParticleTunnel::SaveParameterSet, this));
#endif
}

//------------------------------------------------------------------------------
ParticleTunnel::~ParticleTunnel()
{
  for (int i = 0; i < _particleEmitters.Size(); ++i)
  {
    _particleEmitters[i].Destroy();
  }
}

//------------------------------------------------------------------------------
bool ParticleTunnel::Init(const char* configFile)
{
  BEGIN_INIT_SEQUENCE();

  _configName = configFile;
  AddFileWatchResult res = RESOURCE_MANAGER.AddFileWatch(configFile, true, [this](const string& filename, void*)
  {
    vector<char> buf;
    if (!RESOURCE_MANAGER.LoadFile(filename.c_str(), &buf))
      return false;

    return ParseParticleTunnelSettings(InputBuffer(buf), &_settings);
  });

  // Background state setup
  INIT(_backgroundBundle.Create(BundleOptions()
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/particle_tunnel", "PsBackground")));

  // Particle state setup
  //vector<D3D11_INPUT_ELEMENT_DESC> inputs = {
  //  CD3D11_INPUT_ELEMENT_DESC("POSITION", DXGI_FORMAT_R32G32B32A32_FLOAT),
  //  CD3D11_INPUT_ELEMENT_DESC("TEXCOORD", DXGI_FORMAT_R32G32B32A32_FLOAT) };

  vector<D3D11_INPUT_ELEMENT_DESC> inputs = {
    CD3D11_INPUT_ELEMENT_DESC("POSITION", DXGI_FORMAT_R32G32B32A32_FLOAT)
  };

  INIT(_particleBundle.Create(BundleOptions()
    .VertexShader("shaders/out/particle_tunnel", "VsParticle")
    .GeometryShader("shaders/out/particle_tunnel", "GsParticle")
    .PixelShader("shaders/out/particle_tunnel", "PsParticle")
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
    .PixelShader("shaders/out/particle_tunnel", "PsComposite")));

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
    .VertexShader("shaders/out/plexus", "VsLines")
    .GeometryShader("shaders/out/plexus", "GsLines")
    .PixelShader("shaders/out/plexus", "PsLines")
    .VertexFlags(VF_POS)
    .RasterizerDesc(rasterizeDescCullNone)
    .BlendDesc(blendDescBlendOneOne)
    .DepthStencilDesc(depthDescDepthDisabled)
    .DynamicVb(128 * 1024, sizeof(V3))
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST)));

  INIT_RESOURCE(_lineTexture, RESOURCE_MANAGER.LoadTexture("gfx/line.png"));

  INIT(_lineBundle.Create(BundleOptions()
    .RasterizerDesc(rasterizeDescCullNone)
    .DepthStencilDesc(depthDescDepthDisabled)
    .BlendDesc(blendDescPreMultipliedAlpha)
    .DynamicVb((u32)maxVerts * 3 * 2, sizeof(Vector3))
    .VertexShader("shaders/out/particle_tunnel", "VsLines")
    .VertexFlags(VertexFlags::VF_POS)
    .GeometryShader("shaders/out/particle_tunnel", "GsLines")
    .PixelShader("shaders/out/particle_tunnel", "PsLines")
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST)));

  // Generic setup
  INIT(_cbPerFrame.Create());
  _cbPerFrame.tint = _settings.tint;
  _cbPerFrame.inner = _settings.inner_color;
  _cbPerFrame.outer = _settings.outer_color;

  int w, h;
  GRAPHICS.GetBackBufferSize(&w, &h);
  _cbPerFrame.dim.x = (float)w;
  _cbPerFrame.dim.y = (float)h;
  INIT(_cbBasic.Create());
  INIT(_cbFracture.Create());
  INIT(_cbPerObject.Create());

  MeshLoader meshLoader;
  INIT(meshLoader.Load("gfx/shatter_plane1.boba"));
  CreateScene(meshLoader, false, sizeof(FracturePiece), &_scene);

  for (scene::Mesh* mesh : _scene.meshes)
  {
    FracturePiece* p = (FracturePiece*)mesh->userData;
    p->dir = PointOnHemisphere(V3(0,0,-1));
    p->rot = RandomVector();
  }

  INIT(_fractureBundle.Create(BundleOptions()
    .RasterizerDesc(rasterizeDescCullNone)
    .VertexShader("shaders/out/particle_tunnel", "VsFracture")
    .VertexFlags(VertexFlags::VF_POS | VertexFlags::VF_NORMAL | VertexFlags::VF_TEX2_0)
    .PixelShader("shaders/out/particle_tunnel", "PsFracture")));

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
void ParticleTunnel::UpdateCameraMatrix(const UpdateState& state)
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

  _cbPerFrame.world = Matrix::Identity();
  _cbPerFrame.view = view.Transpose();
  _cbPerFrame.proj = proj.Transpose();
  _cbPerFrame.viewProj = viewProj.Transpose();
  _cbPerFrame.viewDir = Vector4(dir.x, dir.y, dir.z, 1);
  _cbPerFrame.camPos = Vector4(pos.x, pos.y, pos.z, 1);

  Matrix mtx = Matrix::Identity();
  _cbBasic.world = mtx.Transpose();
  _cbBasic.view = view.Transpose();
  _cbBasic.proj = proj.Transpose();
  _cbBasic.viewProj = viewProj.Transpose();
  _cbBasic.cameraPos = _cbPerFrame.camPos;

  float zz = -100;
  _fixedCamera._pos.z = zz;
  // tan fov = w / h
  _fixedCamera._fov = atan(100.f / fabsf(zz));
  _fixedCamera._target = Vector3(0,0,0);
  _fixedCamera.Update(state);
  view = _fixedCamera._view;
  proj = _fixedCamera._proj;
  viewProj = view * proj;
  _cbFracture.world = Matrix::CreateScale(aspect, 1, 1);
  _cbFracture.view = view.Transpose();
  _cbFracture.proj = proj.Transpose();
  _cbFracture.viewProj = viewProj.Transpose();
  _cbFracture.cameraPos = Expand(_fixedCamera._pos, 0);
}

//------------------------------------------------------------------------------
void ParticleTunnel::UpdateEmitter(const TaskData& data)
{
  EmitterKernelData* emitterData = (EmitterKernelData*)data.kernelData.data;
  ParticleEmitter* emitter = emitterData->emitter;

  for (int i = 0; i < emitterData->ticks; ++i)
  {
    emitter->Update(emitterData->dt);
  }

  emitter->CopyToBuffer(emitterData->vtx);
}


//------------------------------------------------------------------------------
bool ParticleTunnel::Update(const UpdateState& state)
{
  _curTime = state.localTime.TotalMilliseconds() / 1000.f;

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

  float ms = state.localTime.TotalMilliseconds() / 1000.f;
  
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


  _cbPerFrame.time.x = ms;
  _cbPerFrame.time.w = _beatTrack;


  static AvgStopWatch stopWatch;
  stopWatch.Start();

  rmt_ScopedCPUSample(Particles_Update);
  float dt = 1.f / state.frequency;

  ObjectHandle vb = _particleBundle.objects._vb;
  ParticleType* vtx = _ctx->MapWriteDiscard<ParticleType>(_particleBundle.objects._vb);

  SimpleAppendBuffer<TaskId, 32> tasks;

  for (int i = 0; i < _particleEmitters.Size(); ++i)
  {
    EmitterKernelData* data = g_ScratchMemory.Alloc<EmitterKernelData>(1);
    *data = EmitterKernelData{ &_particleEmitters[i], dt, state.numTicks, vtx + i * _settings.num_particles };
    KernelData kd;
    kd.data = data;
    kd.size = sizeof(EmitterKernelData);
    tasks.Append(SCHEDULER.AddTask(kd, UpdateEmitter));
  }

  for (const TaskId& taskId : tasks)
    SCHEDULER.Wait(taskId);

  _ctx->Unmap(vb);

  double avg = stopWatch.Stop();
  TANO.AddPerfCallback([=]() {
    ImGui::Text("Update time: %.3fms", 1000 * avg);
  });

  return true;
}

//------------------------------------------------------------------------------
bool ParticleTunnel::Render()
{
  static Color black(0, 0, 0, 0);

  ScopedRenderTarget rt(DXGI_FORMAT_R16G16B16A16_FLOAT);

  u32 cbFlags = ShaderType::VertexShader | ShaderType::GeometryShader | ShaderType::PixelShader;
  _ctx->SetConstantBuffer(_cbPerFrame, cbFlags, 0);

  // Render the background
  _ctx->SetRenderTarget(rt._rtHandle, GRAPHICS.GetDepthStencil(), &black);
  _ctx->SetBundle(_backgroundBundle);
  _ctx->Draw(3, 0);

  _cbPerFrame.world = Matrix::Identity();
  _ctx->SetConstantBuffer(_cbPerFrame, cbFlags, 0);

  // Render particles
  _ctx->SetBundleWithSamplers(_particleBundle, PixelShader);
  _ctx->SetShaderResource(_particleTexture);
  _ctx->Draw(_settings.num_particles * _particleEmitters.Size(), 0);

  ScopedRenderTarget rtLines(DXGI_FORMAT_R16G16B16A16_FLOAT);
  _ctx->SetRenderTarget(rtLines._rtHandle, GRAPHICS.GetDepthStencil(), &black);

  if (_drawText && _curText)
  {
    RenderTargetDesc desc = GRAPHICS.GetBackBufferDesc();
    _cbBasic.dim = Vector4((float)desc.width, (float)desc.height, 0, 0);
    V3 params = BLACKBOARD.GetVec3Var("intro.lineParams");
    _cbBasic.params = Vector4(params.x, params.y, params.z, _lineFade);
    _ctx->SetConstantBuffer(_cbBasic, cbFlags, 0);

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
  }

  // lines
  _ctx->SetConstantBuffer(_cbPerFrame, cbFlags, 0);
  _ctx->UnsetRenderTargets(0, 1);

  ScopedRenderTarget rtBlur(DXGI_FORMAT_R16G16B16A16_FLOAT, BufferFlags(BufferFlag::CreateSrv | BufferFlag::CreateUav));

  FullscreenEffect* fullscreen = GRAPHICS.GetFullscreenEffect();
  fullscreen->Blur(rtLines._rtHandle, rtBlur._rtHandle, _settings.blur_radius);

  _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::PixelShader, 0);


  ScopedRenderTarget rtCompose(DXGI_FORMAT_R16G16B16A16_FLOAT, BufferFlag::CreateSrv);
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

  _ctx->SetBundle(_fractureBundle);

  int w, h;
  GRAPHICS.GetBackBufferSize(&w, &h);
  float aspect = (float)w / h;
  Matrix mtxScale = Matrix::CreateScale(aspect, 1, 1);

  float explodeFactor = max(0.f, _curTime - BLACKBOARD.GetFloatVar("intro.explodeTime"));
  float moveSpeed = BLACKBOARD.GetFloatVar("intro.moveSpeed");
  float rotSpeed = BLACKBOARD.GetFloatVar("intro.rotSpeed");

  _ctx->SetConstantBuffer(_cbFracture, ShaderType::VertexShader, 0);

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

      _cbPerObject.world = (mtxRotX * mtxRotY * mtxRotZ * mtx * mtxScale * mtxDir).Transpose();
      _ctx->SetConstantBuffer(_cbPerObject, ShaderType::VertexShader, 2);

      _ctx->DrawIndexed(mesh->indexCount, mesh->startIndexLocation, mesh->baseVertexLocation);
    }
  }

  return true;
}

//------------------------------------------------------------------------------
#if WITH_IMGUI
void ParticleTunnel::RenderParameterSet()
{
  ImGui::Checkbox("extended", &extended);
  if (extended)
  {
    if (ImGui::ColorEdit4("Tint", &_settings.tint.x)) _cbPerFrame.tint = _settings.tint;
    if (ImGui::ColorEdit4("Inner", &_settings.inner_color.x)) _cbPerFrame.inner = _settings.inner_color;
    if (ImGui::ColorEdit4("Outer", &_settings.outer_color.x)) _cbPerFrame.outer = _settings.outer_color;
    ImGui::Separator();
    ImGui::InputInt("# particles", &_settings.num_particles, 25, 100);

    ImGui::SliderFloat("blur", &_settings.blur_radius, 1, 100);

    if (ImGui::SliderFloat("min dist", &_settings.text_min_dist, 1, 2000)) Reset();
    if (ImGui::SliderFloat("max dist", &_settings.text_max_dist, 1, 2000)) Reset();
    if (ImGui::SliderFloat("triangle prob", &_settings.text_triangle_prob, 0.f, 1.f)) Reset();
  }
  else
  {
    ImGui::SliderAngle("camera xz-plane", &angle);
    ImGui::SliderFloat("camera distance", &distance, 1, 2000);
    ImGui::SliderFloat("camera height", &height, -100, 100);
  }

  if (ImGui::Button("Reset"))
    Reset();
}
#endif

//------------------------------------------------------------------------------
#if WITH_IMGUI
void ParticleTunnel::SaveParameterSet()
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
void ParticleTunnel::Reset()
{
}

//------------------------------------------------------------------------------
bool ParticleTunnel::Close()
{
  return true;
}

//------------------------------------------------------------------------------
BaseEffect* ParticleTunnel::Create(const char* name, u32 id)
{
  return new ParticleTunnel(name, id);
}

//------------------------------------------------------------------------------
const char* ParticleTunnel::Name()
{
  return "particle_tunnel";
}

//------------------------------------------------------------------------------
void ParticleTunnel::Register()
{
  DEMO_ENGINE.RegisterFactory(Name(), ParticleTunnel::Create);
}
