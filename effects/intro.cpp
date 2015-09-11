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
void CreateTriangles(const V3* vtx, int numVerts, vector<u32>* tris)
{
  V3 minBounds = vtx[0];
  V3 maxBounds = vtx[1];
  for (int i = 0; i < numVerts; ++i)
  {
    minBounds = Min(minBounds, vtx[i]);
    maxBounds = Max(maxBounds, vtx[i]);
  }

  // create a number of buckets between min/max
  int numBuckets = 10;
  struct Bucket
  {
    V3 center;
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
void DistortVerts(V3* dst,
    const V3* src,
    int num,
    const V3* randomPoints,
    float scale,
    float strength,
    const V3 ptOfs,
    float ptScale)
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
      neighbours[i * num + j] = n[j];
    }
    // terminate
    neighbours[i * num + cnt] = -1;
  }
}

//------------------------------------------------------------------------------
void Intro::GenRandomPoints(float kernelSize)
{
  V3* tmp = g_ScratchMemory.Alloc<V3>(_randomPoints.Capacity());
  for (int i = 0; i < _randomPoints.Capacity(); ++i)
  {
    float xSpread = 0.5f;
    float ySpread = 0.5f;
    V3 v(randf(-xSpread, xSpread), randf(-ySpread, ySpread), randf(-1.f, 1.f));
    v = Normalize(v);
    tmp[i] = v;
  }

  _randomPoints.Resize(_randomPoints.Capacity());

  BlurLine(tmp, _randomPoints.Data(), _randomPoints.Capacity(), kernelSize);
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
  return ParseIntroSettings(InputBuffer(buf), &_settings);
}

//------------------------------------------------------------------------------
Vector4 ColorToVector4(const Color& c)
{
  return Vector4(c.x, c.y, c.z, c.w);
}

//------------------------------------------------------------------------------
float Noise3(const V3& v, float scale)
{
  return stb_perlin_noise3(v.x / scale, v.y / scale, v.z / scale);
}

//------------------------------------------------------------------------------
void Intro::CreateKeyframes(TextData::Segment* segment)
{
  int numVerts = (int)segment->verts.size();
  int numKeyframes = _settings.num_keyframes;

  segment->keyframes.resize(numVerts * numKeyframes);

  float scale = _settings.keyframe_scale;
  float step = _settings.keyframe_step;
  float angleScale = _settings.angle_scale;

  for (int i = 0; i < numVerts; ++i)
  {
    V3 pos = segment->verts[i];
    float angle = XM_2PI * Noise3(pos, scale);
    float speed = 0;
    for (int j = 0; j < numKeyframes; ++j)
    {
      pos.x += step * speed * cos(angle);
      pos.y += step * speed * sin(angle);

      float v = Noise3(pos, scale);
      speed += 0.5f;
      angle += XM_2PI * v / angleScale;

      segment->keyframes[i * numKeyframes + (numKeyframes - 1 - j)] = pos;
    }
  }
}

//------------------------------------------------------------------------------
bool Intro::Init()
{
  BEGIN_INIT_SEQUENCE();

  _camera.FromProtocol(_settings.camera);

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
    .DynamicVb(MAX_NUM_PARTICLES, sizeof(V4))
    .DepthStencilDesc(depthDescDepthDisabled)
    .BlendDesc(blendDescPreMultipliedAlpha)
    .RasterizerDesc(rasterizeDescCullNone)));

  INIT(_compositeBundle.Create(BundleOptions()
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/intro.composite", "PsComposite")));

  INIT(_textPolyBundle.Create(BundleOptions()
    .VertexShader("shaders/out/intro.textPoly", "VsMesh")
    .PixelShader("shaders/out/intro.textPoly", "PsMesh")
    .InputElement(CD3D11_INPUT_ELEMENT_DESC("POSITION", DXGI_FORMAT_R32G32B32_FLOAT))
    .InputElement(CD3D11_INPUT_ELEMENT_DESC("NORMAL", DXGI_FORMAT_R32G32B32_FLOAT))));

  INIT(_plexusLineBundle.Create(BundleOptions()
    .VertexShader("shaders/out/intro.plexus", "VsLines")
    .GeometryShader("shaders/out/intro.plexus", "GsLines")
    .PixelShader("shaders/out/intro.plexus", "PsLines")
    .VertexFlags(VF_POS)
    .RasterizerDesc(rasterizeDescCullNone)
    .BlendDesc(blendDescBlendOneOne)
    .DepthStencilDesc(depthDescDepthDisabled)
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST)));

  // clang-format on

  INIT_RESOURCE_FATAL(_particleTexture, RESOURCE_MANAGER.LoadTexture(_settings.texture.c_str()));
  INIT_RESOURCE_FATAL(
      _csParticleBlur, GRAPHICS.LoadComputeShaderFromFile("shaders/out/intro.blur", "BoxBlurY"));

  // Create default emitter
  for (int i = 0; i < 20; ++i)
  {
    _particleEmitters.Append(RadialParticleEmitter())
        .Create(V3(0, 0, 0), 25.f * (i + 1), _settings.num_particles);
  }

  GenRandomPoints(_settings.deform.blur_kernel);

  // Text setup
  INIT(_textWriter.Init("gfx/text3.boba"));
  const char* text[] = {
      "neurotica efs", "radio silence", "solskogen",
  };

  int maxVerts = 0;
  for (int i = 0; i < ELEMS_IN_ARRAY(text); ++i)
  {
    TextData& t = _textData[i];

    _textWriter.GenerateIndexedTris(text[i], TextWriter::TextOutline, &t.outline.verts, &t.outline.indices);
    _textWriter.GenerateTris(text[i], TextWriter::TextCap1, &t.cap.verts);

    t.vb = GRAPHICS.CreateBuffer(D3D11_BIND_VERTEX_BUFFER, 128 * 1024, true, nullptr, sizeof(V3));
    t.vbTri = GRAPHICS.CreateBuffer(D3D11_BIND_VERTEX_BUFFER, 128 * 1024, true, nullptr, 2 * sizeof(V3));

    auto fnFinalize = [this, &maxVerts](TextData::Segment& segment, bool calcNeighours) {

      segment.transformedVerts = segment.verts;
      if (calcNeighours)
      {
        int num = (u32)segment.verts.size();
        segment.neighbours = new int[num * num];
        memset(segment.neighbours, 0xff, num * num * sizeof(int));
        CalcTextNeighbours(num, segment.indices, segment.neighbours);
      }
      CreateKeyframes(&segment);

      maxVerts = max(maxVerts, (int)segment.verts.size());

    };

    fnFinalize(t.outline, true);
    fnFinalize(t.cap, false);
  }

  // Generic setup
  INIT(_cbBackground.Create());
  INIT(_cbComposite.Create());
  INIT(_cbParticle.Create());
  INIT(_cbTextPoly.Create());
  _cbBackground.ps0.inner = ColorToVector4(_settings.inner_color);
  _cbBackground.ps0.outer = ColorToVector4(_settings.outer_color);

  INIT(_cbPlexus.Create());

  int w, h;
  GRAPHICS.GetBackBufferSize(&w, &h);
  Vector4 dim((float)w, (float)h, 0, 0);
  _cbPlexus.gs0.dim = dim;
  _cbPlexus.gs0.world = Matrix::Identity();

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
void Intro::UpdateCameraMatrix(const UpdateState& state)
{
  Matrix view = _camera._view;
  Matrix proj = _camera._proj;
  Matrix viewProj = view * proj;

  _cbParticle.gs0.world = Matrix::Identity();
  _cbParticle.gs0.viewProj = viewProj.Transpose();

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

    _cbPlexus.gs0.viewProj = viewProj.Transpose();
    _cbPlexus.gs0.cameraPos = pos;

    _cbTextPoly.vs0.viewProj = viewProj.Transpose();
    _cbTextPoly.vs0.objWorld = Matrix::Identity();
  }
}

//------------------------------------------------------------------------------
void Intro::UpdateText(const UpdateState& state, TextData* textData, const char* prefix)
{
  float ms = state.localTime.TotalMicroseconds() / (float)1e6;

  char fadeInStart[64], fadeInEnd[64];
  char fadeOutStart[64], fadeOutEnd[64];
  char textPos[64], textScale[64];

  sprintf(fadeInStart, "intro.%sFadeInStart", prefix);
  sprintf(fadeInEnd, "intro.%sFadeInEnd", prefix);
  sprintf(fadeOutStart, "intro.%sFadeOutStart", prefix);
  sprintf(fadeOutEnd, "intro.%sFadeOutEnd", prefix);
  sprintf(textPos, "intro.%sPos", prefix);
  sprintf(textScale, "intro.%sScale", prefix);

  if (ms < BLACKBOARD.GetFloatVar(fadeInStart))
  {
    textData->state = TextData::STATE_INACTIVE;
  }
  else if (ms >= BLACKBOARD.GetFloatVar(fadeOutEnd))
  {
    textData->state = TextData::STATE_DONE;
  }
  else
  {
    textData->state = TextData::STATE_ACTIVE;
  }

  if (textData->state != TextData::STATE_ACTIVE)
    return;

  auto fnCalcFade = [](float ms, const char* s, const char* e, bool invert)
  {
    float ss = BLACKBOARD.GetFloatVar(s);
    float ee = BLACKBOARD.GetFloatVar(e);
    if (Inside(ms, ss, ee))
    {
      float dd = (ms - ss) / (ee - ss);
      return invert ? 1 - dd : dd;
    }

    return 1.0f;
  };

  float scale = ms - BLACKBOARD.GetFloatVar(fadeInStart);
  float delta = BLACKBOARD.GetFloatVar(fadeInEnd) - BLACKBOARD.GetFloatVar(fadeInStart);
  if (textData->state == TextData::STATE_ACTIVE)
    textData->fade = fnCalcFade(ms, fadeInStart, fadeInEnd, false);
  else
    textData->fade = 1;
  scale = Clamp(0.f, 1.f, scale / delta);
  V3 ptOfs = BLACKBOARD.GetVec3Var(textPos);
  float ptScale = BLACKBOARD.GetFloatVar(textScale);

#if 0
  scale = (1 - scale) * BLACKBOARD.GetFloatVar("intro.maxStrength");

  DistortVerts(textData->transformedVerts.data(),
    textData->verts.data(),
    (int)textData->verts.size(),
    _randomPoints.Data(),
    _settings.deform.perlin_scale,
    scale,
    ptOfs,
    ptScale);
#else

  int numKeyframes = _settings.num_keyframes;
  int idx0 = max(0, min(numKeyframes - 1, (int)(scale * numKeyframes)));
  int idx1 = max(0, min(numKeyframes - 1, idx0 + 1));
  float frac = scale * numKeyframes - (float)idx0;

  auto fnInterpolate = [=](TextData::Segment& segment) {
    for (int i = 0; i < (int)segment.verts.size(); ++i)
    {
      V3 v0 = segment.keyframes[i * numKeyframes + idx0];
      V3 v1 = segment.keyframes[i * numKeyframes + idx1];
      segment.transformedVerts[i] = ptOfs + ptScale * lerp(v0, v1, frac);
    }
  };

  fnInterpolate(textData->outline);
  fnInterpolate(textData->cap);
#endif
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

  UpdateText(state, &_textData[0], "text0");
  UpdateText(state, &_textData[1], "text1");
  UpdateText(state, &_textData[2], "text2");

  _cbComposite.ps0.time.x = ms;
  _cbComposite.ps0.tonemap = Vector4(1, 1, 0, 0);

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
  V4* vtx = _ctx->MapWriteDiscard<V4>(_particleBundle.objects._vb);

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

  _camera.Update(state);

  float zz = -1000;
  _textCamera._pos.z = zz;
  _textCamera._fov = atan(100.f / fabsf(zz));
  _textCamera.Update(state);

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
    _ctx->SetRenderTarget(rtColor._rtHandle, GRAPHICS.GetDepthStencil(), &black);
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

  ScopedRenderTarget rtBlur2(rtColor._desc, BufferFlag::CreateSrv | BufferFlag::CreateUav);
  {
    // blur
    _ctx->UnsetRenderTargets(0, 1);
    fullscreen->BlurVertCustom(rtColor, rtBlur2, rtBlur2._desc, _csParticleBlur, 20, 1);
  }

  ScopedRenderTarget rtLines(DXGI_FORMAT_R16G16B16A16_FLOAT);
  if (_drawText)
  {
    _ctx->SetRenderTarget(rtLines, GRAPHICS.GetDepthStencil(), &black);
    V3 params = BLACKBOARD.GetVec3Var("intro.lineParams");

    // tris
    _ctx->SetBundle(_textPolyBundle);

    for (int i = 0; i < 3; ++i)
    {
      TextData* cur = &_textData[i];
      if (cur->state == TextData::STATE_INACTIVE)
        continue;

      float f = cur->fade;
      _cbTextPoly.ps0.params = Vector4(f, f, f, f);
      _cbTextPoly.Set(_ctx, 0);

      ObjectHandle vb = cur->vbTri;
      V3* vtx = _ctx->MapWriteDiscard<V3>(vb);
      V3* src = cur->cap.transformedVerts.data();
      int num = (int)cur->cap.transformedVerts.size() / 3;
      for (int j = 0; j < num; ++j)
      {
        V3& v0 = src[j * 3 + 0];
        V3& v1 = src[j * 3 + 1];
        V3& v2 = src[j * 3 + 2];

        V3 n = Cross(v1 - v0, v2 - v0);
        vtx[j * 6 + 0] = v0;
        vtx[j * 6 + 1] = n;
        vtx[j * 6 + 2] = v1;
        vtx[j * 6 + 3] = n;
        vtx[j * 6 + 4] = v2;
        vtx[j * 6 + 5] = n;
      }
      //memcpy(vtx, cur->cap.transformedVerts.data(), cur->cap.transformedVerts.size() * sizeof(V3));
      _ctx->Unmap(vb);
      _ctx->SetVertexBuffer(vb);
      _ctx->Draw((int)cur->cap.transformedVerts.size(), 0);
    }

    // lines
    _ctx->SetBundle(_plexusLineBundle);
    for (int i = 0; i < 3; ++i)
    {
      TextData* cur = &_textData[i];
      if (cur->state == TextData::STATE_INACTIVE)
        continue;

      _cbPlexus.ps0.lineParams = Vector4(params.x, params.y, params.z, cur->fade);
      _cbPlexus.Set(_ctx, 0);

      ObjectHandle vb = cur->vb;
      V3* vtx = _ctx->MapWriteDiscard<V3>(vb);

      int numLines = CalcPlexusGrouping(vtx,
        cur->outline.transformedVerts.data(),
        (int)cur->outline.transformedVerts.size(),
        cur->outline.neighbours,
        (int)cur->outline.transformedVerts.size(),
        _settings.plexus);

      _ctx->Unmap(vb);
      _ctx->SetVertexBuffer(vb);
      _ctx->Draw(numLines, 0);
    }

    _ctx->UnsetRenderTargets(0, 1);
  }

  ScopedRenderTarget rtBlur(
      DXGI_FORMAT_R16G16B16A16_FLOAT, BufferFlags(BufferFlag::CreateSrv | BufferFlag::CreateUav));
  {
    // blur
    fullscreen->Blur(rtLines._rtHandle, rtBlur._rtHandle, rtBlur._desc, _settings.blur_radius, 1);
  }

  ScopedRenderTarget rtCompose(DXGI_FORMAT_R16G16B16A16_FLOAT, BufferFlag::CreateSrv);
  {
    // composite
    _cbComposite.ps0.tonemap = Vector4(_settings.tonemap.exposure, _settings.tonemap.min_white, 0, 0);
    _cbComposite.Set(_ctx, 0);
    ObjectHandle inputs[] = {rtColor, rtLines, rtBlur, rtBlur2};
    fullscreen->Execute(inputs,
        4,
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
    bool recalcKeyframes = false;
    recalcKeyframes |= ImGui::SliderFloat("angle scale", &_settings.angle_scale, 1, 20);
    recalcKeyframes |= ImGui::SliderFloat("kf step", &_settings.keyframe_step, 1, 50);
    recalcKeyframes |= ImGui::SliderFloat("kf scale", &_settings.keyframe_scale, 1, 100);
    recalcKeyframes |= ImGui::SliderInt("kf count", &_settings.num_keyframes, 20, 100);

    if (recalcKeyframes)
    {
      for (int i = 0; i < 3; ++i)
      {
        CreateKeyframes(&_textData[i].cap);
        CreateKeyframes(&_textData[i].outline);
      }
    }

    ImGui::SliderFloat("eps", &_settings.plexus.eps, 0.1f, 25.0f);
    ImGui::SliderFloat("min-dist", &_settings.plexus.min_dist, 0.1f, 25.0f);
    ImGui::SliderFloat("max-dist", &_settings.plexus.max_dist, 10.0, 1500.0f);
    ImGui::SliderInt("num-nearest", &_settings.plexus.num_nearest, 1, 20);
    ImGui::SliderInt("num-neighbours", &_settings.plexus.num_neighbours, 1, 100);

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
void Intro::SaveParameterSet(bool inc)
{
  _camera.ToProtocol(&_settings.camera);
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
