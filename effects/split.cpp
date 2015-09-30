#include "split.hpp"
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
#include "../mesh_utils.hpp"
#include "../debug_api.hpp"
#include "../mesh_loader.hpp"

using namespace tano;
using namespace bristol;
using namespace DirectX;

static int SEGMENT_SPLITS = 10;
static int ROTATION_SEGMENTS = 10;
static int NUM_INITIAL_SEGMENTS = 15;
static float INITIAL_SPREAD = 25;

#define DEBUG_DRAW_SPLINE 0
#define USE_MESH_SPLINE 0

//------------------------------------------------------------------------------
void AddRing(float curT,
    const vec3& prevT,
    const CardinalSpline& spline,
    float scale,
    vec3* newD,
    vec3* newN,
    vec3* newT,
    vector<PN>* out)
{
  float delta = 1.f / SEGMENT_SPLITS;

  vec3 pos = spline.Interpolate(curT);

  // Propagate frame along spline, using method by Ken Sloan
  vec3 d = Normalize(spline.Interpolate(curT + delta) - pos);
  // note, this uses t from the previous frame
  vec3 n = Cross(d, prevT);
  vec3 t = Cross(n, d);

  *newD = d;
  *newN = n;
  *newT = t;

  float angle = 0;
  float angleInc = XM_2PI / ROTATION_SEGMENTS;
  for (int k = 0; k < ROTATION_SEGMENTS; ++k)
  {
    vec3 pp = pos;
    vec3 dd = Normalize(d);
    Matrix mtx = Matrix::CreateFromAxisAngle(ToVector3(dd), angle);
    vec3 xx = scale * n;
    vec3 rr = Normalize(xx);
    rr = FromVector3(Vector3::Transform(ToVector3(rr), mtx));
    vec3 vv = pp + FromVector3(Vector3::Transform(ToVector3(xx), mtx));
    angle += angleInc;
    out->push_back(PN{vec3(vv), vec3(rr)});

#if DEBUG_DRAW_SPLINE
    DEBUG_API.AddDebugLine(vv, vv + rr, Color(1, 1, 0), Color(1, 0, 1));
#endif
  }
}

//------------------------------------------------------------------------------
void Pathy::Create(const MeshLoader& meshLoader)
{
  SeqDelete(&segments);

  float speedMean = BLACKBOARD.GetFloatVar("split.speedMean");
  float speedVar = BLACKBOARD.GetFloatVar("split.speedVar");

  float time = 0;
  float delta = 1.f / SEGMENT_SPLITS;

#if USE_MESH_SPLINE
  for (protocol::SplineBlob* spline : meshLoader.splines)
  {
    // if (strcmp(spline->name, "MoSpline") != 0)
    //  continue;

    Segment* s = new Segment{vec3(spline->points[0], spline->points[1], spline->points[2]),
        1,
        GaussianRand(speedMean, speedMean),
        0,
        0,
        0};
    segments.push_back(s);
    segmentStart.push_back(SegmentStart{time, s});

    vector<vec3> pts(spline->numPoints);
    memcpy(pts.data(), spline->points, spline->numPoints * sizeof(vec3));
    for (u32 i = 0; i < spline->numPoints; ++i)
    {
      pts[i] = 1.0f / 2 * pts[i];
    }

    s->spline.Create(pts.data(), spline->numPoints, 1.0f / 5);
  }
#else

  for (int i = 0; i < NUM_INITIAL_SEGMENTS; ++i)
  {
    float ss = INITIAL_SPREAD;
    float x = randf(-ss, ss);
    float z = randf(-ss, ss);
    Segment* s = new Segment{vec3(x, 0, z), 1, GaussianRand(speedMean, speedMean), 0, 0, 0};
    segments.push_back(s);
    segmentStart.push_back(SegmentStart{time, s});
  }

  int numVerts = 0;
  while (numVerts < TOTAL_POINTS)
  {
    // update all the active instances
    for (int i = 0; i < (int)segments.size(); ++i)
    {
      Segment& segment = *segments[i];

      float angleX = segment.angleX;
      float angleY = segment.angleY;
      float angleZ = segment.angleZ;

      vec3 cur = segment.cur;
      float scale = segment.scale;
      float curLen = len * scale;
      curLen = len;

      segment.verts.push_back(cur);
      numVerts++;
      if (numVerts >= TOTAL_POINTS)
        break;

      float r = randf(0.f, 1.f);
      if (r >= childProb && (int)segments.size() < maxChildren)
      {
        Segment* s = new Segment{cur,
            scale * childScale,
            scale * GaussianRand(speedMean, speedVar),
            angleX + GaussianRand(angleXMean, angleXVariance),
            angleY + GaussianRand(angleYMean, angleYVariance),
            angleZ + GaussianRand(angleZMean, angleZVariance)};

        segments.push_back(s);
        segmentStart.push_back(SegmentStart{time, s});
      }

      Matrix mtx = Matrix::CreateFromYawPitchRoll(angleX, angleY, angleZ);
      vec3 delta = curLen * FromVector3(Vector3::Transform(Vector3(0, -1, 0), mtx));

      segment.angleX += GaussianRand(angleXMean, angleXVariance);
      segment.angleY += GaussianRand(angleYMean, angleYVariance);
      segment.angleZ += GaussianRand(angleZMean, angleZVariance);
      segment.cur += delta;
    }
    time++;
  }
#endif

  for (int i = 0; i < (int)segments.size(); ++i)
  {
    Segment* segment = segments[i];
#if !USE_MESH_SPLINE
    segment->spline.Create(segment->verts.data(), (int)segment->verts.size(), 1.0f / 3);
#endif

    float tt = segmentStart[i].time;

    // Reference frame
    segment->frameD =
        Normalize(segment->spline.Interpolate(tt + delta) - segment->spline.Interpolate(tt));
    segment->frameT = Perp(segment->frameD);
    segment->frameN = Cross(segment->frameD, segment->frameT);
    segment->frameT = Cross(segment->frameN, segment->frameD);

    AddRing(0,
        segment->frameT,
        segment->spline,
        segment->scale,
        &segment->frameD,
        &segment->frameN,
        &segment->frameT,
        &segment->completeRings);
  }

  // assert(numVerts <= TOTAL_POINTS);
}

//------------------------------------------------------------------------------
void Pathy::CreateTubesIncremental(float orgTime)
{
  // tubeVerts.clear();

  for (SegmentStart start : segmentStart)
  {
    Segment* s = start.segment;
    float time = orgTime * s->speed;
    if (start.time > time)
    {
      break;
    }

    s->started = true;

    float delta = 1.f / SEGMENT_SPLITS;
    float elapsedTime = time - start.time;
    int numTicks = (int)(elapsedTime / delta);

    if (numTicks > s->lastNumTicks)
    {
      s->lastNumTicks = numTicks;

      // add a new full ring
      AddRing(numTicks * delta,
          s->frameT,
          s->spline,
          s->scale,
          &s->frameD,
          &s->frameN,
          &s->frameT,
          &s->completeRings);
    }

#if DEBUG_DRAW_SPLINE
    {
      vec3 p0 = s->spline.Interpolate(elapsedTime);
      DEBUG_API.AddDebugLine(p0, p0 + s->frameD, Color(1, 0, 0));
      DEBUG_API.AddDebugLine(p0, p0 + s->frameN, Color(0, 1, 0));
      DEBUG_API.AddDebugLine(p0, p0 + s->frameT, Color(0, 0, 1));
    }
#endif

    s->inprogressRing.clear();
    float ss = (elapsedTime - (int)(elapsedTime / delta) * delta) / delta;
    ss = 1;
    AddRing(elapsedTime,
        s->frameT,
        s->spline,
        s->scale,
        &s->frameD,
        &s->frameN,
        &s->frameT,
        &s->inprogressRing);
  }
}

//------------------------------------------------------------------------------
Split::Split(const string& name, const string& config, u32 id) : BaseEffect(name, config, id)
{
}

//------------------------------------------------------------------------------
Split::~Split()
{
}

//------------------------------------------------------------------------------
bool Split::OnConfigChanged(const vector<char>& buf)
{
  bool res = ParseSplitSettings(InputBuffer(buf), &_settings);
  _freeflyCamera.FromProtocol(_settings.camera);
  return res;
}

//------------------------------------------------------------------------------
bool Split::Init()
{
  BEGIN_INIT_SEQUENCE();

  // clang-format off
  INIT_FATAL(_backgroundBundle.Create(BundleOptions()
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/trail.background", "PsBackground")));

  INIT_FATAL(_compositeBundle.Create(BundleOptions()
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/trail.composite", "PsComposite")));

  INIT_FATAL(_skyBundle.Create(BundleOptions()
    .DepthStencilDesc(depthDescDepthDisabled)
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/split.sky", "PsSky")));

  _meshFrontFace = GRAPHICS.CreateRasterizerState(rasterizeDescCullFrontFace);
  _meshBackFace =  GRAPHICS.CreateRasterizerState(CD3D11_RASTERIZER_DESC(CD3D11_DEFAULT()));

  INIT_FATAL(_meshBundle.Create(BundleOptions()
    //.RasterizerDesc(rasterizeDescWireframe)
     .BlendDesc(blendDescBlendSrcAlpha)
     //.DepthStencilDesc(depthDescDepthDisabled)
     .InputElement(CD3D11_INPUT_ELEMENT_DESC("POSITION", DXGI_FORMAT_R32G32B32_FLOAT))
     .InputElement(CD3D11_INPUT_ELEMENT_DESC("NORMAL", DXGI_FORMAT_R32G32B32_FLOAT))
    .DynamicVb(10 * 1024 * 1024, 2 * sizeof(vec3))
    .StaticIb(CreateCylinderIndices(ROTATION_SEGMENTS, 100000, true))
    .VertexShader("shaders/out/split.mesh", "VsMesh")
    .PixelShader("shaders/out/split.mesh", "PsMesh")));

  INIT(_particleBundle.Create(BundleOptions()
    .DynamicVb(1024 * 1024 * 6, sizeof(vec3))
    .VertexShader("shaders/out/split.particle", "VsParticle")
    .GeometryShader("shaders/out/split.particle", "GsParticle")
    .PixelShader("shaders/out/split.particle", "PsParticle")
    .InputElement(CD3D11_INPUT_ELEMENT_DESC("POSITION", DXGI_FORMAT_R32G32B32_FLOAT))
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_POINTLIST)
    .DepthStencilDesc(depthDescDepthWriteDisabled)
    .BlendDesc(blendDescBlendSrcAlpha)
    .RasterizerDesc(rasterizeDescCullNone)));
  // clang-format on

  INIT_FATAL(_cbComposite.Create());
  INIT_FATAL(_cbBackground.Create());
  INIT_FATAL(_cbMesh.Create());
  INIT_FATAL(_cbSky.Create());
  INIT_FATAL(_cbParticle.Create());

#if USE_MESH_SPLINE
  INIT_FATAL(_meshLoader.Load("gfx/krumlins3.boba"));
#endif
  _pathy.Create(_meshLoader);

  INIT_RESOURCE_FATAL(_particleTexture, RESOURCE_MANAGER.LoadTexture("gfx/particle_white.png"));

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool Split::Update(const UpdateState& state)
{
  UpdateCameraMatrix(state);

  _pathy.CreateTubesIncremental(state.localTime.TotalSecondsAsFloat());

  _cbBackground.ps0.upper = BLACKBOARD.GetVec4Var("particle_trail.upper");
  _cbBackground.ps0.lower = BLACKBOARD.GetVec4Var("particle_trail.lower");
  return true;
}

//------------------------------------------------------------------------------
bool Split::FixedUpdate(const FixedUpdateState& state)
{
  return true;
}

//------------------------------------------------------------------------------
void Split::UpdateCameraMatrix(const UpdateState& state)
{
  Matrix view = _freeflyCamera._view;
  Matrix proj = _freeflyCamera._proj;

  Matrix viewProj = view * proj;

  _cbMesh.vs0.viewProj = viewProj.Transpose();
  _cbMesh.ps0.cameraPos = _freeflyCamera._pos;
  _cbMesh.vs1.objWorld = Matrix::Identity();

  RenderTargetDesc desc = GRAPHICS.GetBackBufferDesc();
  vec4 dim((float)desc.width, (float)desc.height, 0, 0);
  _cbSky.ps0.dim = dim;
  _cbSky.ps0.cameraPos = _freeflyCamera._pos;
  _cbSky.ps0.cameraLookAt = _freeflyCamera._pos + _freeflyCamera._dir;

  _cbParticle.gs0.viewProj = viewProj.Transpose();
  _cbParticle.gs0.cameraPos = _freeflyCamera._pos;

#if DEBUG_DRAW_SPLINE
  DEBUG_API.SetTransform(Matrix::Identity(), viewProj);
#endif
}

//------------------------------------------------------------------------------
template <typename T>
size_t CopyOut(T* dst, const vector<T>& src)
{
  size_t len = src.size() * sizeof(T);
  memcpy(dst, src.data(), len);
  return src.size();
}

//------------------------------------------------------------------------------
template <typename T>
size_t CopyOutN(T* dst, const vector<T>& src, size_t ofs, size_t n)
{
  n = min(n, ofs > src.size() ? 0 : src.size() - ofs);
  size_t len = n * sizeof(T);
  memcpy(dst, src.data() + ofs, len);
  return n;
}

//------------------------------------------------------------------------------
struct SegmentData
{
  vec3 center;
  const Pathy::Segment* s;
  int segmentIdx;
};

//------------------------------------------------------------------------------
vec3 RingCenter(const PN* verts)
{
  vec3 center(verts[0].pos);

  for (int i = 1; i < ROTATION_SEGMENTS; ++i)
  {
    center += verts[i].pos;
  }

  return center /= (float)ROTATION_SEGMENTS;
}

//------------------------------------------------------------------------------
void Split::RenderSplit()
{
  rmt_ScopedCPUSample(Split_Render);

  static Color black(0, 0, 0, 0);
  FullscreenEffect* fullscreen = GRAPHICS.GetFullscreenEffect();

  ScopedRenderTarget rtColor(DXGI_FORMAT_R16G16B16A16_FLOAT);

  //_ctx->SetRenderTarget(rtColor, &black);
  _ctx->SetRenderTarget(rtColor, GRAPHICS.GetDepthStencil(), &black);
  {
    // sky
    _cbSky.Set(_ctx, 0);
    _ctx->SetBundle(_skyBundle);
    _ctx->Draw(3, 0);
  }

  {
    // tubes
    vec3 camPos = _freeflyCamera._pos;
    // vector<Pathy::Segment*> sortedSegments = _pathy.segments;
    //// sort back to front
    // sort(sortedSegments.begin(),
    //    sortedSegments.end(),
    //    [&](Pathy::Segment* lhs, Pathy::Segment* rhs)
    //    {
    //      return DistanceSquared(vec3(camPos), vec3(lhs->cur))
    //             > DistanceSquared(vec3(camPos), vec3(rhs->cur));
    //    });

    ObjectHandle handle = _meshBundle.objects._vb;

    vector<SegmentData> ss;
    // calc the center points of all the rings
    for (const Pathy::Segment* s : _pathy.segments)
    {
      if (s->started)
      {
        int numSegments = (int)s->completeRings.size() / ROTATION_SEGMENTS;
        for (int i = 0; i < numSegments - 1; ++i)
        {
          ss.push_back(SegmentData{ RingCenter(&s->completeRings[i * ROTATION_SEGMENTS]), s, i });
        }
        ss.push_back(SegmentData{ RingCenter(s->inprogressRing.data()), s, -1 });
      }
    }

    // sort by distance to camera
    sort(ss.begin(),
      ss.end(),
      [&](const SegmentData& lhs, const SegmentData& rhs)
    {
      return DistanceSquared(vec3(camPos), vec3(lhs.center))
        > DistanceSquared(vec3(camPos), vec3(rhs.center));
    });

    PN* vtx = _ctx->MapWriteDiscard<PN>(handle);
    // copy out the sorted verts
    int numVerts = 0;
    int numIndices = 0;
    for (const SegmentData& d : ss)
    {
      const Pathy::Segment* s = d.s;
      int numSegments = (int)s->completeRings.size() / ROTATION_SEGMENTS;
      int idx = d.segmentIdx;

      if (idx == -1)
      {
        numVerts += 2 * ROTATION_SEGMENTS;
        numIndices += ROTATION_SEGMENTS * 6;

        vtx += CopyOutN(vtx, s->completeRings, (numSegments - 1) * ROTATION_SEGMENTS, ROTATION_SEGMENTS);
        vtx += CopyOut(vtx, s->inprogressRing);
      }
      else
      {
        if (numSegments > 1)
        {
          numVerts += 2 * ROTATION_SEGMENTS;
          numIndices += ROTATION_SEGMENTS * 6;

          vtx += CopyOutN(vtx, s->completeRings, idx * ROTATION_SEGMENTS, 2 * ROTATION_SEGMENTS);
        }
      }
    }

#if 0
    for (const Pathy::Segment* s : sortedSegments)
    {
      if (s->started)
      {
        // create stand alone segments for each ring
        int numSegments = (int)s->completeRings.size() / ROTATION_SEGMENTS;
        for (int i = 0; i < numSegments - 1; ++i)
        {
          vtx += CopyOutN(vtx, s->completeRings, i * ROTATION_SEGMENTS, 2 * ROTATION_SEGMENTS);
        }

        vtx += CopyOutN(vtx, s->completeRings, (numSegments - 1) * ROTATION_SEGMENTS, ROTATION_SEGMENTS);
        vtx += CopyOut(vtx, s->inprogressRing);
      }
    }
#endif
    _ctx->Unmap(handle);

    {
      // back facing
      _cbMesh.Set(_ctx, 0);
      _cbMesh.Set(_ctx, 1);
      _ctx->SetBundle(_meshBundle);
      int startVtx = 0;
      _ctx->SetRasterizerState(_meshFrontFace);
      _ctx->DrawIndexed(numIndices, 0, 0);
#if 0
      for (const Pathy::Segment* s : sortedSegments)
      {
        if (s->started)
        {
          // calc # faces at the current segment
          int numVerts = 2 * (int)(s->completeRings.size() + s->inprogressRing.size());
          int n = ((numVerts / SEGMENT_SPLITS) - 1) * 6 * ROTATION_SEGMENTS;
          _ctx->DrawIndexed(n, 0, startVtx);
          startVtx += numVerts;
        }
      }
#endif
    }

#if 0
    {
      // particles
      int numParticles = 0;
      ObjectHandle handle = _particleBundle.objects._vb;
      vec3* vtx = _ctx->MapWriteDiscard<vec3>(handle);
      for (const Pathy::Segment* s : _pathy.segments)
      {
        if (s->started)
        {
          float t = 0;
          for (int i = 0; i < 100; ++i)
          {
            *vtx++ = s->spline.Interpolate(t);
            t += 1;
          }
          numParticles += 100;
        }
      }
      _ctx->Unmap(handle);

      _cbParticle.Set(_ctx, 0);
      _ctx->SetBundleWithSamplers(_particleBundle, ShaderType::PixelShader);

      ObjectHandle srv[] = { _particleTexture };
      _ctx->SetShaderResources(srv, 1, ShaderType::PixelShader);
      _ctx->Draw(numParticles, 0);
      _ctx->UnsetShaderResources(0, 1, ShaderType::PixelShader);
    }
#endif
    {
      // front facing
      _cbMesh.Set(_ctx, 0);
      _cbMesh.Set(_ctx, 1);
      _ctx->SetBundle(_meshBundle);
      int startVtx = 0;
      _ctx->SetRasterizerState(_meshBackFace);
      _ctx->DrawIndexed(numIndices, 0, 0);
      // for (const Pathy::Segment* s : sortedSegments)
      //{
      //  if (s->started)
      //  {
      //    // calc # faces at the current segment
      //    int numVerts = 2 * (int)(s->completeRings.size() + s->inprogressRing.size());
      //    int n = ((numVerts / SEGMENT_SPLITS) - 1) * 6 * ROTATION_SEGMENTS;
      //    _ctx->DrawIndexed(n, 0, startVtx);
      //    startVtx += numVerts;
      //  }
      //}
    }
  }

  {
    // composite
    _cbComposite.ps0.tonemap = vec2(_settings.tonemap.exposure, _settings.tonemap.min_white);
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
}

//------------------------------------------------------------------------------
void Split::RenderTransparent()
{
}

//------------------------------------------------------------------------------
bool Split::Render()
{
  RenderSplit();

  return true;
}

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Split::RenderParameterSet()
{
  bool recalc = false;
  recalc |= ImGui::SliderFloat("len", &_pathy.len, 1.f, 25.f);
  recalc |= ImGui::SliderFloat("child-prob", &_pathy.childProb, 0.f, 1.f);
  recalc |= ImGui::SliderFloat("child-scale", &_pathy.childScale, 0.f, 1.f);
  recalc |= ImGui::SliderFloat("x-mean", &_pathy.angleXMean, 0, XM_PI / 4);
  recalc |= ImGui::SliderFloat("x-var", &_pathy.angleXVariance, 0, 1);
  recalc |= ImGui::SliderFloat("y-mean", &_pathy.angleYMean, 0, XM_PI / 4);
  recalc |= ImGui::SliderFloat("y-var", &_pathy.angleYVariance, 0, 1);
  recalc |= ImGui::SliderFloat("z-mean", &_pathy.angleZMean, 0, XM_PI / 4);
  recalc |= ImGui::SliderFloat("z-var", &_pathy.angleZVariance, 0, 1);

  if (recalc)
    _pathy.Create(_meshLoader);

  ImGui::Separator();
  ImGui::SliderFloat("Exposure", &_settings.tonemap.exposure, 0.1f, 2.0f);
  ImGui::SliderFloat("Min White", &_settings.tonemap.min_white, 0.1f, 2.0f);

  if (ImGui::Button("Reset"))
    Reset();
}
#endif

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Split::SaveParameterSet(bool inc)
{
  _freeflyCamera.ToProtocol(&_settings.camera);
  SaveSettings(_settings, inc);
}
#endif

//------------------------------------------------------------------------------
void Split::Reset()
{
  _freeflyCamera._pos = vec3(0.f, 0.f, 0.f);
  _freeflyCamera._pitch = _freeflyCamera._yaw = _freeflyCamera._roll = 0.f;
}

//------------------------------------------------------------------------------
bool Split::Close()
{
  return true;
}

//------------------------------------------------------------------------------
BaseEffect* Split::Create(const char* name, const char* config, u32 id)
{
  return new Split(name, config, id);
}

//------------------------------------------------------------------------------
const char* Split::Name()
{
  return "split";
}

//------------------------------------------------------------------------------
void Split::Register()
{
  DEMO_ENGINE.RegisterFactory(Name(), Split::Create);
}
