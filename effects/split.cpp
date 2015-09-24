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

using namespace tano;
using namespace bristol;

static int SEGMENT_SPLITS = 10;
static int ROTATION_SEGMENTS = 10;
static int NUM_INITIAL_SEGMENTS = 15;
static float INITIAL_SPREAD = 25;

//------------------------------------------------------------------------------
void AddRing(float curT,
    const V3& prevT,
    const CardinalSpline& spline,
    float scale,
    V3* newD,
    V3* newN,
    V3* newT,
    vector<PN>* out)
{
  float delta = 1.f / SEGMENT_SPLITS;

  V3 pos = spline.Interpolate(curT);

  // Propagate frame along spline, using method by Ken Sloan
  V3 d = Normalize(spline.Interpolate(curT + delta) - pos);
  // note, this uses t from the previous frame
  V3 n = Cross(d, prevT);
  V3 t = Cross(n, d);

  *newD = d;
  *newN = n;
  *newT = t;

  float angle = 0;
  float angleInc = XM_2PI / ROTATION_SEGMENTS;
  for (int k = 0; k < ROTATION_SEGMENTS; ++k)
  {
    Vector3 pp = ToVector3(pos);
    Vector3 dd = ToVector3(d);
    dd.Normalize();
    Matrix mtx = Matrix::CreateFromAxisAngle(dd, angle);
    Vector3 xx = scale * ToVector3(n);
    //Vector3 rr = ToVector3(V3(scale * 1, 0, 0));
    Vector3 rr = xx;
    rr.Normalize();
    rr = Vector3::Transform(rr, mtx);
    Vector3 vv = pp + Vector3::Transform(xx, mtx);
    //Vector3 nn = vv - pp;
    //nn.Normalize();
    angle += angleInc;
    out->push_back(PN{V3(vv), V3(rr)});

    DEBUG_API.AddDebugLine(vv, vv+rr, Color(1, 1, 0), Color(1, 0, 1));

  }
}

//------------------------------------------------------------------------------
void Pathy::Create()
{
  SeqDelete(&segments);

  float time = 0;
  verts.clear();
  float delta = 1.f / SEGMENT_SPLITS;

  float speedMean = BLACKBOARD.GetFloatVar("split.speedMean");
  float speedVar = BLACKBOARD.GetFloatVar("split.speedVar");

  for (int i = 0; i < NUM_INITIAL_SEGMENTS; ++i)
  {
    float ss = INITIAL_SPREAD;
    float x = randf(-ss, ss);
    float z = randf(-ss, ss);
    Segment* s = new Segment{Vector3(x, 0, z), 1, GaussianRand(speedMean, speedMean), 0, 0, 0};
    segments.push_back(s);
    segmentStart.push_back(SegmentStart{time, s});
  }

  int numVerts = 0;
  while (numVerts < TOTAL_POINTS)
  {
    // update all the active instances
    for (int i = 0, e = (int)segments.size(); i < e; ++i)
    {
      Segment& segment = *segments[i];

      float angleX = segment.angleX;
      float angleY = segment.angleY;
      float angleZ = segment.angleZ;

      Vector3 cur = segment.cur;
      float scale = segment.scale;
      float curLen = len * scale;
      curLen = len;

      segment.verts.push_back(V3(cur));
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
      Vector3 delta = curLen * Vector3::Transform(Vector3(0, -1, 0), mtx);

      segment.angleX += GaussianRand(angleXMean, angleXVariance);
      segment.angleY += GaussianRand(angleYMean, angleYVariance);
      segment.angleZ += GaussianRand(angleZMean, angleZVariance);
      segment.cur += delta;
    }

    time++;
  }

  // copy over all the vertices
  for (int i = 0; i < (int)segments.size(); ++i)
  {
    Segment* segment = segments[i];
    segment->spline.Create(segment->verts.data(), (int)segment->verts.size(), 1);

    float tt = segmentStart[i].time;
    // Reference frame
    segment->frameD = Normalize(segment->spline.Interpolate(tt + delta) - segment->spline.Interpolate(tt));
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

    int ofs = (int)verts.size();
    lines.push_back(Line{ofs, (int)segment->verts.size()});
    verts.resize(verts.size() + segment->verts.size());
    copy(segment->verts.begin(), segment->verts.end(), verts.begin() + ofs);
  }

  assert(numVerts <= TOTAL_POINTS);
}

//------------------------------------------------------------------------------
void Pathy::CreateTubesIncremental(float orgTime)
{
  tubeVerts.clear();

  for (SegmentStart start : segmentStart)
  {
    Segment* s = start.segment;
    float time = orgTime * s->speed;
    if (start.time > time)
    {
      break;
    }

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

    {
      V3 p0 = s->spline.Interpolate(elapsedTime);
      DEBUG_API.AddDebugLine(ToVector3(p0), ToVector3(p0 + s->frameD), Color(1, 0, 0));
      DEBUG_API.AddDebugLine(ToVector3(p0), ToVector3(p0 + s->frameN), Color(0, 1, 0));
      DEBUG_API.AddDebugLine(ToVector3(p0), ToVector3(p0 + s->frameT), Color(0, 0, 1));
    }

    s->inprogressRing.clear();
    float ss = (elapsedTime - (int)(elapsedTime / delta) * delta) / delta;
    ss = 1;
    AddRing(
        elapsedTime, s->frameT, s->spline, s->scale, &s->frameD, &s->frameN, &s->frameT, &s->inprogressRing);

    Append(s->completeRings, &tubeVerts);
    Append(s->inprogressRing, &tubeVerts);
  }
}

//------------------------------------------------------------------------------
V3* Pathy::CopyOut(V3* buf)
{
  assert(verts.size() <= TOTAL_POINTS);

  memcpy(buf, tubeVerts.data(), tubeVerts.size() * sizeof(PN));
  return buf + verts.size();
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
     .DepthStencilDesc(depthDescDepthDisabled)
     .InputElement(CD3D11_INPUT_ELEMENT_DESC("POSITION", DXGI_FORMAT_R32G32B32_FLOAT))
     .InputElement(CD3D11_INPUT_ELEMENT_DESC("NORMAL", DXGI_FORMAT_R32G32B32_FLOAT))
    .DynamicVb(10 * 1024 * 1024, 2 * sizeof(V3))
    .StaticIb(CreateCylinderIndices(ROTATION_SEGMENTS, 1000))
    .VertexShader("shaders/out/split.mesh", "VsMesh")
    .PixelShader("shaders/out/split.mesh", "PsMesh")));
  // clang-format on

  INIT_FATAL(_cbComposite.Create());
  INIT_FATAL(_cbBackground.Create());
  INIT_FATAL(_cbMesh.Create());
  INIT_FATAL(_cbSky.Create());

  _pathy.Create();

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool Split::Update(const UpdateState& state)
{
  UpdateCameraMatrix(state);

  _pathy.CreateTubesIncremental(state.localTime.TotalSecondsAsFloat());

  {
    ObjectHandle handle = _meshBundle.objects._vb;
    V3* vtx = _ctx->MapWriteDiscard<V3>(handle);
    _pathy.CopyOut(vtx);
    _ctx->Unmap(handle);
  }

  _cbBackground.ps0.upper = ToVector4(BLACKBOARD.GetVec4Var("particle_trail.upper"));
  _cbBackground.ps0.lower = ToVector4(BLACKBOARD.GetVec4Var("particle_trail.lower"));
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
  _cbMesh.vs0.cameraPos = _freeflyCamera._pos;
  _cbMesh.vs1.objWorld = Matrix::Identity();

  RenderTargetDesc desc = GRAPHICS.GetBackBufferDesc();
  Vector4 dim((float)desc.width, (float)desc.height, 0, 0);
  _cbSky.ps0.dim = dim;
  _cbSky.ps0.cameraPos = _freeflyCamera._pos;
  _cbSky.ps0.cameraLookAt = _freeflyCamera._pos + _freeflyCamera._dir;

  DEBUG_API.SetTransform(Matrix::Identity(), viewProj);

}

//------------------------------------------------------------------------------
bool Split::Render()
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

    vector<Pathy::Segment*> sortedSegments = _pathy.segments;
    sort(sortedSegments.begin(), sortedSegments.end(), [](Pathy::Segment* lhs, Pathy::Segment* rhs)
    {
      return lhs->cur.z > rhs->cur.z;
    });

    _cbMesh.Set(_ctx, 0);
    _cbMesh.Set(_ctx, 1);
    _ctx->SetBundle(_meshBundle);
    int startVtx = 0;
    _ctx->SetRasterizerState(_meshFrontFace);
    for (const Pathy::Segment* s : sortedSegments)
    {
      // calc # faces at the current segment
      int numVerts = (int)(s->completeRings.size() + s->inprogressRing.size());
      int n = ((numVerts / SEGMENT_SPLITS) - 1) * 6 * ROTATION_SEGMENTS;
      _ctx->DrawIndexed(n, 0, startVtx);
      startVtx += numVerts;
    }

    startVtx = 0;
    _ctx->SetRasterizerState(_meshBackFace);

    for (const Pathy::Segment* s : sortedSegments)
    {
      // calc # faces at the current segment
      int numVerts = (int)(s->completeRings.size() + s->inprogressRing.size());
      int n = ((numVerts / SEGMENT_SPLITS) - 1) * 6 * ROTATION_SEGMENTS;
      _ctx->DrawIndexed(n, 0, startVtx);
      startVtx += numVerts;
    }

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
    _pathy.Create();

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
  _freeflyCamera._pos = Vector3(0.f, 0.f, 0.f);
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
