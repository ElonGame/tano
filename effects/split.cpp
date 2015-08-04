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
#include "../generated/input_buffer.hpp"
#include "../generated/output_buffer.hpp"
#include "../fullscreen_effect.hpp"

using namespace tano;
using namespace bristol;

static int MAX_NUM_PARTICLES = 32 * 1024;

//------------------------------------------------------------------------------
void Pathy::Create()
{
  SeqDelete(&segments);

  float time = 0;
  verts.clear();

  for (int i = 0; i < 10; ++i)
  {
    float ss = 50;
    float x = randf(-ss, ss);
    float z = randf(-ss, ss);
    Segment* s = new Segment{ Vector3(x, 200, z), 1, 0, 0, 0 };
    segments.push_back(s);
    segmentStart.push_back(SegmentStart{ time, s });
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

      segment.verts.push_back(V3(cur));
      numVerts++;
      if (numVerts >= TOTAL_POINTS)
        break;

      float r = randf(0.f, 1.f);
      if (r >= childProb && (int)segments.size() < maxChildren)
      {
        Segment* s = new Segment{ cur,
          scale * childScale,
          angleX + GaussianRand(angleXMean, angleXVariance),
          angleY + GaussianRand(angleYMean, angleYVariance),
          angleZ + GaussianRand(angleZMean, angleZVariance) };

        segments.push_back(s);
        segmentStart.push_back(SegmentStart{ time, s });
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
  for (Segment* segment : segments)
  {
    segment->spline.Create(segment->verts.data(), (int)segment->verts.size(), 1);
    int ofs = (int)verts.size();
    lines.push_back(Line{ofs, (int)segment->verts.size()});
    verts.resize(verts.size() + segment->verts.size());
    copy(segment->verts.begin(), segment->verts.end(), verts.begin() + ofs);
  }

  assert(numVerts <= TOTAL_POINTS);

}

//------------------------------------------------------------------------------
V3 Abs(const V3& v)
{
  return V3(fabs(v.x), fabs(v.y), fabs(v.z));
}

//------------------------------------------------------------------------------
// Returns a vector perpendicular to u, using the method by Hughes-Moller
V3 Perp(V3 u)
{
  V3 a = Abs(u);
  V3 v;
  if (a.x <= a.y && a.x <= a.z)
    v = V3(0, -u.z, u.y);
  else if (a.y <= a.x && a.y <= a.z)
    v = V3(-u.z, 0, u.x);
  else
    v = V3(-u.y, u.x, 0);

  return Normalize(v);
}

//------------------------------------------------------------------------------
void Pathy::CreateTubes(float time)
{
  int SEGMENT_SPLITS = 10;
  int ROTATION_SEGMENTS = 10;

  tubeVerts.clear();

  for (SegmentStart start : segmentStart)
  {
    if (start.time > time)
    {
      break;
    }

    float localTime = 0;

    Segment* s = start.segment;

    // each full second is a complete segment
    int fullSegments = (int)(time - start.time);
    float delta = 1.f / SEGMENT_SPLITS;

    V3 d, t, n;
    // Reference frame
    d = Normalize(s->spline.Interpolate(localTime + delta) - s->spline.Interpolate(localTime));
    t = Perp(d);
    n = Cross(d, t);
    t = Cross(n, d);
   
    float endTime = time - start.time;
    while (true)
    {
      localTime = min(localTime, endTime);
      V3 pos = s->spline.Interpolate(localTime);

      // Propagate frame along spline, using method by Ken Sloan
      d = Normalize(s->spline.Interpolate(localTime + delta) - pos);
      // note, this uses t from the previous frame
      n = Cross(d, t);
      t = Cross(n, d);


      float angle = 0;
      float angleInc = XM_2PI / ROTATION_SEGMENTS;
      for (int k = 0; k < ROTATION_SEGMENTS; ++k)
      {
        Matrix mtx = Matrix::CreateFromAxisAngle(ToVector3(d), angle);
        Vector3 vv = ToVector3(pos) + Vector3::Transform(ToVector3(t), mtx);
        tubeVerts.push_back(vv);
        angle += angleInc;
      }
      
      if (localTime == endTime)
        break;

      localTime += delta;
    }
  }
}

//------------------------------------------------------------------------------
V3* Pathy::CopyOut(V3* buf)
{
  assert(verts.size() <= TOTAL_POINTS);

  memcpy(buf, verts.data(), verts.size() * sizeof(V3));
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
  _camera.FromProtocol(_settings.camera);
  return res;
}

//------------------------------------------------------------------------------
bool Split::Init()
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
    .DynamicVb(128 * 1024, sizeof(V3))
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
    .DynamicVb(128 * 1024, sizeof(V3))
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_LINESTRIP)));
  // clang-format on

  INIT_RESOURCE(_particleTexture, RESOURCE_MANAGER.LoadTexture("gfx/particle3.png"));

  INIT(_cbParticle.Create());
  INIT(_cbComposite.Create());
  INIT(_cbPlexus.Create());
  INIT(_cbBackground.Create());

  _pathy.Create();

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool Split::Update(const UpdateState& state)
{
  UpdateCameraMatrix(state);

  _pathy.CreateTubes(state.globalTime.TotalSecondsAsFloat());

  {
    ObjectHandle handle = _lineBundle.objects._vb;
    V3* vtx = _ctx->MapWriteDiscard<V3>(handle);
    _pathy.CopyOut(vtx);
    _ctx->Unmap(handle);
  }

  {
    ObjectHandle handle = _particleBundle.objects._vb;
    void* vtx = _ctx->MapWriteDiscard<void*>(handle);
    memcpy(vtx, _pathy.tubeVerts.data(), (int)_pathy.tubeVerts.size() * sizeof(Vector3));
    _ctx->Unmap(handle);
  }

  _cbBackground.ps0.upper = ToVector4(BLACKBOARD.GetVec4Var("particle_trail.upper"));
  _cbBackground.ps0.lower = ToVector4(BLACKBOARD.GetVec4Var("particle_trail.lower"));
  return true;
}

//------------------------------------------------------------------------------
bool Split::FixedUpdate(const FixedUpdateState& state)
{
  _camera.Update(state);
  return true;
}

//------------------------------------------------------------------------------
void Split::UpdateCameraMatrix(const UpdateState& state)
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
bool Split::Render()
{
  rmt_ScopedCPUSample(Split_Render);

  static Color black(0, 0, 0, 0);
  FullscreenEffect* fullscreen = GRAPHICS.GetFullscreenEffect();

  ScopedRenderTarget rtColor(DXGI_FORMAT_R16G16B16A16_FLOAT);

  _ctx->SetRenderTarget(rtColor, &black);

  {
    // Render the background
    _cbBackground.Set(_ctx, 0);
    _ctx->SetRenderTarget(rtColor, GRAPHICS.GetDepthStencil(), &black);
    _ctx->SetBundle(_backgroundBundle);
    _ctx->Draw(3, 0);
  }

#if 0
  {
    // lines
    RenderTargetDesc desc = GRAPHICS.GetBackBufferDesc();
    _cbPlexus.gs0.dim = Vector4((float)desc.width, (float)desc.height, 0, 0);
    V3 params = BLACKBOARD.GetVec3Var("particle_trail.lineParams");
    _cbPlexus.ps0.lineParams = Vector4(params.x, params.y, params.z, 1);
    _cbPlexus.Set(_ctx, 0);
    _ctx->SetBundle(_lineBundle);
    for (Pathy::Line& line : _pathy.lines)
    {
      _ctx->Draw(line.size, line.startOfs);
    }
  }
#endif
  {
    int n = (int)_pathy.tubeVerts.size();
    _cbParticle.gs0.numParticles.x = (float)n;
    _cbParticle.Set(_ctx, 0);
    _ctx->SetBundleWithSamplers(_particleBundle, ShaderType::PixelShader);
    _ctx->SetShaderResource(_particleTexture);
    _ctx->Draw(n, 0);
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
  _camera.ToProtocol(&_settings.camera);
  SaveSettings(_settings, inc);
}
#endif

//------------------------------------------------------------------------------
void Split::Reset()
{
  _camera._pos = Vector3(0.f, 0.f, 0.f);
  _camera._pitch = _camera._yaw = _camera._roll = 0.f;
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
