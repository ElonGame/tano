#include "tunnel.hpp"
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

static float Z_SPACING = 50;
int CAMERA_STEP = 5;

//------------------------------------------------------------------------------
Tunnel::Tunnel(const string& name, const string& config, u32 id) : BaseEffect(name, config, id)
{
#if WITH_IMGUI
  PROPERTIES.Register(Name(), bind(&Tunnel::RenderParameterSet, this), bind(&Tunnel::SaveParameterSet, this));

  PROPERTIES.SetActive(Name());
#endif
}

//------------------------------------------------------------------------------
Tunnel::~Tunnel()
{
}

//------------------------------------------------------------------------------
bool Tunnel::OnConfigChanged(const vector<char>& buf)
{
  return ParseTunnelSettings(InputBuffer(buf), &_settings);
}

//------------------------------------------------------------------------------
bool Tunnel::Init()
{
  BEGIN_INIT_SEQUENCE();

  {
    // create spline
    int numPoints = 10000;
    float s = 20;

    V3 cur(0, 0, 0);
    vector<V3> controlPoints;
    vector<V3> cameraControlPoints;

    for (int i = 0; i < numPoints; ++i)
    {
      float xOfs = randf(-s, +s);
      float yOfs = randf(-s, +s);

      controlPoints.push_back(cur);
      cur += V3(xOfs, yOfs, Z_SPACING);

      if ((i % CAMERA_STEP) == 0)
        cameraControlPoints.push_back(cur);
    }

    _spline.Create(controlPoints.data(), (int)controlPoints.size());
    _cameraSpline.Create(cameraControlPoints.data(), (int)cameraControlPoints.size());
  }

  // clang-format off
  INIT(_linesBundle.Create(BundleOptions()
    .VertexShader("shaders/out/tunnel", "VsTunnelLines")
    .GeometryShader("shaders/out/tunnel", "GsTunnelLines")
    .PixelShader("shaders/out/tunnel", "PsTunnelLines")
    .VertexFlags(VF_POS)
    .RasterizerDesc(rasterizeDescCullNone)
    .BlendDesc(blendDescBlendOneOne)
    .DepthStencilDesc(depthDescDepthDisabled)
    .DynamicVb(128 * 1024, sizeof(V3))
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST)));

  INIT(_compositeBundle.Create(BundleOptions()
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/tunnel.composite", "PsComposite")));
  // clang-format on

  INIT(_cbLines.Create());
  INIT(_cbComposite.Create());

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool Tunnel::Update(const UpdateState& state)
{
  V3 pos(V3(_camera._pos));

  V2 cameraParams = BLACKBOARD.GetVec2Var("tunnel.cameraParams");
  //_camera.SetMaxSpeedAndForce(cameraParams.x, cameraParams.y);

  // create spline segment
  float radius = BLACKBOARD.GetFloatVar("tunnel.radius", state.localTime.TotalSecondsAsFloat());
  // radius *= (1 + 0.2f * sinf(state.localTime.TotalSecondsAsFloat()));
  int numSegments = BLACKBOARD.GetIntVar("tunnel.segments");

  _tunnelVerts.Clear();

  SimpleAppendBuffer<V3, 512> ring1, ring2;

  SimpleAppendBuffer<V3, 512>* rings[] = {&ring1, &ring2};

  float START_OFS = -2;
  int tmp = (int)((_camera._pos.z / Z_SPACING) / Z_SPACING * Z_SPACING);
  float distClamped = (float)tmp;
  // create the first 2 rings
  for (int j = 0; j < 2; ++j)
  {
    V3 pos = _spline.Interpolate(distClamped + (float)j + START_OFS);
    float angle = 0;
    float angleInc = 2 * XM_PI / numSegments;
    V3 prev = pos + radius * V3(cosf(angle), sinf(angle), 0);
    for (int i = 0; i < numSegments; ++i)
    {
      angle += angleInc;
      V3 p = pos + radius * V3(cosf(angle), sinf(angle), 0);
      rings[j]->Append(prev);
      rings[j]->Append(p);
      prev = p;
    }
  }

  for (int j = 2; j < 50; ++j)
  {
    // copy out the first ring, add lines to the second, fill the first, and
    // flip the buffers

    auto& ring0 = *rings[0];
    auto& ring1 = *rings[1];
    for (int i = 0; i < numSegments * 2; ++i)
    {
      _tunnelVerts.Append(ring0[i]);
    }

    for (int i = 0; i < numSegments * 2; i += 2)
    {
      _tunnelVerts.Append(ring0[i]);
      _tunnelVerts.Append(ring1[i]);
    }

    ring0.Clear();
    V3 pos = _spline.Interpolate(distClamped + (float)j + START_OFS);

    float angle = 0;
    float angleInc = 2 * XM_PI / numSegments;
    V3 prev = pos + radius * V3(cosf(angle), sinf(angle), 0);
    for (int i = 0; i < numSegments; ++i)
    {
      angle += angleInc;
      V3 p = pos + radius * V3(cosf(angle), sinf(angle), 0);
      ring0.Append(prev);
      ring0.Append(p);
      prev = p;
    }

    swap(rings[0], rings[1]);
  }

  UpdateCameraMatrix(state);
  return true;
}

//------------------------------------------------------------------------------
bool Tunnel::FixedUpdate(const FixedUpdateState& state)
{
  // Update how far along the spline we've travelled
  float t = state.localTime.TotalSecondsAsFloat();
  float speed = BLACKBOARD.GetFloatVar("tunnel.speed", t);
  float dirScale = BLACKBOARD.GetFloatVar("tunnel.dirScale");
  _dist += state.delta * speed;

  V3 pos = _cameraSpline.Interpolate(_dist / CAMERA_STEP);
  _camera._pos = ToVector3(pos) + Vector3(
    5 * sinf(state.localTime.TotalSecondsAsFloat()),
    5 * cosf(state.localTime.TotalSecondsAsFloat()),
    0);

  _camera._dir = Vector3(
    sinf(state.localTime.TotalSecondsAsFloat()) * dirScale,
    cosf(state.localTime.TotalSecondsAsFloat()) * dirScale,
    1);

  //_camera.SetFollowTarget(ToVector3(pos));
  _camera.Update(state);
  return true;
}

//------------------------------------------------------------------------------
void Tunnel::UpdateCameraMatrix(const UpdateState& state)
{
  Matrix view = _camera._view;
  Matrix proj = _camera._proj;

  Matrix viewProj = view * proj;
  _cbLines.gs0.world = Matrix::Identity();
  _cbLines.gs0.viewProj = viewProj.Transpose();
  _cbLines.gs0.cameraPos = _camera._pos;
}

//------------------------------------------------------------------------------
bool Tunnel::Render()
{
  rmt_ScopedCPUSample(Tunnel_Render);

  static Color black(0, 0, 0, 0);

  _ctx->SetSwapChain(GRAPHICS.DefaultSwapChain(), black);

  ScopedRenderTargetFull rtColor(
      DXGI_FORMAT_R16G16B16A16_FLOAT, BufferFlag::CreateSrv, BufferFlag::CreateSrv);
  _ctx->SetRenderTarget(rtColor._rtHandle, rtColor._dsHandle, &black);

  _cbLines.gs0.dim = Vector4((float)rtColor._desc.width, (float)rtColor._desc.height, 0, 0);
  V3 params = BLACKBOARD.GetVec3Var("tunnel.lineParams");
  _cbLines.ps0.lineParams = Vector4(params.x, params.y, params.z, 1);
  _cbLines.Set(_ctx, 0);

  ObjectHandle h = _linesBundle.objects._vb;
  V3* verts = _ctx->MapWriteDiscard<V3>(h);
  memcpy(verts, _tunnelVerts.Data(), _tunnelVerts.DataSize());
  _ctx->Unmap(h);

  int numVerts = _tunnelVerts.Size();
  _ctx->SetBundle(_linesBundle);
  _ctx->Draw(numVerts, 0);

  FullscreenEffect* fullscreen = GRAPHICS.GetFullscreenEffect();

  {
    _cbComposite.ps0.tonemap = Vector2(_settings.tonemap.exposure, _settings.tonemap.min_white);
    _cbComposite.Set(_ctx, 0);

    ObjectHandle inputs[] = {rtColor, rtColor._dsHandle};
    fullscreen->Execute(inputs,
        2,
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
void Tunnel::RenderParameterSet()
{
  ImGui::SliderFloat("Exposure", &_settings.tonemap.exposure, 0.1f, 20.0f);
  ImGui::SliderFloat("Min White", &_settings.tonemap.min_white, 0.1f, 20.0f);

  if (ImGui::Button("Reset"))
    Reset();
}
#endif

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Tunnel::SaveParameterSet()
{
  SaveSettings(_settings);
}
#endif

//------------------------------------------------------------------------------
void Tunnel::Reset()
{
  _camera._pos = Vector3(0.f, 0.f, 0.f);
  //_camera._pitch = _camera._yaw = _camera._roll = 0.f;
}

//------------------------------------------------------------------------------
bool Tunnel::Close()
{
  return true;
}

//------------------------------------------------------------------------------
BaseEffect* Tunnel::Create(const char* name, const char* config, u32 id)
{
  return new Tunnel(name, config, id);
}

//------------------------------------------------------------------------------
const char* Tunnel::Name()
{
  return "tunnel";
}

//------------------------------------------------------------------------------
void Tunnel::Register()
{
  DEMO_ENGINE.RegisterFactory(Name(), Tunnel::Create);
}
