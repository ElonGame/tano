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

using namespace tano;
using namespace bristol;

//------------------------------------------------------------------------------
Tunnel::Tunnel(const string &name, const string& config, u32 id)
  : BaseEffect(name, config, id)
{
#if WITH_IMGUI
  PROPERTIES.Register(Name(),
    bind(&Tunnel::RenderParameterSet, this),
    bind(&Tunnel::SaveParameterSet, this));

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
    float zSpacing = 50;
    float s = 20;

    V3 cur(0, 0, 0);
    vector<V3> controlPoints;

    for (int i = 0; i < numPoints; ++i)
    {
      float xOfs = randf(-s, +s);
      float yOfs = randf(-s, +s);

      controlPoints.push_back(cur);
      cur += V3(xOfs, yOfs, zSpacing);
    }

    _spline.Create(controlPoints.data(), (int)controlPoints.size());
  }

  INIT(_tunnelBundle.Create(BundleOptions()
    .VertexShader("shaders/out/plexus", "VsLines")
    .GeometryShader("shaders/out/plexus", "GsLines")
    .PixelShader("shaders/out/plexus", "PsLines")
    .VertexFlags(VF_POS)
    .RasterizerDesc(rasterizeDescCullNone)
    .BlendDesc(blendDescBlendOneOne)
    .DepthStencilDesc(depthDescDepthWriteDisabled)
    .DynamicVb(128 * 1024, sizeof(V3))
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST)));

  INIT(_cbTunnel.Create());

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool Tunnel::Update(const UpdateState& state)
{
  V3 pos(V3(_camera._pos));

  // create spline segment
  float radius = BLACKBOARD.GetFloatVar("tunnel.radius", state.localTime.TotalSecondsAsFloat());
  int numSegments = BLACKBOARD.GetIntVar("tunnel.segments");

  _tunnelVerts.Clear();

  for (int j = 0; j < 50; ++j)
  {
    V3 pos = _spline.Interpolate(_dist + (float)j);

    float angle = 0;
    float angleInc = 2 * XM_PI / numSegments;
    V3 prev = pos + radius * V3(cosf(angle), sinf(angle), 0);
    for (int i = 0; i < numSegments; ++i)
    {
      angle += angleInc;
      V3 p = pos + radius * V3(cosf(angle), sinf(angle), 0);
      _tunnelVerts.Append(prev);
      _tunnelVerts.Append(p);

      prev = p;
    }
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
  _dist += state.delta * speed;

  V3 pos = _spline.Interpolate(_dist);
  _camera._pos = ToVector3(pos);
  _camera.Update(state);
  return true;
}

//------------------------------------------------------------------------------
void Tunnel::UpdateCameraMatrix(const UpdateState& state)
{
  Matrix view = _camera._view;
  Matrix proj = _camera._proj;

  Matrix viewProj = view * proj;
  _cbTunnel.gs0.world = Matrix::Identity();
  _cbTunnel.gs0.viewProj = viewProj.Transpose();
  _cbTunnel.gs0.cameraPos = _camera._pos;
}

//------------------------------------------------------------------------------
bool Tunnel::Render()
{
  rmt_ScopedCPUSample(Tunnel_Render);

  static Color black(0, 0, 0, 0);

  _ctx->SetSwapChain(GRAPHICS.DefaultSwapChain(), black);

  RenderTargetDesc desc = GRAPHICS.GetBackBufferDesc();
  _cbTunnel.gs0.dim = Vector4((float)desc.width, (float)desc.height, 0, 0);
  V3 params = BLACKBOARD.GetVec3Var("tunnel.lineParams");
  _cbTunnel.ps0.lineParams = Vector4(params.x, params.y, params.z, 1);
  _cbTunnel.Set(_ctx, 0);

  ObjectHandle h = _tunnelBundle.objects._vb;
  V3* verts = _ctx->MapWriteDiscard<V3>(h);
  memcpy(verts, _tunnelVerts.Data(), _tunnelVerts.DataSize());
  _ctx->Unmap(h);

  int numVerts = _tunnelVerts.Size();
  _ctx->SetBundle(_tunnelBundle);
  _ctx->Draw(numVerts, 0);

  _ctx->SetBundle(_tunnelBundle);

  return true;
}

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Tunnel::RenderParameterSet()
{
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
