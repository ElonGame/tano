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
#include "../fullscreen_effect.hpp"
#include "../mesh_utils.hpp"
#include "../mesh_loader.hpp"

using namespace tano;
using namespace bristol;

static float Z_SPACING = 50;
static int CAMERA_STEP = 10;
static int TUNNEL_DEPTH = 100;
// note, this is mirrored in scratch.bb
static int TUNNEL_SEGMENTS = 50;

//------------------------------------------------------------------------------
Tunnel::Tunnel(const string& name, const string& config, u32 id) : BaseEffect(name, config, id)
{
}

//------------------------------------------------------------------------------
Tunnel::~Tunnel()
{
}

//------------------------------------------------------------------------------
bool Tunnel::OnConfigChanged(const vector<char>& buf)
{
  _settings = ParseTunnelSettings(InputBuffer(buf));
  return true;
}

//------------------------------------------------------------------------------
bool Tunnel::Init()
{
  BEGIN_INIT_SEQUENCE();

  {
    // create spline
    int numPoints = 10000;
    float s = 20;

    vec3 cur(0, 0, 0);
    vector<vec3> controlPoints;
    vector<vec3> cameraControlPoints;

    for (int i = 0; i < numPoints; ++i)
    {
      float xOfs = randf(-s, +s);
      float yOfs = randf(-s, +s);

      controlPoints.push_back(cur);
      cur += vec3(xOfs, yOfs, Z_SPACING);

      if ((i % CAMERA_STEP) == 0)
        cameraControlPoints.push_back(cur);
    }

    _spline.Create(controlPoints.data(), (int)controlPoints.size());
    _cameraSpline.Create(cameraControlPoints.data(), (int)cameraControlPoints.size());
  }

  // clang-format off

  vector<u32> indices = CreateCylinderIndices(TUNNEL_SEGMENTS, TUNNEL_DEPTH);
  _numTunnelFaceIndices = (u32)indices.size();

  INIT_FATAL(_tunnelFaceBundle.Create(BundleOptions()
    .VertexShader("shaders/out/tunnel.facecolor", "VsFace")
    .GeometryShader("shaders/out/tunnel.facecolor", "GsFace")
    .PixelShader("shaders/out/tunnel.facecolor", "PsFace")
    .InputElement(CD3D11_INPUT_ELEMENT_DESC("POSITION", DXGI_FORMAT_R32G32B32_FLOAT))
    //.InputElement(CD3D11_INPUT_ELEMENT_DESC("NORMAL", DXGI_FORMAT_R32G32B32_FLOAT))
    .RasterizerDesc(rasterizeDescCullNone)
    .DynamicVb(128 * 1024, sizeof(vec3))
    .StaticIb(indices)));

  INIT_FATAL(_linesBundle.Create(BundleOptions()
    .VertexShader("shaders/out/tunnel.lines", "VsTunnelLines")
    .GeometryShader("shaders/out/tunnel.lines", "GsTunnelLines")
    .PixelShader("shaders/out/tunnel.lines", "PsTunnelLines")
    .VertexFlags(VF_POS)
    .RasterizerDesc(rasterizeDescCullNone)
    .BlendDesc(blendDescBlendOneOne)
    //.DepthStencilDesc(depthDescDepthWriteDisabled)
    .DepthStencilDesc(depthDescDepthDisabled)
    .DynamicVb(128 * 1024, sizeof(vec3))
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST)));

  INIT_FATAL(_compositeBundle.Create(BundleOptions()
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/tunnel.composite", "PsComposite")));

  INIT_FATAL(_meshBundle.Create(BundleOptions()
    .VertexShader("shaders/out/tunnel.mesh", "VsMesh")
    .InputElement(CD3D11_INPUT_ELEMENT_DESC("SV_POSITION", DXGI_FORMAT_R32G32B32_FLOAT))
    .InputElement(CD3D11_INPUT_ELEMENT_DESC("NORMAL", DXGI_FORMAT_R32G32B32_FLOAT))
    .InputElement(CD3D11_INPUT_ELEMENT_DESC("TEXCOORD", DXGI_FORMAT_R32G32_FLOAT))
    .PixelShader("shaders/out/tunnel.mesh", "PsMesh")));
  // clang-format on

  INIT(_cbLines.Create());
  INIT(_cbComposite.Create());
  INIT(_cbMesh.Create());
  INIT(_cbFace.Create());

  // MeshLoader loader;
  // INIT_FATAL(loader.Load("gfx/newblob2.boba"));
  // INIT_FATAL(CreateScene(loader, SceneOptions(), &_scene));

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool Tunnel::Update(const UpdateState& state)
{
  vec3 pos(vec3(_freeflyCamera._pos));

  vec2 cameraParams = g_Blackboard->GetVec2Var("tunnel.cameraParams");
  _tunnelPlexusVerts.Clear();
  _tunnelOrgVerts.Clear();

  PlexusUpdate(state);
  UpdateCameraMatrix(state);
  return true;
}

//------------------------------------------------------------------------------
void Tunnel::PlexusUpdate(const UpdateState& state)
{
  float radius = g_Blackboard->GetFloatVar("tunnel.radius", state.localTime.TotalSecondsAsFloat());
  int numSegments = g_Blackboard->GetIntVar("tunnel.segments");

  int MAX_N = 16;
  int* neighbours = g_ScratchMemory.Alloc<int>(16 * 1024 * MAX_N);

  float START_OFS = -2;
  int tmp = (int)((_freeflyCamera._pos.z / Z_SPACING) / Z_SPACING * Z_SPACING);
  float distSnapped = (float)tmp;

  int numVerts = TUNNEL_DEPTH * numSegments;

  int idx = 0;
  for (int j = 0; j < TUNNEL_DEPTH; ++j)
  {
    vec3 pos = _spline.Interpolate(distSnapped + (float)j + START_OFS);

    float angle = 0;
    float angleInc = 2 * XM_PI / numSegments;
    for (int i = 0; i < numSegments; ++i)
    {
      vec3 p = pos + radius * vec3(cosf(angle), sinf(angle), 0);
      _tunnelOrgVerts.Append(p);
      //points[idx] = p;
      // add neighbours
      int n = 0;

      auto dn = [=](int n)
      {
        int tmp = i - n;
        return tmp < 0 ? tmp + numSegments : tmp;
      };
      auto up = [=](int n)
      {
        int tmp = i + n;
        return tmp % numSegments;
      };
      int dn1 = dn(1), dn2 = dn(2);
      int up1 = up(1), up2 = up(2);

      neighbours[idx * MAX_N + n++] = (j * numSegments) + dn1;
      neighbours[idx * MAX_N + n++] = (j * numSegments) + up1;

      if (j > 0)
      {
        neighbours[idx * MAX_N + n++] = ((j - 1) * numSegments) + i;
        neighbours[idx * MAX_N + n++] = ((j - 1) * numSegments) + dn1;
        neighbours[idx * MAX_N + n++] = ((j - 1) * numSegments) + up1;
      }

      if (j < TUNNEL_DEPTH - 1)
      {
        neighbours[idx * MAX_N + n++] = ((j + 1) * numSegments) + i;
        neighbours[idx * MAX_N + n++] = ((j + 1) * numSegments) + dn1;
        neighbours[idx * MAX_N + n++] = ((j + 1) * numSegments) + up1;
      }

      neighbours[idx * MAX_N + n] = -1;

      idx++;
      angle += angleInc;
    }
  }

  int plexusVerts = CalcPlexusGrouping(
      _tunnelPlexusVerts.Data(), _tunnelOrgVerts.Data(), idx, neighbours, MAX_N, _settings.plexus);
  _tunnelPlexusVerts.Resize(plexusVerts);
}

//------------------------------------------------------------------------------
bool Tunnel::FixedUpdate(const FixedUpdateState& state)
{
  // Update how far along the spline we've travelled
  float t = state.localTime.TotalSecondsAsFloat();
  float speed = g_Blackboard->GetFloatVar("tunnel.speed", t);
  float dirScale = g_Blackboard->GetFloatVar("tunnel.dirScale");
  _dist += state.delta * speed;

  vec3 pos = _cameraSpline.Interpolate(_dist / CAMERA_STEP);

  if (!_useFreeFly)
  {
    // ha, anything related to using lo here pretty much makes you sick :)
    float lo = g_Blackboard->GetFloatVar("Beat-Lo", state.globalTime.TotalSecondsAsFloat());

    float t = state.localTime.TotalSecondsAsFloat();
    _freeflyCamera._pos = pos + vec3(5 * sinf(t + _bonusTime), 5 * cosf(t + _bonusTime), 0);
    _freeflyCamera._dir = vec3(sinf(t) * dirScale, cosf(t) * dirScale, 1);
  }

  if (g_KeyUpTrigger.IsTriggered('1'))
    _useFreeFly = !_useFreeFly;

  //_freeflyCamera .SetFollowTarget(ToVector3(pos));
  _freeflyCamera.Update(state.delta);
  return true;
}

//------------------------------------------------------------------------------
void Tunnel::UpdateCameraMatrix(const UpdateState& state)
{
  Matrix view = _freeflyCamera._view;
  Matrix proj = _freeflyCamera._proj;

  Matrix viewProj = view * proj;
  _cbLines.gs0.world = Matrix::Identity();
  _cbLines.gs0.viewProj = viewProj.Transpose();
  _cbLines.gs0.cameraPos = _freeflyCamera._pos;

  float t = state.localTime.TotalSecondsAsFloat();
  _cbMesh.vs0.viewProj = viewProj.Transpose();
  _cbMesh.vs0.time = t;

  RenderTargetDesc desc = g_Graphics->GetBackBufferDesc();
  vec4 dim((float)desc.width, (float)desc.height, 0, 0);
  _cbFace.vs0.viewProj = viewProj.Transpose();
  //_cbFace.vs0.world = Matrix::Identity();
  //_cbFace.vs0.cameraPos = _freeflyCamera._pos;
  _cbFace.ps0.cameraPos = _freeflyCamera._pos;

  //_cbFace.gs0.dim = dim;
}

//------------------------------------------------------------------------------
bool Tunnel::Render()
{
  rmt_ScopedCPUSample(Tunnel_Render);

  static Color black(0, 0, 0, 0);
  FullscreenEffect* fullscreen = g_Graphics->GetFullscreenEffect();

  _ctx->SetSwapChain(g_Graphics->DefaultSwapChain(), black);

  ScopedRenderTargetFull rtColor(
      DXGI_FORMAT_R16G16B16A16_FLOAT, BufferFlag::CreateSrv, BufferFlag::CreateSrv);
  _ctx->SetRenderTarget(rtColor._rtHandle, rtColor._dsHandle, &black);

#if 1
  {
    // tunnel face

    ObjectHandle h = _tunnelFaceBundle.objects._vb;
    vec3* verts = _ctx->MapWriteDiscard<vec3>(h);
    memcpy(verts, _tunnelOrgVerts.Data(), _tunnelOrgVerts.DataSize());
    _ctx->Unmap(h);

    _cbFace.Set(_ctx, 0);
    _ctx->SetBundle(_tunnelFaceBundle);
    _ctx->DrawIndexed(_numTunnelFaceIndices, 0, 0);
  }

  {
    // tunnel

    ObjectHandle h = _linesBundle.objects._vb;
    vec3* verts = _ctx->MapWriteDiscard<vec3>(h);
    memcpy(verts, _tunnelPlexusVerts.Data(), _tunnelPlexusVerts.DataSize());
    _ctx->Unmap(h);

    _cbLines.gs0.dim = vec4((float)rtColor._desc.width, (float)rtColor._desc.height, 0, 0);
    vec3 params = g_Blackboard->GetVec3Var("tunnel.lineParams");
    _cbLines.ps0.lineParams = vec4(params.x, params.y, params.z, 1);
    _cbLines.Set(_ctx, 0);

    int numVerts = _tunnelPlexusVerts.Size();
    _ctx->SetBundle(_linesBundle);
    _ctx->Draw(numVerts, 0);
  }


#endif

  {
    // mesh
    _cbMesh.Set(_ctx, 0);
    _ctx->SetBundle(_meshBundle);

    for (scene::MeshBuffer* buf : _scene.meshBuffers)
    {
      _ctx->SetVertexBuffer(buf->vb);
      _ctx->SetIndexBuffer(buf->ib);

      for (scene::Mesh* mesh : buf->meshes)
      {
        Matrix mtx = mesh->mtxLocal;
        if (mesh->parentPtr)
        {
          mtx = mtx * mesh->parentPtr->mtxLocal;
        }

        _cbMesh.vs1.objWorld = mtx.Transpose();
        _cbMesh.Set(_ctx, 1);
        _ctx->DrawIndexed(mesh->indexCount, mesh->startIndexLocation, mesh->baseVertexLocation);
      }
    }
  }

  {
    // composite
    _cbComposite.ps0.tonemap = vec2(_settings.tonemap.exposure, _settings.tonemap.min_white);
    _cbComposite.Set(_ctx, 0);

    ObjectHandle inputs[] = {rtColor, rtColor._dsHandle};
    fullscreen->Execute(inputs,
        2,
        g_Graphics->GetBackBuffer(),
        g_Graphics->GetBackBufferDesc(),
        g_Graphics->GetDepthStencil(),
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

  ImGui::SliderFloat("eps", &_settings.plexus.eps, 0.1f, 25.0f);
  ImGui::SliderFloat("min-dist", &_settings.plexus.min_dist, 0.1f, 25.0f);
  ImGui::SliderFloat("max-dist", &_settings.plexus.max_dist, 10.0, 150.0f);
  ImGui::SliderInt("num-nearest", &_settings.plexus.num_nearest, 1, 20);
  ImGui::SliderInt("num-neighbours", &_settings.plexus.num_neighbours, 1, 100);

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
  _freeflyCamera._pos = vec3(0.f, 0.f, 0.f);
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
  g_DemoEngine->RegisterFactory(Name(), Tunnel::Create);
}
