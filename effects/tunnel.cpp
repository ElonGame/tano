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
  INIT_FATAL(_linesBundle.Create(BundleOptions()
    .VertexShader("shaders/out/tunnel.lines", "VsTunnelLines")
    .GeometryShader("shaders/out/tunnel.lines", "GsTunnelLines")
    .PixelShader("shaders/out/tunnel.lines", "PsTunnelLines")
    .VertexFlags(VF_POS)
    .RasterizerDesc(rasterizeDescCullNone)
    .BlendDesc(blendDescBlendOneOne)
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

  //MeshLoader loader;
  //INIT_FATAL(loader.Load("gfx/newblob2.boba"));
  //INIT_FATAL(CreateScene(loader, SceneOptions(), &_scene));

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool Tunnel::Update(const UpdateState& state)
{
  vec3 pos(vec3(_freeflyCamera._pos));

  vec2 cameraParams = BLACKBOARD.GetVec2Var("tunnel.cameraParams");
  _tunnelVerts.Clear();

  // NormalUpdate(state);
  PlexusUpdate(state);
  UpdateCameraMatrix(state);
  return true;
}

//------------------------------------------------------------------------------
void Tunnel::PlexusUpdate(const UpdateState& state)
{
  float radius = BLACKBOARD.GetFloatVar("tunnel.radius", state.localTime.TotalSecondsAsFloat());
  int numSegments = BLACKBOARD.GetIntVar("tunnel.segments");

  vec3* points = g_ScratchMemory.Alloc<vec3>(16 * 1024);
  int MAX_N = 16;
  int* neighbours = g_ScratchMemory.Alloc<int>(16 * 1024 * MAX_N);

  float START_OFS = -2;
  int tmp = (int)((_freeflyCamera._pos.z / Z_SPACING) / Z_SPACING * Z_SPACING);
  float distClamped = (float)tmp;
  int depth = 50;

  int num = depth * numSegments;

  int idx = 0;
  for (int j = 0; j < depth; ++j)
  {
    vec3 pos = _spline.Interpolate(distClamped + (float)j + START_OFS);

    float angle = 0;
    float angleInc = 2 * XM_PI / numSegments;
    for (int i = 0; i < numSegments; ++i)
    {
      vec3 p = pos + radius * vec3(cosf(angle), sinf(angle), 0);
      points[idx] = p;
      // add neighbours
      int n = 0;

      auto dn = [=](int n) { int tmp = i - n; return tmp < 0 ? tmp + numSegments : tmp; };
      auto up = [=](int n) { int tmp = i + n; return tmp % numSegments; };
      int dn1 = dn(1), dn2 = dn(2);
      int up1 = up(1), up2 = up(2);

      neighbours[idx * MAX_N + n++] = (j * numSegments) + dn1;
      neighbours[idx * MAX_N + n++] = (j * numSegments) + up1;

      if (j > 0)
      {
        neighbours[idx * MAX_N + n++] = ((j - 1) * numSegments) + i;
        neighbours[idx * MAX_N + n++] = ((j - 1) * numSegments) + dn1;
        //neighbours[idx * MAX_N + n++] = ((j - 1) * numSegments) + dn2;
        neighbours[idx * MAX_N + n++] = ((j - 1) * numSegments) + up1;
        //neighbours[idx * MAX_N + n++] = ((j - 1) * numSegments) + up2;
      }

      if (j < depth - 1)
      {
        neighbours[idx * MAX_N + n++] = ((j + 1) * numSegments) + i;
        neighbours[idx * MAX_N + n++] = ((j + 1) * numSegments) + dn1;
        //neighbours[idx * MAX_N + n++] = ((j + 1) * numSegments) + dn2;
        neighbours[idx * MAX_N + n++] = ((j + 1) * numSegments) + up1;
        //neighbours[idx * MAX_N + n++] = ((j + 1) * numSegments) + up2;
      }

      neighbours[idx * MAX_N + n] = -1;

      idx++;
      angle += angleInc;
    }
  }

  int numVerts = CalcPlexusGrouping(_tunnelVerts.Data(), points, idx, neighbours, MAX_N, _settings.plexus);
  _tunnelVerts.Resize(numVerts);
}

//------------------------------------------------------------------------------
void Tunnel::NormalUpdate(const UpdateState& state)
{
  float radius = BLACKBOARD.GetFloatVar("tunnel.radius", state.localTime.TotalSecondsAsFloat());
  int numSegments = BLACKBOARD.GetIntVar("tunnel.segments");

  SimpleAppendBuffer<vec3, 512> ring1, ring2;
  SimpleAppendBuffer<vec3, 512>* rings[] = {&ring1, &ring2};

  float START_OFS = -2;
  int tmp = (int)((_freeflyCamera._pos.z / Z_SPACING) / Z_SPACING * Z_SPACING);
  float distClamped = (float)tmp;
  // create the first 2 rings
  for (int j = 0; j < 2; ++j)
  {
    vec3 pos = _spline.Interpolate(distClamped + (float)j + START_OFS);
    float angle = 0;
    float angleInc = 2 * XM_PI / numSegments;
    vec3 prev = pos + radius * vec3(cosf(angle), sinf(angle), 0);
    for (int i = 0; i < numSegments; ++i)
    {
      angle += angleInc;
      vec3 p = pos + radius * vec3(cosf(angle), sinf(angle), 0);
      rings[j]->Append(prev);
      rings[j]->Append(p);
      prev = p;
    }
  }

  for (int j = 0; j < 50; ++j)
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
    vec3 pos = _spline.Interpolate(distClamped + (float)j + START_OFS);

    float angle = 0;
    float angleInc = 2 * XM_PI / numSegments;
    vec3 prev = pos + radius * vec3(cosf(angle), sinf(angle), 0);
    for (int i = 0; i < numSegments; ++i)
    {
      angle += angleInc;
      vec3 p = pos + radius * vec3(cosf(angle), sinf(angle), 0);
      ring0.Append(prev);
      ring0.Append(p);
      prev = p;
    }

    swap(rings[0], rings[1]);
  }
}

//------------------------------------------------------------------------------
bool Tunnel::FixedUpdate(const FixedUpdateState& state)
{
  // Update how far along the spline we've travelled
  float t = state.localTime.TotalSecondsAsFloat();
  float speed = BLACKBOARD.GetFloatVar("tunnel.speed", t);
  float dirScale = BLACKBOARD.GetFloatVar("tunnel.dirScale");
  _dist += state.delta * speed;

  vec3 pos = _cameraSpline.Interpolate(_dist / CAMERA_STEP);

  static bool useFreeFly = true;
  if (!useFreeFly)
  {
    _freeflyCamera._pos = pos + vec3(5 * sinf(state.localTime.TotalSecondsAsFloat()),
      5 * cosf(state.localTime.TotalSecondsAsFloat()),
      0);

    _freeflyCamera._dir = vec3(sinf(state.localTime.TotalSecondsAsFloat()) * dirScale,
      cosf(state.localTime.TotalSecondsAsFloat()) * dirScale,
      1);
  }

  if (g_KeyUpTrigger.IsTriggered('1'))
    useFreeFly = !useFreeFly;

  //_freeflyCamera.SetFollowTarget(ToVector3(pos));
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

  _cbMesh.vs0.viewProj = viewProj.Transpose();
  _cbMesh.vs0.time = state.localTime.TotalSecondsAsFloat();
}

//------------------------------------------------------------------------------
bool Tunnel::Render()
{
  rmt_ScopedCPUSample(Tunnel_Render);

  static Color black(0, 0, 0, 0);
  FullscreenEffect* fullscreen = GRAPHICS.GetFullscreenEffect();

  _ctx->SetSwapChain(GRAPHICS.DefaultSwapChain(), black);

  ScopedRenderTargetFull rtColor(
      DXGI_FORMAT_R16G16B16A16_FLOAT, BufferFlag::CreateSrv, BufferFlag::CreateSrv);
  _ctx->SetRenderTarget(rtColor._rtHandle, rtColor._dsHandle, &black);

#if 1
  {
    // tunnel
    _cbLines.gs0.dim = vec4((float)rtColor._desc.width, (float)rtColor._desc.height, 0, 0);
    vec3 params = BLACKBOARD.GetVec3Var("tunnel.lineParams");
    _cbLines.ps0.lineParams = vec4(params.x, params.y, params.z, 1);
    _cbLines.Set(_ctx, 0);

    ObjectHandle h = _linesBundle.objects._vb;
    vec3* verts = _ctx->MapWriteDiscard<vec3>(h);
    memcpy(verts, _tunnelVerts.Data(), _tunnelVerts.DataSize());
    _ctx->Unmap(h);

    int numVerts = _tunnelVerts.Size();
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
  //_freeflyCamera._pitch = _freeflyCamera._yaw = _freeflyCamera._roll = 0.f;

  //_greetsBlock.Init();
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
