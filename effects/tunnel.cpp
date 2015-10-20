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

static float START_OFS = -2;
static float Z_SPACING = 50;
static int CAMERA_STEP = 10;
static int TUNNEL_DEPTH = 100;
// note, this is mirrored in scratch.bb
static int TUNNEL_SEGMENTS = 50;

int GRID_SIZE = 20;
float CLOTH_SIZE = 10;

struct ClothStrip
{
  struct Particle
  {
    vec3 pos;
    vec3 lastPos;
    vec3 acc;
  };

  struct Constraint
  {
    Particle* p0;
    Particle* p1;
    float restLength;
  };

  //------------------------------------------------------------------------------
  void UpdateParticles(float dt)
  {
    size_t numParticles = _particles.size();
    float dt2 = dt * dt;

    vec3 gg = g_Blackboard->GetVec3Var("tunnel.gravity");
    float damping = g_Blackboard->GetFloatVar("tunnel.damping");

    // Add forces
    for (size_t i = 0; i < numParticles; ++i)
    {
      _particles[i].acc = gg;
      _particles[i].acc += 10 * vec3{randf(-1.f, 1.f), randf(-1.f, 1.f), randf(-1.f, 1.f)};
    }

    Particle* p = &_particles[0];
    for (size_t i = 0; i < numParticles; ++i)
    {
      // verlet integration
      vec3 tmp = p->pos;
      p->pos = p->pos + (1.0f - damping) * (p->pos - p->lastPos) + p->acc * dt2;
      p->lastPos = tmp;
      ++p;
    }

    // apply the constraints
    for (int i = 0; i < 2; ++i)
    {
      for (const Constraint& c : _constraints)
      {
        Particle* p0 = c.p0;
        Particle* p1 = c.p1;

        float dist = Distance(p0->pos, p1->pos);
        vec3 dir = ((dist - c.restLength) / dist) * (p1->pos - p0->pos);
        p0->pos += 0.5f * dir;
        p1->pos -= 0.5f * dir;
      }
    }

    vec3 aa(0, 0, 50);
    float w = 10;
    for (int j = 0; j < _clothDimX; ++j)
    {
      vec3 pp(aa.x - _clothDimX / 2 * w + j * 2, aa.y, aa.z);
      _particles[j].pos = pp;
    }


    // top row is fixed
    //float incX = CLOTH_SIZE / (_clothDimX - 1);
    //vec3 cur(-CLOTH_SIZE / 2.f, CLOTH_SIZE / 2.f, 0);
    //for (int i = 0; i < _clothDimX; ++i)
    //{
    //  _particles[i].pos = cur;
    //  cur.x += incX;
    //  ++p;
    //}

    // CopyOut
  }

  //------------------------------------------------------------------------------
  void Create(int dimX, int dimY)
  {
    int numParticles = dimX * dimY;

    _clothDimX = dimX;
    _clothDimY = dimY;
    _numParticles = numParticles;

    // create the grid
    //vector<u32> indices((width - 1)*(height - 1) * 2 * 3);
    //u32* idx = &indices[0];

    //for (int i = 0; i < height - 1; ++i)
    //{
    //  for (int j = 0; j < width - 1; ++j)
    //  {
    //    // 0--1
    //    // 2--3
    //    u32 i0 = (i + 0)*width + j + 0;
    //    u32 i1 = (i + 0)*width + j + 1;
    //    u32 i2 = (i + 1)*width + j + 0;
    //    u32 i3 = (i + 1)*width + j + 1;

    //    idx[0] = i0;
    //    idx[1] = i1;
    //    idx[2] = i2;

    //    idx[3] = i2;
    //    idx[4] = i1;
    //    idx[5] = i3;

    //    _numTris += 2;
    //    idx += 6;
    //  }
    //}
    //_clothGpuObjects.CreateIndexBuffer(
    //    (u32)indices.size() * sizeof(u32), DXGI_FORMAT_R32_UINT, indices.data());

    _particles.resize(numParticles);
    Particle* cur = &_particles[0];
    for (int i = 0; i < dimY; ++i)
    {
      vec3 p(0, -i * 20.f, 50);
      float w = 10;
      for (int j = 0; j < dimX; ++j)
      {
        vec3 pp(p.x - dimX/2 * w + j * 2, p.y, p.z);
        cur->pos = pp;
        cur->lastPos = pp;
        cur->acc = vec3{0,0,0};
        cur++;
      }
    }

    //ResetParticles();

    // create cloth constraints
    // each particle is connected horiz, vert and diag (both 1 and 2 steps away)
    for (int i = 0; i < dimY; ++i)
    {
      for (int j = 0; j < dimX; ++j)
      {
        u32 idx0 = i*dimX + j;
        Particle* p0 = &_particles[idx0];

        static int ofs[] = {
          -1, +0,
          -1, +1,
          +0, +1,
          +1, +1,
          +1, +0,
          +1, -1,
          +0, -1,
          -1, -1
        };

        for (int idx = 0; idx < 8; ++idx)
        {
          for (int s = 1; s <= 2; ++s)
          {
            int xx = j + s * ofs[idx * 2 + 0];
            int yy = i + s * ofs[idx * 2 + 1];
            if (xx < 0 || xx >= dimX || yy < 0 || yy >= dimY)
              continue;

            u32 idx1 = yy*dimX + xx;
            Particle* p1 = &_particles[idx1];
            _constraints.push_back({ p0, p1, Distance(p0->pos, p1->pos) });
          }
        }
      }
    }
  }

  vec3 _gravity = vec3{0, 0, 0};
  float _damping = 0.99f;
  int _clothDimX, _clothDimY;
  int _numParticles = 0;
  int _numTris = 0;

  vector<Particle> _particles;
  vector<Constraint> _constraints;
};

ClothStrip g_cloth;

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
  INIT_FATAL(_tunnelFaceBundle.Create(BundleOptions()
    .VertexShader("shaders/out/tunnel.facecolor", "VsFace")
    .GeometryShader("shaders/out/tunnel.facecolor", "GsFace")
    .PixelShader("shaders/out/tunnel.facecolor", "PsFace")
    .InputElement(CD3D11_INPUT_ELEMENT_DESC("POSITION", DXGI_FORMAT_R32G32B32_FLOAT))
    .RasterizerDesc(rasterizeDescCullNone)
    .DynamicVb(128 * 1024, sizeof(vec3))));

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

  INIT(_particleBundle.Create(BundleOptions()
    .DynamicVb(1024 * 1024, sizeof(vec3))
    .VertexShader("shaders/out/tunnel.particle", "VsParticle")
    .GeometryShader("shaders/out/tunnel.particle", "GsParticle")
    .PixelShader("shaders/out/tunnel.particle", "PsParticle")
    .InputElement(CD3D11_INPUT_ELEMENT_DESC("POSITION", DXGI_FORMAT_R32G32B32_FLOAT))
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_POINTLIST)
    .DepthStencilDesc(depthDescDepthWriteDisabled)
    .BlendDesc(blendDescBlendOneOne)
    .RasterizerDesc(rasterizeDescCullNone)));

  // clang-format on

  g_cloth.Create(4, 20);

  INIT_FATAL(_cbLines.Create());
  INIT_FATAL(_cbComposite.Create());
  INIT_FATAL(_cbMesh.Create());
  INIT_FATAL(_cbFace.Create());
  INIT_FATAL(_cbParticle.Create());

  // Particles
  INIT_RESOURCE_FATAL(_particleTexture, RESOURCE_MANAGER.LoadTexture("gfx/particle_white.png"));

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
  _tunnelFaceVerts.Clear();

  PlexusUpdate(state);
  UpdateCameraMatrix(state);
  return true;
}

//------------------------------------------------------------------------------
void Tunnel::PlexusUpdate(const UpdateState& state)
{
  float radius = g_Blackboard->GetFloatVar("tunnel.radius", state.localTime.TotalSecondsAsFloat());
  int numSegments = g_Blackboard->GetIntVar("tunnel.segments");

  vec3* points = g_ScratchMemory.Alloc<vec3>(16 * 1024);
  int MAX_N = 16;
  int* neighbours = g_ScratchMemory.Alloc<int>(16 * 1024 * MAX_N);

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
      points[idx] = p;

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

  // Add the face polygons
  {
    float a = g_Blackboard->GetFloatVar("tunnel.ofsScale");
    float b = g_Blackboard->GetFloatVar("tunnel.lenScale");
    float c = g_Blackboard->GetFloatVar("tunnel.rScale");
    float prob = g_Blackboard->GetFloatVar("tunnel.rProb");
    float distSnapped = (float)tmp;
    for (int j = 0; j < TUNNEL_DEPTH - 1; ++j)
    {
      vec3 pos = _spline.Interpolate(distSnapped + (float)j + START_OFS);

      int startSegment = (int)(pos.z * a) % TUNNEL_SEGMENTS;
      int numSegments = (int)(pos.z * b) % TUNNEL_SEGMENTS;

      // check if we should draw this segment at all
      float zz = (pos.z * c) - floorf(pos.z * c);
      if (prob < zz)
        continue;

      for (int i = 0; i < numSegments; ++i)
      {
        int s = (i + startSegment) % TUNNEL_SEGMENTS;
        _tunnelFaceVerts.Append(points[(j + 1) * TUNNEL_SEGMENTS + s]);
        _tunnelFaceVerts.Append(points[(j + 0) * TUNNEL_SEGMENTS + s]);
        _tunnelFaceVerts.Append(points[(j + 1) * TUNNEL_SEGMENTS + (s + 1) % TUNNEL_SEGMENTS]);

        _tunnelFaceVerts.Append(points[(j + 1) * TUNNEL_SEGMENTS + (s + 1) % TUNNEL_SEGMENTS]);
        _tunnelFaceVerts.Append(points[(j + 0) * TUNNEL_SEGMENTS + s]);
        _tunnelFaceVerts.Append(points[(j + 0) * TUNNEL_SEGMENTS + (s + 1) % TUNNEL_SEGMENTS]);

        _numTunnelFaces += 2;
      }
    }
  }

  int plexusVerts = CalcPlexusGrouping(
      _tunnelPlexusVerts.Data(), points, idx, neighbours, MAX_N, _settings.plexus);
  _tunnelPlexusVerts.Resize(plexusVerts);
}

//------------------------------------------------------------------------------
bool Tunnel::FixedUpdate(const FixedUpdateState& state)
{
  g_cloth.UpdateParticles(state.delta);

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
  _cbLines.gs0.view = view.Transpose();
  _cbLines.gs0.cameraPos = _freeflyCamera._pos;

  float t = state.localTime.TotalSecondsAsFloat();
  _cbMesh.vs0.viewProj = viewProj.Transpose();
  _cbMesh.vs0.time = t;

  RenderTargetDesc desc = g_Graphics->GetBackBufferDesc();
  vec4 dim((float)desc.width, (float)desc.height, 0, 0);
  _cbFace.vs0.viewProj = viewProj.Transpose();
  _cbFace.ps0.cameraPos = _freeflyCamera._pos;

  // The depth value written to the z-buffer is after the w-divide,
  // but the z-value we compare against is still in proj-space, so
  // we'll need to do the backward transform:
  // f*(z-n) / (f-n)*z = zbuf => z = f*n / (f-zbuf(f-n))
  float n = _freeflyCamera._nearPlane;
  float f = _freeflyCamera._farPlane;

  _cbParticle.ps0.nearFar = vec4(n, f, f * n, f - n);
  _cbParticle.gs0.world = Matrix::Identity();
  _cbParticle.gs0.viewProj = viewProj.Transpose();
  _cbParticle.gs0.cameraPos = _freeflyCamera._pos;
}

//------------------------------------------------------------------------------
bool Tunnel::Render()
{
  rmt_ScopedCPUSample(Tunnel_Render);

  static Color black(0, 0, 0, 0);
  FullscreenEffect* fullscreen = g_Graphics->GetFullscreenEffect();

  _ctx->SetSwapChain(g_Graphics->DefaultSwapChain(), black);

  ScopedRenderTargetFull rtColor(
    DXGI_FORMAT_R11G11B10_FLOAT, BufferFlag::CreateSrv, BufferFlag::CreateSrv);
  _ctx->SetRenderTarget(rtColor._rtHandle, rtColor._dsHandle, &black);

  {
    // tunnel face

    ObjectHandle h = _tunnelFaceBundle.objects._vb;
    vec3* verts = _ctx->MapWriteDiscard<vec3>(h);
    memcpy(verts, _tunnelFaceVerts.Data(), _tunnelFaceVerts.DataSize());
    _ctx->Unmap(h);

    _cbFace.Set(_ctx, 0);
    _ctx->SetBundle(_tunnelFaceBundle);
    _ctx->Draw(_tunnelFaceVerts.Size(), 0);
  }

  ScopedRenderTarget rtLines(DXGI_FORMAT_R11G11B10_FLOAT, BufferFlag::CreateSrv);
  _ctx->SetRenderTarget(rtLines, g_Graphics->GetBackBuffer(), &black);

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

  _ctx->UnsetRenderTargets(0, 1);

  {
    // particles
    vec3* vtx = _ctx->MapWriteDiscard<vec3>(_particleBundle.objects._vb);

    for (int i = 0; i < g_cloth._numParticles; ++i)
    {
      vtx[i] = g_cloth._particles[i].pos;
    }

#if 0
    int numBoids = 1000;
    for (int i = 0; i < numBoids; ++i)
    {
      vec3 pos = _spline.Interpolate(i + START_OFS);
      vtx[i] = pos;
    }
#endif
    _ctx->Unmap(_particleBundle.objects._vb);

    _cbParticle.Set(_ctx, 0);
    _ctx->SetBundleWithSamplers(_particleBundle, ShaderType::PixelShader);

    // Unset the DSV, as we want to use it as a texture resource
    _ctx->SetRenderTarget(rtColor._rtHandle, ObjectHandle(), nullptr);
    ObjectHandle srv[] = { _particleTexture, rtColor._dsHandle };
    _ctx->SetShaderResources(srv, 2, ShaderType::PixelShader);
    _ctx->Draw(g_cloth._numParticles, 0);
    _ctx->UnsetShaderResources(0, 2, ShaderType::PixelShader);
  }

  ScopedRenderTarget rtColorBlurred(rtColor._desc, BufferFlag::CreateSrv | BufferFlag::CreateUav);
  fullscreen->Blur(rtColor, rtColorBlurred, rtColorBlurred._desc, 10, 2);

  {
    // composite
    _cbComposite.ps0.tonemap = vec2(_settings.tonemap.exposure, _settings.tonemap.min_white);
    _cbComposite.Set(_ctx, 0);

    ObjectHandle inputs[] = {rtColor, rtLines, rtColorBlurred};
    fullscreen->Execute(inputs,
        3,
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
void Tunnel::SaveParameterSet(bool inc)
{
  SaveSettings(_settings, inc);
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
