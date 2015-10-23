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

//------------------------------------------------------------------------------
Snake::Snake(RandomUniform& randomFloat,
    int dimX,
    int dimY,
    float segmentWidth,
    float segmentHeight,
    const vec3& anchor,
    const vec3& dir)
    : _clothDimX(dimX)
    , _clothDimY(dimY)
    , _segmentWidth(segmentWidth)
    , _segmentHeight(segmentHeight)
    , _anchor(anchor)
    , _dir(dir)
    , _numParticles(dimX * dimY)
    , _forceAngle(randomFloat.Next(-2.f, 2.f))
{
  _particles.resize(_numParticles);
  Particle* p = &_particles[0];

  for (int i = 0; i < dimY; ++i)
  {
    vec3 cur = anchor + (float)(i * segmentHeight) * dir;
    for (int j = 0; j < dimX; ++j)
    {
      cur.x = (-((dimX / 2 - 1) + 0.5f) + j) * segmentWidth;
      p->pos = cur;
      p->lastPos = cur;
      p->acc = vec3{0, 0, 0};
      p++;
    }
  }

  // if a rope, only add constraints in the y-dimension
  if (dimX == 1)
  {
    // create cloth constraints
    // each particle is connected horiz, vert and diag (both 1 and 2 steps away)
    for (int i = 0; i < dimY; ++i)
    {
      u32 idx0 = i * dimX;
      Particle* p0 = &_particles[idx0];

      static int ofs[] = {-1, +1};

      for (int idx = 0; idx < 2; ++idx)
      {
        for (int s = 1; s <= 2; ++s)
        {
          int yy = i + s * ofs[idx];
          if (yy < 0 || yy >= dimY)
            continue;

          u32 idx1 = yy * dimX;
          Particle* p1 = &_particles[idx1];
          //_constraints.push_back({ p0, p1, Distance(p0->pos, p1->pos) });
          _constraints.push_back({idx0, idx1, Distance(p0->pos, p1->pos)});
        }
      }
    }
  }
  else
  {
    // create cloth constraints
    // each particle is connected horiz, vert and diag (both 1 and 2 steps away)
    for (int i = 0; i < dimY; ++i)
    {
      for (int j = 0; j < dimX; ++j)
      {
        u32 idx0 = i * dimX + j;
        Particle* p0 = &_particles[idx0];

        static int ofs[] = {-1, +0, -1, +1, +0, +1, +1, +1, +1, +0, +1, -1, +0, -1, -1, -1};

        for (int idx = 0; idx < 8; ++idx)
        {
          for (int s = 1; s <= 2; ++s)
          {
            int xx = j + s * ofs[idx * 2 + 0];
            int yy = i + s * ofs[idx * 2 + 1];
            if (xx < 0 || xx >= dimX || yy < 0 || yy >= dimY)
              continue;

            u32 idx1 = yy * dimX + xx;
            Particle* p1 = &_particles[idx1];
            //_constraints.push_back({ p0, p1, Distance(p0->pos, p1->pos) });
            _constraints.push_back({idx0, idx1, Distance(p0->pos, p1->pos)});
          }
        }
      }
    }
  }
}

//------------------------------------------------------------------------------
void Snake::Update(float dt)
{
  size_t numParticles = _particles.size();
  float dt2 = dt * dt;

  vec3 gg = g_Blackboard->GetVec3Var("tunnel.gravity");
  float damping = g_Blackboard->GetFloatVar("tunnel.damping");
  float windForce = g_Blackboard->GetFloatVar("tunnel.windForce");

  vec3 force = windForce * vec3{sinf(_forceAngle), 0, cosf(_forceAngle)};
  _forceAngle += dt * g_Blackboard->GetFloatVar("tunnel.forceSpeed");

  // Add forces
  for (size_t i = 0; i < numParticles; ++i)
  {
    _particles[i].acc = gg;
    _particles[i].acc += force;
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
      Particle* p0 = &_particles[c.i0];
      Particle* p1 = &_particles[c.i1];

      float dist = Distance(p0->pos, p1->pos);
      vec3 dir = ((dist - c.restLength) / dist) * (p1->pos - p0->pos);
      p0->pos += 0.5f * dir;
      p1->pos -= 0.5f * dir;
    }
  }

  // fix the upper row
  vec3 cur = _anchor;
  for (int i = 0; i < _clothDimX; ++i)
  {
    _particles[i].pos = cur;
  }
}

//------------------------------------------------------------------------------
int Snake::CopyOutLines(vec3* out)
{
  int numVerts = 0;
  for (int i = 0; i < _clothDimY; ++i)
  {
    *out++ = _particles[i * _clothDimX].pos;
    *out++ = _particles[i * _clothDimX + _clothDimX - 1].pos;
    numVerts += 2;
  }

  for (int i = 0; i < _clothDimY - 1; ++i)
  {
    *out++ = _particles[i * _clothDimX].pos;
    *out++ = _particles[(i + 1) * _clothDimX].pos;

    *out++ = _particles[i * _clothDimX + _clothDimX - 1].pos;
    *out++ = _particles[(i + 1) * _clothDimX + _clothDimX - 1].pos;
    numVerts += 4;
  }

  return numVerts;
}

//------------------------------------------------------------------------------
// ClothStrip g_cloth(1, 75, 10, 2, vec3(0, 100, 200), vec3(0, -1, 0));
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

    float radius = g_Blackboard->GetFloatVar("tunnel.radius");

    for (int i = 0; i < numPoints; ++i)
    {
      float xOfs = _randomFloat.Next(-s, +s);
      float yOfs = _randomFloat.Next(-s, +s);

      controlPoints.push_back(cur);
      cur += vec3(xOfs, yOfs, Z_SPACING);

      if ((i % CAMERA_STEP) == 0)
        cameraControlPoints.push_back(cur);
    }

    _spline.Create(controlPoints.data(), (int)controlPoints.size());
    _cameraSpline.Create(cameraControlPoints.data(), (int)cameraControlPoints.size());

    for (int i = 0; i < 20; ++i)
    {
      vec3 cur = _spline.Interpolate(i * 10.f + START_OFS);
      _snakes.push_back(Snake{_randomFloat, 1, 50, 5, 5, cur + vec3{0, radius, 0}, vec3{0, -1, 0}});
    }
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
    .DepthStencilDesc(depthDescDepthWriteDisabled)
    .DynamicVb(128 * 1024, sizeof(vec3))
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST)));

  INIT_FATAL(_snakeBundle.Create(BundleOptions()
    .VertexShader("shaders/out/tunnel.lines", "VsTunnelLines")
    .GeometryShader("shaders/out/tunnel.lines", "GsTunnelLines")
    .PixelShader("shaders/out/tunnel.lines", "PsTunnelLines")
    .VertexFlags(VF_POS)
    .RasterizerDesc(rasterizeDescCullNone)
    .BlendDesc(blendDescBlendOneOne)
    .DepthStencilDesc(depthDescDepthWriteDisabled)
    .DynamicVb(128 * 1024, sizeof(vec3))
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST)));

  INIT_FATAL(_compositeBundle.Create(BundleOptions()
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/tunnel.composite", "PsComposite")));

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

  INIT_FATAL(_cbLines.Create());
  INIT_FATAL(_cbComposite.Create());
  INIT_FATAL(_cbFace.Create());
  INIT_FATAL(_cbParticle.Create());

  // Particles
  INIT_RESOURCE_FATAL(_particleTexture, RESOURCE_MANAGER.LoadTexture("gfx/particle_white.png"));

  for (int i = 0; i < 1000; ++i)
  {
    for (Snake& s : _snakes)
    {
      s.Update(0.01f);
    }
  }

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool Tunnel::Update(const UpdateState& state)
{
  for (Snake& s : _snakes)
  {
    s.Update(state.delta.TotalSecondsAsFloat());
  }

  vec3 pos(vec3(_freeflyCamera._pos));

  vec2 cameraParams = g_Blackboard->GetVec2Var("tunnel.cameraParams");
  _tunnelPlexusVerts.Clear();
  _tunnelFaceVerts.Clear();

  _cbComposite.ps0.time =
    vec2(state.localTime.TotalSecondsAsFloat(), state.globalTime.TotalSecondsAsFloat());

  PlexusUpdate(state);
  UpdateCameraMatrix(state);
  return true;
}

//------------------------------------------------------------------------------
void Tunnel::PlexusUpdate(const UpdateState& state)
{
  float radius = g_Blackboard->GetFloatVar("tunnel.radius");
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
  // Update how far along the spline we've travelled
  float t = state.localTime.TotalSecondsAsFloat();
  float speed = g_Blackboard->GetFloatVar("tunnel.speed", t);
  float dirScale = g_Blackboard->GetFloatVar("tunnel.dirScale");
  _dist += state.delta * speed;

  vec3 pos = _cameraSpline.Interpolate(_dist / CAMERA_STEP);

  if (!_useFreeFly)
  {
    float t = state.localTime.TotalSecondsAsFloat();
    _freeflyCamera._pos = pos + vec3(5 * sinf(t), 5 * cosf(t), 0);
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

  RenderTargetDesc desc = g_Graphics->GetBackBufferDesc();
  vec4 dim((float)desc.width, (float)desc.height, 0, 0);
  _cbFace.vs0.viewProj = viewProj.Transpose();
  _cbFace.vs0.view = view.Transpose();
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
  {
    // tunnel face
    _ctx->SetRenderTarget(rtColor._rtHandle, rtColor._dsHandle, &black);

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

  {
    // snakes!
    ObjectHandle h = _snakeBundle.objects._vb;
    vec3* verts = _ctx->MapWriteDiscard<vec3>(h);
    int numVerts = 0;
    for (Snake& c : _snakes)
    {
      int tmp = c.CopyOutLines(verts);
      verts += tmp;
      numVerts += tmp;
    }
    _ctx->Unmap(h);

    _cbLines.gs0.dim = vec4((float)rtColor._desc.width, (float)rtColor._desc.height, 0, 0);
    vec3 params = g_Blackboard->GetVec3Var("tunnel.snakeLineParams");
    _cbLines.ps0.lineParams = vec4(params.x, params.y, params.z, 1);
    _cbLines.Set(_ctx, 0);

    _ctx->SetBundle(_snakeBundle);
    _ctx->Draw(numVerts, 0);

    // snake particles
    _cbParticle.gs0.particleSize = g_Blackboard->GetFloatVar("tunnel.snakeParticleSize");
    _cbParticle.ps0.particleColor = g_Blackboard->GetVec4Var("tunnel.snakeParticleColor");

    _cbParticle.Set(_ctx, 0);
    _ctx->SetBundleWithSamplers(_particleBundle, ShaderType::PixelShader);
    _ctx->SetVertexBuffer(h);

    // Unset the DSV, as we want to use it as a texture resource
    _ctx->SetRenderTarget(rtColor._rtHandle, ObjectHandle(), nullptr);
    ObjectHandle srv[] = {_particleTexture, rtColor._dsHandle};
    _ctx->SetShaderResources(srv, 2, ShaderType::PixelShader);
    _ctx->Draw(numVerts, 0);
    _ctx->UnsetShaderResources(0, 2, ShaderType::PixelShader);
  }
  _ctx->UnsetRenderTargets(0, 1);

  {
    // particles
    vec3* vtx = _ctx->MapWriteDiscard<vec3>(_particleBundle.objects._vb);

    int numParticles = 1000;
    for (int i = 0; i < numParticles; ++i)
    {
      vec3 pos = _spline.Interpolate(i + START_OFS);
      vtx[i] = pos;
    }
    _ctx->Unmap(_particleBundle.objects._vb);
    _cbParticle.gs0.particleSize = g_Blackboard->GetFloatVar("tunnel.cloudParticleSize");
    _cbParticle.ps0.particleColor = g_Blackboard->GetVec4Var("tunnel.cloudParticleColor");
    _cbParticle.Set(_ctx, 0);
    _ctx->SetBundleWithSamplers(_particleBundle, ShaderType::PixelShader);

    // Unset the DSV, as we want to use it as a texture resource
    _ctx->SetRenderTarget(rtColor._rtHandle, ObjectHandle(), nullptr);
    ObjectHandle srv[] = {_particleTexture, rtColor._dsHandle};
    _ctx->SetShaderResources(srv, 2, ShaderType::PixelShader);
    _ctx->Draw(numParticles, 0);
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
