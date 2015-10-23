#include "credits.hpp"
#include "../tano.hpp"
#include "../graphics.hpp"
#include "../graphics_context.hpp"
#include "../demo_engine.hpp"
#include "../resource_manager.hpp"
#include "../init_sequence.hpp"
#include "../generated/demo.parse.hpp"
#include "../mesh_utils.hpp"
#include "../fullscreen_effect.hpp"
#include "../stop_watch.hpp"
#include "../blackboard.hpp"
#include "../random.hpp"

/*
  update timing:

  doozblade
  base: 6.9 ms
  conversion to V3: 5.8 ms

  mothership:
  V3: 2.5 ms
  back to p0/p1 constraints: 2.3 ms
  4 constraints at a time: 0.9 ms. Yeah, this is probably fast enough for now :)
*/

using namespace tano;
using namespace bristol;
using namespace DirectX;

namespace
{
  int GRID_SIZE = 20;
  float CLOTH_SIZE = 10;

  int MAX_PARTICLES = 16 * 1024;
}

StopWatch g_stopWatch;
static RandomGauss150 RANDOM_150;

//------------------------------------------------------------------------------
Credits::Credits(const string &name, const string& config, u32 id)
  : BaseEffect(name, config, id)
  , _avgUpdate(100)
{
}

//------------------------------------------------------------------------------
Credits::~Credits()
{
}

//------------------------------------------------------------------------------
bool Credits::OnConfigChanged(const vector<char>& buf)
{
  _settings = ParseCreditsSettings(InputBuffer(buf));
  return true;
}

//------------------------------------------------------------------------------
bool Credits::Init()
{
  BEGIN_INIT_SEQUENCE();

  _freeflyCamera.FromProtocol(_settings.camera);

  {
    // setup text camera
    float zz = -100;
    _textCamera._pos.z = zz;
    _textCamera._fov = atan(100.f / fabsf(zz));
  }

  CD3D11_RASTERIZER_DESC rasterDesc = CD3D11_RASTERIZER_DESC(CD3D11_DEFAULT());
  rasterDesc.CullMode = D3D11_CULL_NONE;
  rasterDesc.FillMode = D3D11_FILL_WIREFRAME;
  INIT(_clothState.Create(nullptr, nullptr, &rasterDesc));

  INIT(_clothGpuObjects.LoadVertexShader("shaders/out/basic", "VsPos", VF_POS));
  INIT(_clothGpuObjects.LoadPixelShader("shaders/out/basic", "PsPos"));

  // clang-format off
  INIT(_compositeBundle.Create(BundleOptions()
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/credits.composite", "PsComposite")));

  INIT(_particleBundle.Create(BundleOptions()
    .VertexShader("shaders/out/credits.particle", "VsParticle")
    .GeometryShader("shaders/out/credits.particle", "GsParticle")
    .PixelShader("shaders/out/credits.particle", "PsParticle")
    .InputElement(CD3D11_INPUT_ELEMENT_DESC("POSITION", DXGI_FORMAT_R32G32B32A32_FLOAT))
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_POINTLIST)
    .DynamicVb(MAX_PARTICLES, sizeof(vec4))
    .DepthStencilDesc(depthDescDepthDisabled)
    .BlendDesc(blendDescPreMultipliedAlpha)
    .RasterizerDesc(rasterizeDescCullNone)));

  INIT(_textBundle.Create(BundleOptions()
    .PixelShader("shaders/out/intro.text", "PsIntroTextDistort")));

  // clang-format on

  INIT_RESOURCE(_csBlur, g_Graphics->LoadComputeShaderFromFile("shaders/out/credits.blur", "BoxBlurY"));

  INIT_RESOURCE(_particleTexture, RESOURCE_MANAGER.LoadTexture(_settings.particle_texture.c_str()));

  INIT_RESOURCE_FATAL(_creditsTexture[0].h,
    RESOURCE_MANAGER.LoadTexture("gfx/credits_1.png", false, &_creditsTexture[0].info));
  INIT_RESOURCE_FATAL(_creditsTexture[1].h,
    RESOURCE_MANAGER.LoadTexture("gfx/credits_2.png", false, &_creditsTexture[1].info));

  INIT(_cbText.Create());
  INIT(_cbComposite.Create());
  INIT(_cbParticle.Create());
  
  ResetParticleSpline();

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
void Credits::ResetParticleSpline()
{
  int num = g_Blackboard->GetIntVar("credits.numParticles");
  vector<int> tmp(num);
  for (int i = 0; i < num; ++i)
    tmp[i] = i;

  InitParticleSpline(tmp);

  for (int i = 0; i < 1000; ++i)
  {
    UpdateParticleSpline(0.05f);
  }

}

//------------------------------------------------------------------------------
void Credits::UpdateParticles(const FixedUpdateState& state)
{
  if (_clothParticles.empty())
    return;

  g_stopWatch.Start();

  size_t numParticles = _clothParticles.size();
  float dt = state.delta;
  float dt2 = dt * dt;

  const IoState& ioState = TANO.GetIoState();
  if (ioState.keysPressed['1'])
  {
    int s = 20;
    for (int i = 0; i <= s; ++i)
    {
      for (int j = 0; j <= s; ++j)
      {
        int xOfs = j - s/2;
        int yOfs = i - s/2;
        float dx = (float)xOfs;
        float dy = (float)yOfs;
        float r = Clamp(0.f, 1.f, s/2 - sqrtf(dx*dx+dy*dy));
        _clothParticles[(_clothDimY/2+yOfs) * _clothDimX + _clothDimX/2+xOfs].acc = r * vec3(0, 0, 50);
      }
    }
  }


  ClothParticle* p = &_clothParticles[0];
  for (size_t i = 0; i < numParticles; ++i)
  {
    // verlet integration
    vec3 tmp = p->pos;
    p->pos += (1.0f - _settings.damping) * (p->pos - p->lastPos) + dt2 * p->acc;
    p->lastPos = tmp;
    p->acc = vec3(0, 0, 0);
    ++p;
  }

  // apply the constraints
  p = &_clothParticles[0];
  Constraint* c = &_constraints[0];
  for (int i = 0; i < 5; ++i)
  {
    int num = (int)(_constraints.size() / 4);
    for (int j = 0; j < num; ++j)
    {
      const Constraint& c0 = c[j * 4 + 0];
      const Constraint& c1 = c[j * 4 + 1];
      const Constraint& c2 = c[j * 4 + 2];
      const Constraint& c3 = c[j * 4 + 3];

      vec3 v0 = (c0.p1->pos - c0.p0->pos);
      vec3 v1 = (c1.p1->pos - c1.p0->pos);
      vec3 v2 = (c2.p1->pos - c2.p0->pos);
      vec3 v3 = (c3.p1->pos - c3.p0->pos);

      float dist0 = max(0.001f, Length(v0));
      float dist1 = max(0.001f, Length(v1));
      float dist2 = max(0.001f, Length(v2));
      float dist3 = max(0.001f, Length(v3));

      float s0 = 1 - c0.restLength / dist0;
      float s1 = 1 - c1.restLength / dist1;
      float s2 = 1 - c2.restLength / dist2;
      float s3 = 1 - c3.restLength / dist3;

      vec3 dir0 = 0.5f * s0 * v0;
      vec3 dir1 = 0.5f * s1 * v1;
      vec3 dir2 = 0.5f * s2 * v2;
      vec3 dir3 = 0.5f * s3 * v3;

      c0.p0->pos += dir0;
      c0.p1->pos -= dir0;
      c1.p0->pos += dir1;
      c1.p1->pos -= dir1;
      c2.p0->pos += dir2;
      c2.p1->pos -= dir2;
      c3.p0->pos += dir3;
      c3.p1->pos -= dir3;
    }
  }

  // top row is fixed
  float incX = CLOTH_SIZE / (_clothDimX - 1);
  vec3 top(-CLOTH_SIZE / 2.f, CLOTH_SIZE / 2.f, 0);
  vec3 bottom(-CLOTH_SIZE / 2.f, -CLOTH_SIZE / 2.f, 0);
  for (int i = 0; i < _clothDimX; ++i)
  {
    _clothParticles[i].pos = top;
    _clothParticles[(_clothDimY-1)*_clothDimX+i].pos = bottom;
    top.x += incX;
    bottom.x += incX;
  }

  _avgUpdate.AddSample(g_stopWatch.Stop());

  vec3* vtx = _ctx->MapWriteDiscard<vec3>(_clothGpuObjects._vb);
  memcpy(vtx, _clothParticles.data(), _numClothParticles * sizeof(ClothParticle));
  _ctx->Unmap(_clothGpuObjects._vb);
}

//------------------------------------------------------------------------------
bool Credits::InitParticles()
{
  int w, h;
  g_Graphics->GetBackBufferSize(&w, &h);

  int dimX = w / GRID_SIZE + 1;
  int dimY = h / GRID_SIZE + 1;
  
  int numParticles = dimX * dimY;
  _clothGpuObjects.CreateDynamicVb(numParticles * sizeof(ClothParticle), sizeof(ClothParticle));

  _clothDimX = dimX;
  _clothDimY = dimY;
  _numClothParticles = numParticles;

  // create the grid
  vector<u32> indices((dimX-1)*(dimY-1)*2*3);
  u32* idx = &indices[0];

  for (int i = 0; i < dimY - 1; ++i)
  {
    for (int j = 0; j < dimX - 1; ++j)
    {
      // 0--1
      // 2--3
      u32 i0 = (i + 0)*dimX + j + 0;
      u32 i1 = (i + 0)*dimX + j + 1;
      u32 i2 = (i + 1)*dimX + j + 0;
      u32 i3 = (i + 1)*dimX + j + 1;

      idx[0] = i0;
      idx[1] = i1;
      idx[2] = i2;

      idx[3] = i2;
      idx[4] = i1;
      idx[5] = i3;
      
      _numTris += 2;
      idx += 6;
    }
  }
  _clothGpuObjects.CreateIndexBuffer((u32)indices.size() * sizeof(u32), DXGI_FORMAT_R32_UINT, indices.data());

  _clothParticles.resize(numParticles);

  ResetParticles();

  // Check if we've saved the grouped constraints
  vector<char> buf;
  char filename[MAX_PATH];
  sprintf(filename, "data/cloth_constraints_%d.dat", GRID_SIZE);
  if (RESOURCE_MANAGER.LoadFile(filename, &buf))
  {
    int num = *(int*)&buf[0];
    _constraints.resize(num);
    vector<u16> data(num*2);

    memcpy(data.data(), (u16*)&buf[4], num * 2 * sizeof(u16));

    for (int i = 0; i < num; ++i)
    {
      _constraints[i].p0 = &_clothParticles[data[i * 2 + 0]];
      _constraints[i].p1 = &_clothParticles[data[i * 2 + 1]];
      _constraints[i].restLength = Distance(_constraints[i].p0->pos, _constraints[i].p1->pos);
    }
  }
  else
  {
#if WITH_UNPACKED_RESOUCES
    GroupConstraints();
    FILE* f = RESOURCE_MANAGER.OpenWriteFile(filename);

    // note, for the constraints, just save a u16 particle index
    vector<u16> data;
    data.reserve(_constraints.size() * 2);
    for (const Constraint& c : _constraints)
    {
      data.push_back((u16)c.idx0);
      data.push_back((u16)c.idx1);
    }
    RESOURCE_MANAGER.WriteFile(f, (int)_constraints.size());
    RESOURCE_MANAGER.WriteFile(f, (const char*)&data[0], (int)(data.size() * sizeof(u16)));
    RESOURCE_MANAGER.CloseFile(f);

    // apply the constaint fixup
    for (Constraint& c : _constraints)
    {
      c.p0 = &_clothParticles[c.idx0];
      c.p1 = &_clothParticles[c.idx1];
    }
#endif
  }


  return true;
}

//------------------------------------------------------------------------------
void Credits::GroupConstraints()
{
  int dimX = _clothDimX;
  int dimY = _clothDimY;

  // create cloth constraints
  // each particle is connected horiz, vert and diag (both 1 and 2 steps away)
  for (int i = 0; i < dimY; ++i)
  {
    for (int j = 0; j < dimX; ++j)
    {
      u32 idx0 = i*dimX + j;
      vec3* p0 = &_clothParticles[idx0].pos;

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
          vec3* p1 = &_clothParticles[idx1].pos;

          _constraints.push_back(Constraint(idx0, idx1, Distance(*p0, *p1)));
        }
      }
    }
  }

#if 1
  // make num constraints a multiple of 4
  for (int i = 0; i < (_constraints.size() & 3); ++i)
  {
    _constraints.push_back(Constraint(0, 0, 0));
  }

  // reorder the constaints so no group of 4 refers to the same particles
  // this allows us to process the whole block at once
  int numConstraints = (int)_constraints.size();
  int numChunks = (numConstraints / 4);
  vector<u32> used(numConstraints);
  vector<int> order(numConstraints);
  memset(used.data(), 0xff, numConstraints * sizeof(u32));

  vector<int> unmatched;
  for (int i = 0; i < numChunks; ++i)
  {
    u32 curChunk[8] = { -1, -1, -1, -1, -1, -1, -1, -1 };
    for (int j = 0; j < 4; ++j)
    {
      // find first unused constraint whose particles aren't used in the chunk
      int constraintIdx = -1;
      int ofs = 0;
      for (int k = 0; k < numConstraints; ++k)
      {
        int idx = (k + ofs) % numConstraints;
        if (used[idx] != 0xffffffff)
          continue;

        bool found = true;
        for (int cc = 0; cc < j * 2; ++cc)
        {
          if (_constraints[idx].idx0 == curChunk[cc] || _constraints[idx].idx1 == curChunk[cc])
          {
            found = false;
            break;
          }
        }

        if (found)
        {
          constraintIdx = idx;
          break;
        }
      }

      if (constraintIdx == -1)
      {
        // we've found constraints that can't be done in parallell, so store them
        for (int k = 0; k < numConstraints; ++k)
        {
          if (used[k] == 0xffffffff || used[k] == i)
          {
            // unassigned constraint
            unmatched.push_back(k);
            used[k] = ~0xffffffff;
          }
        }
        goto DONE;
      }

      // store which chunk the constraint was used in
      used[constraintIdx] = i;
      curChunk[j * 2 + 0] = _constraints[constraintIdx].idx0;
      curChunk[j * 2 + 1] = _constraints[constraintIdx].idx1;

      // found a constraint to use
      order[i * 4 + j] = constraintIdx;
    }
  }
DONE:

  // reorder the constraints
  vector<Constraint> reorderedConstraints;
  reorderedConstraints.reserve(numConstraints);

  for (int i = 0; i < numConstraints; ++i)
  {
    if (order[i] == 0xffffffff)
      continue;
    reorderedConstraints.push_back(_constraints[order[i]]);
  }

  _constraints.swap(reorderedConstraints);
#endif
}

//------------------------------------------------------------------------------
void Credits::ResetParticles()
{
  // create grid around -1..+1
  float incX = CLOTH_SIZE / (_clothDimX - 1);
  float incY = CLOTH_SIZE / (_clothDimY - 1);

  vec3 org(-CLOTH_SIZE / 2.f, CLOTH_SIZE / 2.f, 0);
  vec3 cur = org;
  ClothParticle* p = &_clothParticles[0];

  for (int i = 0; i < _clothDimY; ++i)
  {
    cur.x = org.x;
    for (int j = 0; j < _clothDimX; ++j)
    {
      p->pos = cur;
      p->lastPos = cur;
      p->acc = vec3(0, 0, 0);
      cur.x += incX;
      ++p;
    }
    cur.y -= incY;
  }
}

//------------------------------------------------------------------------------
void Credits::InitParticleSpline(const vector<int>& indices)
{
  g_Blackboard->SetNamespace("credits");

  _numParticles = g_Blackboard->GetIntVar("numParticles");

  // if # particles have changes, just redo everything..
  if (_numParticles != (int)_particles.size())
  {
    _particles.resize(_numParticles);
    _particleState.resize(_numParticles);
  }

  float speed = g_Blackboard->GetFloatVar("particleSpeed");
  float speedVar = g_Blackboard->GetFloatVar("particleSpeedVar");

  float angleSpeed = g_Blackboard->GetFloatVar("particleAngleSpeed");
  float angleSpeedVar = g_Blackboard->GetFloatVar("particleAngleSpeedVar");

  float fadeSpeed = g_Blackboard->GetFloatVar("particleFadeSpeed");
  float fadeSpeedVar = g_Blackboard->GetFloatVar("particleFadeSpeedVar");

  float angleVar = g_Blackboard->GetFloatVar("angleVar");

  float height = g_Blackboard->GetFloatVar("particleHeight");
  float heightVar = g_Blackboard->GetFloatVar("particleHeightVar");

  float width = g_Blackboard->GetFloatVar("waveWidth");

  for (int i : indices)
  {
    float s = RANDOM_150.Next(angleSpeed, angleSpeedVar);
    if (s == 0.f)
      s += 0.001f;

    _particleState[i] = ParticleState{
      RANDOM_150.Next(speed, speedVar),
      -width,
      RANDOM_150.Next(height, heightVar),
      RANDOM_150.Next(DirectX::XM_2PI, angleVar),
      XM_2PI / s,
      0,
      RANDOM_150.Next(fadeSpeed, fadeSpeedVar)};
  }

  g_Blackboard->ClearNamespace();
}

//------------------------------------------------------------------------------
void Credits::UpdateParticleSpline(float dt)
{
  if (g_Blackboard->IsDirtyTrigger(this))
  {
    ResetParticleSpline();
  }

  g_Blackboard->SetNamespace("credits");
  float width = g_Blackboard->GetFloatVar("waveWidth");

  vector<int> deadParticles;

  for (int i = 0; i < _numParticles; ++i)
  {
    ParticleState& s = _particleState[i];
    s.pos += dt * s.speed;
    s.angle += dt * s.angleInc;
    s.fade += dt * s.fadeInc;

    _particles[i] = vec4{s.pos, s.height * sinf(s.angle), 0, cosf(s.fade)};

    if (s.pos > width)
      deadParticles.push_back(i);
  }

  if (!deadParticles.empty())
  {
    InitParticleSpline(deadParticles);
  }

  g_Blackboard->ClearNamespace();
}

//------------------------------------------------------------------------------
bool Credits::Update(const UpdateState& state)
{
  _cbComposite.ps0.time =
      vec4(state.localTime.TotalSecondsAsFloat(), state.globalTime.TotalSecondsAsFloat(), 0, 0);

  UpdateCameraMatrix(state);
  UpdateParticleSpline(state.delta.TotalSecondsAsFloat());
  return true;
}

//------------------------------------------------------------------------------
bool Credits::FixedUpdate(const FixedUpdateState& state)
{
  //UpdateParticles(state);
  return true;
}

//------------------------------------------------------------------------------
void Credits::UpdateCameraMatrix(const UpdateState& state)
{

  Matrix view, proj, viewProj;
  proj = _freeflyCamera._proj;
  view = _freeflyCamera._view;
  viewProj = view * proj;
  _cbParticle.gs0.world = Matrix::Identity();
  _cbParticle.gs0.viewProj = viewProj.Transpose();

}

//------------------------------------------------------------------------------
bool Credits::Render()
{
  rmt_ScopedCPUSample(Cloth_Render);

  float s = 0.01f;
  static Color black(s, s, s, 0);

  FullscreenEffect* fullscreen = g_Graphics->GetFullscreenEffect();

  ScopedRenderTarget rtColor(DXGI_FORMAT_R11G11B10_FLOAT);
  _ctx->SetRenderTarget(rtColor, g_Graphics->GetDepthStencil(), &black);

  {
    // particles
    ObjectHandle h = _particleBundle.objects._vb;
    vec4* vtx = _ctx->MapWriteDiscard<vec4>(h);
    memcpy(vtx, _particles.data(), (int)_particles.size() * sizeof(vec4));
    _ctx->Unmap(h);

    // Render particles
    _cbParticle.Set(_ctx, 0);
    _ctx->SetBundleWithSamplers(_particleBundle, PixelShader);
    _ctx->SetShaderResource(_particleTexture);
    _ctx->Draw((int)_particles.size(), 0);
  }

  ScopedRenderTarget rtText(DXGI_FORMAT_R11G11B10_FLOAT);
  _ctx->SetRenderTarget(rtText, g_Graphics->GetDepthStencil(), &black);
  {
    // text
    float right = g_Blackboard->GetFloatVar("credits.textRight");
    float left = g_Blackboard->GetFloatVar("credits.textLeft");
    float s = g_Blackboard->GetFloatVar("credits.textSize");
    float y0 = g_Blackboard->GetFloatVar("credits.textPos0");
    float y1 = g_Blackboard->GetFloatVar("credits.textPos1");

    float ar0 = _creditsTexture[0].info.Width / (float)_creditsTexture[0].info.Height;
    float ar1 = _creditsTexture[1].info.Width / (float)_creditsTexture[1].info.Height;

    float bb0 = 1;

    fullscreen->RenderTexture(
      _creditsTexture[0].h, vec2{ left, y0 - s }, vec2{ left + ar0 * s, y0 }, bb0);

    fullscreen->RenderTexture(
      _creditsTexture[1].h, vec2{ left, y1 - s }, vec2{ left + ar1 * s, y1 }, bb0);
  }

  ScopedRenderTarget rtBlur(rtColor._desc, BufferFlag::CreateSrv | BufferFlag::CreateUav);
  {
    // blur
    _ctx->UnsetRenderTargets(0, 1);
    fullscreen->BlurVertCustom(rtColor, rtBlur, rtBlur._desc, _csBlur, _settings.blur_amount, 1);
  }

  {
    // composite
    _cbComposite.ps0.tonemap = vec4(_settings.tonemap.exposure, _settings.tonemap.min_white, 0, 0);
    _cbComposite.Set(_ctx, 0);

    ObjectHandle inputs[] = { rtColor, rtBlur, rtText };
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
bool Credits::InitAnimatedParameters()
{
  return true;
}

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Credits::RenderParameterSet()
{
  double avg = _avgUpdate.GetAverage();
  ImGui::Text("Avg update: %lfs (%.1lf fps)", avg, 1 / avg );
  ImGui::SliderFloat("Damping", &_settings.damping, 0, 1);
  ImGui::SliderFloat3("Gravity", &_settings.gravity.x, -5, +5);

  ImGui::Separator();
  ImGui::SliderFloat("Exposure", &_settings.tonemap.exposure, 0.1f, 2.0f);
  ImGui::SliderFloat("Min White", &_settings.tonemap.min_white, 0.1f, 2.0f);

  ImGui::Separator();
  ImGui::SliderFloat("Blur Amount", &_settings.blur_amount, 1, 500);

  if (ImGui::Button("Reset"))
    Reset();
}
#endif

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Credits::SaveParameterSet(bool inc)
{
  _freeflyCamera.ToProtocol(&_settings.camera);
  SaveSettings(_settings, inc);
}
#endif

//------------------------------------------------------------------------------
void Credits::Reset()
{
  _freeflyCamera._pos = vec3(0.f, 0.f, -10.f);
  _freeflyCamera._pitch = _freeflyCamera._yaw = _freeflyCamera._roll = 0.f;
}

//------------------------------------------------------------------------------
bool Credits::Close()
{
  return true;
}

//------------------------------------------------------------------------------
BaseEffect* Credits::Create(const char* name, const char* config, u32 id)
{
  return new Credits(name, config, id);
}

//------------------------------------------------------------------------------
const char* Credits::Name()
{
  return "credits";
}

//------------------------------------------------------------------------------
void Credits::Register()
{
  g_DemoEngine->RegisterFactory(Name(), Credits::Create);
}
