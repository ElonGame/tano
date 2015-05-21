#include "cloth.hpp"
#include "../tano.hpp"
#include "../graphics.hpp"
#include "../graphics_context.hpp"
#include "../demo_engine.hpp"
#include "../resource_manager.hpp"
#include "../init_sequence.hpp"
#include "../generated/demo.parse.hpp"
#include "../generated/input_buffer.hpp"
#include "../generated/output_buffer.hpp"
#include "../mesh_utils.hpp"
#include "../post_process.hpp"

/*
  update timing:

  doozblade
  base: 6.9 ms
  conversion to V3: 5.8 ms

  mothership:
  V3: 2.5 ms
  back to p0/p1 constraints: 2.3 ms
*/

using namespace tano;
using namespace bristol;

namespace
{
  int GRID_SIZE = 20;
  float CLOTH_SIZE = 10;
}

struct StopWatch
{
  StopWatch()
  {
    QueryPerformanceFrequency(&_frequency);
  }

  void Start()
  {
    QueryPerformanceCounter(&_start);
  }

  double Stop()
  {
    LARGE_INTEGER tmp;
    QueryPerformanceCounter(&tmp);

    return (double)(tmp.QuadPart - _start.QuadPart) / _frequency.QuadPart;
  }

  LARGE_INTEGER _frequency;
  LARGE_INTEGER _start;
};

StopWatch g_stopWatch;

//------------------------------------------------------------------------------
Cloth::Cloth(const string &name, u32 id)
  : Effect(name, id)
  , _avgUpdate(100)
{
#if WITH_IMGUI
  PROPERTIES.Register(Name(),
    bind(&Cloth::RenderParameterSet, this),
    bind(&Cloth::SaveParameterSet, this));

  PROPERTIES.SetActive(Name());
#endif
}

//------------------------------------------------------------------------------
Cloth::~Cloth()
{
  SAFE_ADELETE(_distTable);
}

//------------------------------------------------------------------------------
bool Cloth::Init(const char* configFile)
{
  BEGIN_INIT_SEQUENCE();

  _configName = configFile;
  vector<char> buf;
  INIT_FATAL(RESOURCE_MANAGER.LoadFile(configFile, &buf));

  INIT(ParseClothSettings(InputBuffer(buf), &_settings));
  _camera._pitch = _settings.camera.pitch;
  _camera._yaw = _settings.camera.yaw;
  _camera._roll = _settings.camera.roll;
  _camera._pos = _settings.camera.pos;

  CD3D11_RASTERIZER_DESC rasterDesc = CD3D11_RASTERIZER_DESC(CD3D11_DEFAULT());
  rasterDesc.CullMode = D3D11_CULL_NONE;
  rasterDesc.FillMode = D3D11_FILL_WIREFRAME;
  INIT(_clothState.Create(nullptr, nullptr, &rasterDesc));

  INIT(_cbPerFrame.Create());
  int w, h;
  GRAPHICS.GetBackBufferSize(&w, &h);
  _cbPerFrame.dim.x = (float)w;
  _cbPerFrame.dim.y = (float)h;

  INIT(_clothGpuObjects.LoadShadersFromFile("shaders/out/basic", "VsPos", nullptr, "PsPos", VF_POS));

  INIT(InitParticles());

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
void Cloth::UpdateParticles(const UpdateState& state)
{
  g_stopWatch.Start();

  size_t numParticles = _particles.size();
  float dt = 1.f / state.frequency;
  float dt2 = dt * dt;

  for (size_t i = 0; i < numParticles; ++i)
  {
    //_particles[i].acc = Vector3(randf(s, s), randf(s, s), randf(s, s));
    _particles[i].acc = _settings.gravity;
  }

  Particle* p = &_particles[0];
  //V3* pos = &_particlePos[0];
  //V3* lastPos = &_particleLastPos[0];
  //V3* acc = &_particleAcc[0];
  for (size_t i = 0; i < numParticles; ++i)
  {
    // verlet integration
    V3 tmp = p->pos;
    p->pos = p->pos + (1.0f - _settings.damping) * (p->pos - p->lastPos) + dt2 * p->acc;
    p->lastPos = tmp;
    ++p;

//    _particles[i].acc = Vector3(0, 0, 0);
//    *acc = V3(0,0,0);
    _particles[i].acc = Vector3(0, 0, 0);

//     ++pos;
//     ++lastPos;
//     ++acc;
  }

  // apply the constraints
#if 1
  p = &_particles[0];
  Constraint* c = &_constraints[0];
  for (int i = 0; i < 2; ++i)
  {
    int num = (int)(_constraints.size() / 4);
    for (int j = 0; j < num; ++j)
    {
#if 0
      const Constraint& c0 = _constraints[j * 4 + 0];
      const Constraint& c1 = _constraints[j * 4 + 1];
      const Constraint& c2 = _constraints[j * 4 + 2];
      const Constraint& c3 = _constraints[j * 4 + 3];

      V3 v0 = (c0.p1->pos - c0.p0->pos);
      float dist0 = Length(v0);
      float s0 = 1 - c0.restLength / dist0;
      V3 dir0 = 0.5f * s0 * v0;
      c0.p0->pos = c0.p0->pos + dir0;
      c0.p1->pos = c0.p1->pos - dir0;

      V3 v1 = (c1.p1->pos - c1.p0->pos);
      float dist1 = Length(v1);
      float s1 = 1 - c1.restLength / dist1;
      V3 dir1 = 0.5f * s1 * v1;
      c1.p0->pos = c1.p0->pos + dir1;
      c1.p1->pos = c1.p1->pos - dir1;

      V3 v2 = (c2.p1->pos - c2.p0->pos);
      float dist2 = Length(v2);
      float s2 = 1 - c2.restLength / dist2;
      V3 dir2 = 0.5f * s2 * v2;
      c2.p0->pos = c2.p0->pos + dir2;
      c2.p1->pos = c2.p1->pos - dir2;

      V3 v3 = (c3.p1->pos - c3.p0->pos);
      float dist3 = Length(v3);
      float s3 = 1 - c3.restLength / dist3;
      V3 dir3 = 0.5f * s3 * v3;
      c3.p0->pos = c3.p0->pos + dir3;
      c3.p1->pos = c3.p1->pos - dir3;
#else
      const Constraint& c0 = c[j * 4 + 0];
      const Constraint& c1 = c[j * 4 + 1];
      const Constraint& c2 = c[j * 4 + 2];
      const Constraint& c3 = c[j * 4 + 3];

      V3 v0 = (c0.p1->pos - c0.p0->pos);
      V3 v1 = (c1.p1->pos - c1.p0->pos);
      V3 v2 = (c2.p1->pos - c2.p0->pos);
      V3 v3 = (c3.p1->pos - c3.p0->pos);

      float dist0 = Length(v0);
      float dist1 = Length(v1);
      float dist2 = Length(v2);
      float dist3 = Length(v3);

      float eps = 0.01f;
      float s0 = dist0 < eps ? 1 : 1 - c0.restLength / dist0;
      float s1 = dist1 < eps ? 1 : 1 - c1.restLength / dist1;
      float s2 = dist2 < eps ? 1 : 1 - c2.restLength / dist2;
      float s3 = dist3 < eps ? 1 : 1 - c3.restLength / dist3;

      V3 dir0 = 0.5f * s0 * v0;
      V3 dir1 = 0.5f * s1 * v1;
      V3 dir2 = 0.5f * s2 * v2;
      V3 dir3 = 0.5f * s3 * v3;

      c0.p0->pos = c0.p0->pos + dir0;
      c0.p1->pos = c0.p1->pos - dir0;
      c1.p0->pos = c1.p0->pos + dir1;
      c1.p1->pos = c1.p1->pos - dir1;
      c2.p0->pos = c2.p0->pos + dir2;
      c2.p1->pos = c2.p1->pos - dir2;
      c3.p0->pos = c3.p0->pos + dir3;
      c3.p1->pos = c3.p1->pos - dir3;
#endif
    }

  }
#else
  for (int i = 0; i < 2; ++i)
  {
    for (auto& kv : _constraintsByParticle)
    {
      Particle* p0 = kv.first;

      for (const ConstraintByParticle& c: kv.second)
      {
        Particle* p1 = c.p1;
        float dist = Distance(p0->pos, p1->pos);
        float s = 1 - c.restLength / dist;
        V3 dir = s * (p1->pos - p0->pos);
        p0->pos = p0->pos + 0.5f * dir;
        p1->pos = p1->pos - 0.5f * dir;
      }
    }
  }
#endif

  // top row is fixed
  float incX = CLOTH_SIZE / (_clothDimX - 1);
  Vector3 cur(-CLOTH_SIZE / 2.f, CLOTH_SIZE / 2.f, 0);
  for (u32 i = 0; i < _clothDimX; ++i)
  {
    _particles[i].pos = cur;
    //_particlePos[i] = cur;
    cur.x += incX;
    //++p;
  }

  _avgUpdate.AddSample(g_stopWatch.Stop());

  V3* vtx = _ctx->MapWriteDiscard<V3>(_clothGpuObjects._vb);
  memcpy(vtx, _particles.data(), _numParticles * sizeof(Particle));
  _ctx->Unmap(_clothGpuObjects._vb);
}

//------------------------------------------------------------------------------
bool Cloth::InitParticles()
{
  int w, h;
  GRAPHICS.GetBackBufferSize(&w, &h);

  int dimX = w / GRID_SIZE + 1;
  int dimY = h / GRID_SIZE + 1;
  
  int numParticles = dimX * dimY;
  _clothGpuObjects.CreateDynamicVb(numParticles * sizeof(Particle), sizeof(Particle));

  _clothDimX = dimX;
  _clothDimY = dimY;
  _numParticles = numParticles;

  _distTable = new float[_clothDimX*_clothDimY];

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

  _particles.resize(numParticles);
  //_particlePos.resize(numParticles);
  //_particleAcc.resize(numParticles);
  //_particleLastPos.resize(numParticles);

  ResetParticles();

  map<Particle*, vector<Particle*>> constraintsByParticle;

  // create cloth constraints
  // each particle is connected horiz, vert and diag (both 1 and 2 steps away)
  for (int i = 0; i < dimY; ++i)
  {
    for (int j = 0; j < dimX; ++j)
    {
      u32 idx0 = i*dimX + j;
      //V3* p0 = &_particlePos[idx0];
      V3* p0 = &_particles[idx0].pos;

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
          int xx = j + s * ofs[idx*2+0];
          int yy = i + s * ofs[idx*2+1];
          if (xx < 0 || xx >= dimX || yy < 0 || yy >= dimY)
            continue;

          u32 idx1 = yy*dimX + xx;
          //V3* p1 = &_particlePos[idx1];
          V3* p1 = &_particles[idx1].pos;

          //constraintsByParticle[min(p0, p1)].push_back(max(p1, p0));
          _constraints.push_back({ &_particles[idx0], &_particles[idx1], Distance(*p0, *p1) });
        }
      }
    }
  }

  // make num constraints a multiple of 4
  for (int i = 0; i < (_constraints.size() & 3); ++i)
  {
    _constraints.push_back({ &_particles[0], &_particles[0], 0 });
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
    Particle* curChunk[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
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
        for (int cc = 0; cc < j*2; ++cc)
        {
          if (_constraints[idx].p0 == curChunk[cc] || _constraints[idx].p1 == curChunk[cc])
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
      curChunk[j * 2 + 0] = _constraints[constraintIdx].p0;
      curChunk[j * 2 + 1] = _constraints[constraintIdx].p1;

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

  for (auto& kv : constraintsByParticle)
  {
    Particle* p0 = kv.first;
    for (Particle* p1: kv.second)
    {
      _constraintsByParticle[p0].push_back({p1, Distance(p0->pos, p1->pos) });
    }
  }

  return true;
}

//------------------------------------------------------------------------------
void Cloth::UpdateDistTable()
{
}

//------------------------------------------------------------------------------
void Cloth::ResetParticles()
{
  // create grid around -1..+1
  float incX = CLOTH_SIZE / (_clothDimX - 1);
  float incY = CLOTH_SIZE / (_clothDimY - 1);

  Vector3 org(-CLOTH_SIZE / 2.f, CLOTH_SIZE / 2.f, 0);
  Vector3 cur = org;
  Particle* p = &_particles[0];

//   V3* pos = &_particlePos[0];
//   V3* lastPos = &_particleLastPos[0];
//   V3* acc = &_particleAcc[0];

  for (u32 i = 0; i < _clothDimY; ++i)
  {
    cur.x = org.x;
    for (u32 j = 0; j < _clothDimX; ++j)
    {
//       *pos = cur;
//       *lastPos = cur;
//       *acc = V3(0, 0, 0);
      p->pos = cur;
      p->lastPos = cur;
      p->acc = V3(0, 0, 0);
      cur.x += incX;
      ++p;

//       ++pos;
//       ++lastPos;
//       ++acc;
    }
    cur.y -= incY;
  }
}

//------------------------------------------------------------------------------
bool Cloth::Update(const UpdateState& state)
{
  UpdateCameraMatrix();

  UpdateParticles(state);
  return true;
}

//------------------------------------------------------------------------------
void Cloth::UpdateCameraMatrix()
{
  _camera.Update();

  int w, h;
  GRAPHICS.GetBackBufferSize(&w, &h);
  float aspectRatio = (float)w / h;

  Matrix proj = _camera._proj;
  Matrix view = _camera._view;

  Matrix viewProj = view * proj;

  _cbPerFrame.world = Matrix::Identity();
  _cbPerFrame.view = view.Transpose();
  _cbPerFrame.proj = proj.Transpose();
  _cbPerFrame.viewProj = viewProj.Transpose();
  _cbPerFrame.cameraPos = _camera._pos;
//  _cbPerFrame.cameraLookAt = target;
//  _cbPerFrame.cameraUp = up;
}

//------------------------------------------------------------------------------
bool Cloth::Render()
{
  rmt_ScopedCPUSample(Cloth_Render);

  static Color black(.1f, .1f, .1f, 0);

  _ctx->SetSwapChain(GRAPHICS.DefaultSwapChain(), black);
  _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::PixelShader, 0);

  _ctx->SetGpuState(_clothState);

  _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::VertexShader, 0);
  _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::PixelShader, 0);

  _ctx->SetGpuObjects(_clothGpuObjects);
  _ctx->DrawIndexed(_numTris*3, 0, 0);

  return true;
}

//------------------------------------------------------------------------------
bool Cloth::InitAnimatedParameters()
{
  return true;
}

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Cloth::RenderParameterSet()
{
  double avg = _avgUpdate.GetAverage();
  ImGui::Text("Avg update: %lfs (%.1lf fps)", avg, 1 / avg );
  ImGui::SliderFloat("Damping", &_settings.damping, 0, 1);
  ImGui::SliderFloat3("Gravity", &_settings.gravity.x, -5, +5);

  if (ImGui::Button("Reset"))
    Reset();
}
#endif

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Cloth::SaveParameterSet()
{
  OutputBuffer buf;
  _settings.camera.pos = _camera._pos;
  _settings.camera.yaw = _camera._yaw;
  _settings.camera.pitch = _camera._pitch;
  _settings.camera.roll = _camera._roll;
  Serialize(buf, _settings);
  if (FILE* f = fopen(_configName.c_str(), "wt"))
  {
    fwrite(buf._buf.data(), 1, buf._ofs, f);
    fclose(f);
  }
}
#endif

//------------------------------------------------------------------------------
void Cloth::Reset()
{
  _camera._pos = Vector3(0.f, 0.f, -10.f);
  _camera._pitch = _camera._yaw = _camera._roll = 0.f;
}

//------------------------------------------------------------------------------
bool Cloth::Close()
{
  return true;
}

//------------------------------------------------------------------------------
Effect* Cloth::Create(const char* name, u32 id)
{
  return new Cloth(name, id);
}

//------------------------------------------------------------------------------
const char* Cloth::Name()
{
  return "cloth";
}

//------------------------------------------------------------------------------
void Cloth::Register()
{
  DEMO_ENGINE.RegisterFactory(Name(), Cloth::Create);
}
