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
#include "../stop_watch.hpp"

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

namespace
{
  int GRID_SIZE = 20;
  float CLOTH_SIZE = 10;
}

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

  INIT(_clothGpuObjects.LoadVertexShader("shaders/out/basic", "VsPos", VF_POS));
  INIT(_clothGpuObjects.LoadPixelShader("shaders/out/basic", "PsPos"));

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
        _particles[(_clothDimY/2+yOfs) * _clothDimX + _clothDimX/2+xOfs].acc = r * V3(0, 0, 50);
      }
    }

    int a = 10;
  }
  else
  {
    //for (size_t i = 0; i < numParticles; ++i)
    //{
    //  _particles[i].acc = V3(_settings.gravity);
    //}
  }


  Particle* p = &_particles[0];
  for (size_t i = 0; i < numParticles; ++i)
  {
    // verlet integration
    V3 tmp = p->pos;
    p->pos += (1.0f - _settings.damping) * (p->pos - p->lastPos) + dt2 * p->acc;
    p->lastPos = tmp;
    p->acc = V3(0, 0, 0);
    ++p;
  }

  // apply the constraints
  p = &_particles[0];
  Constraint* c = &_constraints[0];
  for (int i = 0; i < 2; ++i)
  {
    int num = (int)(_constraints.size() / 4);
    for (int j = 0; j < num; ++j)
    {
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

      float s0 = 1 - c0.restLength / dist0;
      float s1 = 1 - c1.restLength / dist1;
      float s2 = 1 - c2.restLength / dist2;
      float s3 = 1 - c3.restLength / dist3;

      V3 dir0 = 0.5f * s0 * v0;
      V3 dir1 = 0.5f * s1 * v1;
      V3 dir2 = 0.5f * s2 * v2;
      V3 dir3 = 0.5f * s3 * v3;

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
  V3 top(-CLOTH_SIZE / 2.f, CLOTH_SIZE / 2.f, 0);
  V3 bottom(-CLOTH_SIZE / 2.f, -CLOTH_SIZE / 2.f, 0);
  for (int i = 0; i < _clothDimX; ++i)
  {
    _particles[i].pos = top;
    _particles[(_clothDimY-1)*_clothDimX+i].pos = bottom;
    top.x += incX;
    bottom.x += incX;
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
      _constraints[i].p0 = &_particles[data[i * 2 + 0]];
      _constraints[i].p1 = &_particles[data[i * 2 + 1]];
      _constraints[i].restLength = Distance(_constraints[i].p0->pos, _constraints[i].p1->pos);
    }
  }
  else
  {
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
      c.p0 = &_particles[c.idx0];
      c.p1 = &_particles[c.idx1];
    }
  }


  return true;
}

//------------------------------------------------------------------------------
void Cloth::GroupConstraints()
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
          int xx = j + s * ofs[idx * 2 + 0];
          int yy = i + s * ofs[idx * 2 + 1];
          if (xx < 0 || xx >= dimX || yy < 0 || yy >= dimY)
            continue;

          u32 idx1 = yy*dimX + xx;
          V3* p1 = &_particles[idx1].pos;

          _constraints.push_back(Constraint(idx0, idx1, Distance(*p0, *p1)));
        }
      }
    }
  }

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

}

//------------------------------------------------------------------------------
void Cloth::ResetParticles()
{
  // create grid around -1..+1
  float incX = CLOTH_SIZE / (_clothDimX - 1);
  float incY = CLOTH_SIZE / (_clothDimY - 1);

  V3 org(-CLOTH_SIZE / 2.f, CLOTH_SIZE / 2.f, 0);
  V3 cur = org;
  Particle* p = &_particles[0];

  for (int i = 0; i < _clothDimY; ++i)
  {
    cur.x = org.x;
    for (int j = 0; j < _clothDimX; ++j)
    {
      p->pos = cur;
      p->lastPos = cur;
      p->acc = V3(0, 0, 0);
      cur.x += incX;
      ++p;
    }
    cur.y -= incY;
  }
}

//------------------------------------------------------------------------------
bool Cloth::Update(const UpdateState& state)
{
  UpdateCameraMatrix(state);

  UpdateParticles(state);
  return true;
}

//------------------------------------------------------------------------------
void Cloth::UpdateCameraMatrix(const UpdateState& state)
{
  _camera.Update(state);

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

  u32 flags = ShaderType::VertexShader | ShaderType::PixelShader;
  _ctx->SetConstantBuffer(_cbPerFrame, flags, 0);

  _ctx->SetGpuState(_clothState);

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
