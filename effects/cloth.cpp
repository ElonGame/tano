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

using namespace tano;
using namespace bristol;

namespace
{
  int GRID_SIZE = 20;
  float CLOTH_SIZE = 10;
}

//------------------------------------------------------------------------------
Cloth::Cloth(const string &name, u32 id)
: Effect(name, id)
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

  INIT(_clothGpuObjects.LoadShadersFromFile("shaders/out/basic", "VsPos", nullptr, "PsPos", VF_POS));

  INIT(InitParticles());

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
void Cloth::UpdateParticles(const UpdateState& state)
{
  size_t numParticles = _particles.size();
  float dt = 1.f / state.frequency;
  float dt2 = dt * dt;

  for (size_t i = 0; i < numParticles; ++i)
  {
    //_particles[i].acc = Vector3(randf(s, s), randf(s, s), randf(s, s));
    _particles[i].acc = _settings.gravity;
  }

  Particle* p = &_particles[0];
  for (size_t i = 0; i < numParticles; ++i)
  {
    // verlet integration
    Vector3 tmp = p->pos;
    p->pos = p->pos + (1.0f - _settings.damping) * (p->pos - p->lastPos) + p->acc * dt2;
    p->lastPos = tmp;
    ++p;

    _particles[i].acc = Vector3(0, 0, 0);
  }

  // apply the constraints
  for (int i = 0; i < 2; ++i)
  {
    for (const Constraint& c : _constraints)
    {
      Particle* p0 = c.p0;
      Particle* p1 = c.p1;

      float dist = Vector3::Distance(p0->pos, p1->pos);
      Vector3 dir = ((dist - c.restLength) / dist) * (p1->pos - p0->pos);
      p0->pos += 0.5f * dir;
      p1->pos -= 0.5f * dir;
    }
  }

  // top row is fixed
  float incX = CLOTH_SIZE / (_clothDimX - 1);
  Vector3 cur(-CLOTH_SIZE / 2.f, CLOTH_SIZE / 2.f, 0);
  for (u32 i = 0; i < _clothDimX; ++i)
  {
    _particles[i].pos = cur;
    cur.x += incX;
    ++p;
  }

  Vector3* vtx = _ctx->MapWriteDiscard<Vector3>(_clothGpuObjects._vb);
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
  // create grid around -1..+1
/*
  float incX = CLOTH_SIZE / (dimX - 1);
  float incY = CLOTH_SIZE / (dimY - 1);

  Vector3 org(-CLOTH_SIZE/2.f, CLOTH_SIZE/2.f, 0);
  Vector3 cur = org;
  Particle* p = &_particles[0];
  for (int i = 0; i < dimY; ++i)
  {
    cur.x = org.x;
    for (int j = 0; j < dimX; ++j)
    {
      p->pos = cur;
      p->lastPos = cur;
      p->acc = Vector3(0,0,0);
      cur.x += incX;
      ++p;
    }
    cur.y -= incY;
  }
*/
  // create cloth constraints
  // each particle is connected to the left, diag, and down particles (both 1 and 2 steps away)
  for (int i = 0; i < dimY-2; ++i)
  {
    for (int j = 0; j < dimX-2; ++j)
    {
      Particle* p0 = &_particles[i*dimX + j];
      for (int k = 1; k <= 2; ++k)
      {
        Particle* p1 = &_particles[(i + 0)*dimX + j + k];
        Particle* p2 = &_particles[(i + k)*dimX + j + k];
        Particle* p3 = &_particles[(i + k)*dimX + j + 0];

        _constraints.push_back({ p0, p1, Vector3::Distance(p0->pos, p1->pos) });
        _constraints.push_back({ p0, p2, Vector3::Distance(p0->pos, p2->pos) });
        _constraints.push_back({ p0, p3, Vector3::Distance(p0->pos, p3->pos) });
      }
    }
  }

  return true;
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
  for (u32 i = 0; i < _clothDimY; ++i)
  {
    cur.x = org.x;
    for (u32 j = 0; j < _clothDimX; ++j)
    {
      p->pos = cur;
      p->lastPos = cur;
      p->acc = Vector3(0, 0, 0);
      cur.x += incX;
      ++p;
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
  ImGui::SliderFloat("Damping", &_settings.damping, 0, 1);
  ImGui::SliderFloat3("Gravity", &_settings.gravity.x, -5, 0);

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
