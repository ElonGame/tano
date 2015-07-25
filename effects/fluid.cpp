#include "fluid.hpp"
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
Fluid::Fluid(const string &name, const string& config, u32 id)
  : BaseEffect(name, config, id)
{
#if WITH_IMGUI
  PROPERTIES.Register(Name(),
    bind(&Fluid::RenderParameterSet, this),
    bind(&Fluid::SaveParameterSet, this));

  PROPERTIES.SetActive(Name());
#endif
}

//------------------------------------------------------------------------------
Fluid::~Fluid()
{
}

//------------------------------------------------------------------------------
bool Fluid::OnConfigChanged(const vector<char>& buf)
{
  return ParseFluidSettings(InputBuffer(buf), &_settings);
}

//------------------------------------------------------------------------------
bool Fluid::Init()
{
  BEGIN_INIT_SEQUENCE();

  _fluidTexture = GRAPHICS.CreateTexture(FluidSim::FLUID_SIZE, FluidSim::FLUID_SIZE, DXGI_FORMAT_R8G8B8A8_UNORM, nullptr);

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool Fluid::Update(const UpdateState& state)
{
  UpdateCameraMatrix(state);
  UpdateFluidTexture();
  _sim.Update(state);

  return true;
}

//------------------------------------------------------------------------------
bool Fluid::FixedUpdate(const FixedUpdateState& state)
{
  _camera.Update(state);
  return true;
}

//------------------------------------------------------------------------------
void Fluid::UpdateCameraMatrix(const UpdateState& state)
{
  Matrix view = _camera._view;
  Matrix proj = _camera._proj;

  Matrix viewProj = view * proj;
}

//------------------------------------------------------------------------------
bool Fluid::Render()
{
  rmt_ScopedCPUSample(Fluid_Render);

  static Color black(0, 0, 0, 0);

  _ctx->SetSwapChain(GRAPHICS.DefaultSwapChain(), black);

  RenderTargetDesc desc = GRAPHICS.GetBackBufferDesc();

  return true;
}

//------------------------------------------------------------------------------
Fluid::FluidSim::FluidSim()
{
  for (int i = 0; i < FLUID_SIZE_PADDED_SQ; ++i)
  {
    //dForce[i] = 0.f;
    //uForce[i] = 0.f;
    //vForce[i] = 0.f;

    density0[i] = 0.f;
    density1[i] = 0.f;

    u0[i] = 0.f;
    u1[i] = 0.f;

    v0[i] = 0.f;
    v1[i] = 0.f;
  }
}

//------------------------------------------------------------------------------
void Fluid::FluidSim::Update(const UpdateState& state)
{
  for (int i = 0; i < FLUID_SIZE_PADDED_SQ; ++i)
  {
    dOld[i] = 0.f;
    uOld[i] = 0.f;
    vOld[i] = 0.f;
  }

  int size = BLACKBOARD.GetIntVar("fluid.blobSize");
  float diffuseStrength = BLACKBOARD.GetFloatVar("fluid.diffuseStrength");
  float velocityStrength = BLACKBOARD.GetFloatVar("fluid.velocityStrength");

  static float angle = 0;
  angle += state.delta.TotalSecondsAsFloat();
  for (int i = 0; i < size; ++i)
  {
    for (int j = 0; j < size; ++j)
    {
      int x = FLUID_SIZE / 2 - size / 2 + j;
      int y = FLUID_SIZE / 2 - size / 2 + i;

      V2 aa((float)(i - size /2), (float)(j - size / 2));
      aa = Normalize(aa);
      V2 bb = V2(cosf(angle), sinf(angle));
      float d = Dot(aa, bb);

      dOld[IX(x, y)] += diffuseStrength; // / 2 + (diffuseStrength / 2) * sinf(2 * angle);
      uOld[IX(x, y)] += d * velocityStrength * aa.x;
      vOld[IX(x, y)] += d * velocityStrength * aa.y;
    }
  }

  float dt = BLACKBOARD.GetFloatVar("fluid.timeScale") * state.delta.TotalSecondsAsFloat();
  DensityStep(dt);
  VelocityStep(dt);
}

//------------------------------------------------------------------------------
void Fluid::FluidSim::AddForce(float dt, float* out, float* force)
{
  for (int i = 0; i < FLUID_SIZE_PADDED_SQ; ++i)
  {
    out[i] += dt * force[i];
  }
}

//------------------------------------------------------------------------------
void Fluid::FluidSim::Diffuse(int b, float dt, float diff, float* out, float* old)
{
  float a = dt * diff * FLUID_SIZE_SQ;

  // Implicit technique for calcuating D(n+1).
  // Instead of D(t+1) = D(t) + dt * D(t, stuff), do
  // D(t) = D(t+1) - dt * D(t+1, stuff) 

  float r = 1.0f / (1 + 4 * a);

  // Gauss-Seidel solver
  for (int k = 0; k < 20; ++k)
  {
    for (int y = 1; y <= FLUID_SIZE; ++y)
    {
      for (int x = 1; x <= FLUID_SIZE; ++x)
      {
        out[IX(x, y)] =
            r * (old[IX(x, y)] + a * (out[IX(x - 1, y)] + out[IX(x + 1, y)] + out[IX(x, y - 1)] + out[IX(x, y + 1)]));
      }
    }

    BoundaryConditions(b, out);
  }
}

//------------------------------------------------------------------------------
void Fluid::FluidSim::Advect(int b, float dt, float* out, float* old, float* u, float *v)
{
#if 0
  float dt = state.delta * FLUID_SIZE;
  float size = (float)FLUID_SIZE;

  float fy = 1;
  for (int y = 1; y <= FLUID_SIZE; ++y)
  {
    float fx = 1;
    for (int x = 1; x <=  FLUID_SIZE; ++x)
    {
      // for each element, travel backwards along the velocity vector, and bilerp
      // between the 4 cells we end up in
      V2 p = V2(fx, fy) - dt * V2(u[IX(x, y)], v[IX(x, y)]);
      p.x = Clamp(0.5f, size + 0.5f, p.x);
      p.y = Clamp(0.5f, size + 0.5f, p.y);

      int x0 = (int)p.x;
      int x1 = x0 + 1;
      int y0 = (int)p.y;
      int y1 = y0 + 1;

      float s0 = p.x - (float)x0;
      float s1 = 1 - s0;
      float t0 = p.y - (float)y0;
      float t1 = 1 - t0;

      dCur[IX(x, y)] = lerp(
        lerp(dOld[IX(x0, y0)], dOld[IX(x1, y0)], s0),
        lerp(dOld[IX(x0, y1)], dOld[IX(x1, y1)], s1),
        t0);

      fx += 1;
    }
    fy += 1;
  }
#endif
  int i, j, i0, j0, i1, j1;
  float x, y, s0, t0, s1, t1, dt0;
  dt0 = dt * FLUID_SIZE;
  for (j = 1; j <= FLUID_SIZE; j++)
  {
    for (i = 1; i <= FLUID_SIZE; i++)
    {
      x = i - dt0 * u[IX(i, j)];
      y = j - dt0 * v[IX(i, j)];
      if (x < 0.5)
        x = 0.5;
      if (x > FLUID_SIZE + 0.5)
        x = FLUID_SIZE + 0.5;
      i0 = (int)x;
      i1 = i0 + 1;
      if (y < 0.5)
        y = 0.5;
      if (y > FLUID_SIZE + 0.5)
        y = FLUID_SIZE + 0.5;
      j0 = (int)y;
      j1 = j0 + 1;
      s1 = x - i0;
      s0 = 1 - s1;
      t1 = y - j0;
      t0 = 1 - t1;
      out[IX(i, j)] = s0 * (t0 * old[IX(i0, j0)] + t1 * old[IX(i0, j1)]) +
                      s1 * (t0 * old[IX(i1, j0)] + t1 * old[IX(i1, j1)]);
    }
  }

  BoundaryConditions(b, out);
}

//------------------------------------------------------------------------------
void Fluid::FluidSim::DensityStep(float dt)
{
  float diff = BLACKBOARD.GetFloatVar("fluid.diff");
  AddForce(dt, dCur, dOld);
  swap(dCur, dOld);
  Diffuse(0, dt, diff, dCur, dOld);
  swap(dCur, dOld);
  Advect(0, dt, dCur, dOld, uCur, vCur);
}

//------------------------------------------------------------------------------
void Fluid::FluidSim::VelocityStep(float dt)
{
  float visc = BLACKBOARD.GetFloatVar("fluid.visc");
  AddForce(dt, uCur, uOld);
  AddForce(dt, vCur, vOld);
  swap(uCur, uOld);
  swap(vCur, vOld);
  Diffuse(1, dt, visc, uCur, uOld);
  Diffuse(2, dt, visc, vCur, vOld);
  Project(uCur, vCur, uOld, vOld);

  swap(uCur, uOld);
  swap(vCur, vOld);
  Advect(1, dt, uCur, uOld, uOld, vOld);
  Advect(2, dt, vCur, vOld, uOld, vOld);
  Project(uCur, vCur, uOld, vOld);
}

//------------------------------------------------------------------------------
void Fluid::FluidSim::Project(float* u, float* v, float* p, float* div)
{
  float h = 1.0f / FLUID_SIZE;
  for (int j = 1; j <= FLUID_SIZE; j++)
  {
    for (int i = 1; i <= FLUID_SIZE; i++)
    {
      div[IX(i, j)] = -0.5f * h * (u[IX(i + 1, j)] - u[IX(i - 1, j)] + v[IX(i, j + 1)] - v[IX(i, j - 1)]);
      p[IX(i, j)] = 0;
    }
  }

  BoundaryConditions(0, div);
  BoundaryConditions(0, p);

  for (int k = 0; k < 20; k++)
  {
    for (int j = 1; j <= FLUID_SIZE; j++)
    {
      for (int i = 1; i <= FLUID_SIZE; i++)
      {
        p[IX(i, j)] =
            (div[IX(i, j)] + p[IX(i - 1, j)] + p[IX(i + 1, j)] + p[IX(i, j - 1)] + p[IX(i, j + 1)]) / 4;
      }
    }
    BoundaryConditions(0, p);
  }

  for (int j = 1; j <= FLUID_SIZE; j++)
  {
    for (int i = 1; i <= FLUID_SIZE; i++)
    {
      u[IX(i, j)] -= 0.5f * (p[IX(i + 1, j)] - p[IX(i - 1, j)]) / h;
      v[IX(i, j)] -= 0.5f * (p[IX(i, j + 1)] - p[IX(i, j - 1)]) / h;
    }
  }

  BoundaryConditions(1, u);
  BoundaryConditions(2, v);
}

//------------------------------------------------------------------------------
void Fluid::FluidSim::Verify()
{
  for (int i = 0; i < FLUID_SIZE_PADDED_SQ; ++i)
  {
    assert(!isnan(uCur[i]));
    assert(!isnan(uOld[i]));
  }
}

//------------------------------------------------------------------------------
void Fluid::FluidSim::BoundaryConditions(int b, float* x)
{
  int N = FLUID_SIZE;
  for (int i = 1; i <= N; i++)
  {
    x[IX(0, i)] = b == 1 ? -x[IX(1, i)] : x[IX(1, i)];
    x[IX(N + 1, i)] = b == 1 ? -x[IX(N, i)] : x[IX(N, i)];
    x[IX(i, 0)] = b == 2 ? -x[IX(i, 1)] : x[IX(i, 1)];
    x[IX(i, N + 1)] = b == 2 ? -x[IX(i, N)] : x[IX(i, N)];
  }
  x[IX(0, 0)] = 0.5f * (x[IX(1, 0)] + x[IX(0, 1)]);
  x[IX(0, N + 1)] = 0.5f * (x[IX(1, N + 1)] + x[IX(0, N)]);
  x[IX(N + 1, 0)] = 0.5f * (x[IX(N, 0)] + x[IX(N + 1, 1)]);
  x[IX(N + 1, N + 1)] = 0.5f * (x[IX(N, N + 1)] + x[IX(N + 1, N)]);
}

//------------------------------------------------------------------------------
void Fluid::UpdateFluidTexture()
{
  static int texture = 2;
  if (g_KeyUpTrigger.IsTriggered('B'))
    texture = (texture + 1) % 3;

  float s = BLACKBOARD.GetFloatVar("fluid.diffuseScale");
  u32* p = _ctx->MapWriteDiscard<u32>(_fluidTexture);
  for (int i = 0; i < FluidSim::FLUID_SIZE; ++i)
  {
    for (int j = 0; j < FluidSim::FLUID_SIZE; ++j)
    {
      float u = Clamp(0.f, 1.f, s * (0.5f + _sim.uCur[FluidSim::IX(j+1, i+1)]));
      float v = Clamp(0.f, 1.f, s * (0.5f + _sim.vCur[FluidSim::IX(j+1, i+1)]));
      float d = Clamp(0.f, 1.f, s * _sim.dCur[FluidSim::IX(j+1, i+1)]);

      float vals[] = {u, v, d };
      float f = vals[texture];

      u32 r = (u32)(255 * f);
      u32 g = (u32)(255 * f);
      u32 b = (u32)(255 * f);
      p[i * FluidSim::FLUID_SIZE + j] = (0xff000000) | (b << 16) | (g << 8) | (r << 0);
    }
  }
  _ctx->Unmap(_fluidTexture);
}

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Fluid::RenderParameterSet()
{
  ImGui::Image((void*)&_fluidTexture, 
    ImVec2(4 * (float)FluidSim::FLUID_SIZE, 4 * (float)FluidSim::FLUID_SIZE));

  if (ImGui::Button("Reset"))
    Reset();
}
#endif

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Fluid::SaveParameterSet()
{
  SaveSettings(_settings);
}
#endif

//------------------------------------------------------------------------------
void Fluid::Reset()
{
  _camera._pos = Vector3(0.f, 0.f, 0.f);
  _camera._pitch = _camera._yaw = _camera._roll = 0.f;
}

//------------------------------------------------------------------------------
bool Fluid::Close()
{
  return true;
}

//------------------------------------------------------------------------------
BaseEffect* Fluid::Create(const char* name, const char* config, u32 id)
{
  return new Fluid(name, config, id);
}

//------------------------------------------------------------------------------
const char* Fluid::Name()
{
  return "fluid";
}

//------------------------------------------------------------------------------
void Fluid::Register()
{
  DEMO_ENGINE.RegisterFactory(Name(), Fluid::Create);
}
