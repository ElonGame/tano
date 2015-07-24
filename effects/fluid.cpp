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
  return true;
}

//------------------------------------------------------------------------------
bool Fluid::FixedUpdate(const FixedUpdateState& state)
{
  _camera.Update(state);
  _sim.Update(state);
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
    dForce[i] = 0.f;
    uForce[i] = 0.f;
    vForce[i] = 0.f;

    density0[i] = 0.f;
    density1[i] = 0.f;

    u0[i] = 0.f;
    u1[i] = 0.f;

    v0[i] = 0.f;
    v1[i] = 0.f;
  }
}

//------------------------------------------------------------------------------
void Fluid::FluidSim::Update(const FixedUpdateState& state)
{
  static int count = 0;
  if ((count % 4) == 0)
  {
    DensityStep(state);
    VelocityStep(state);
  }

  int size = BLACKBOARD.GetIntVar("fluid.blobSize");
  float strength = BLACKBOARD.GetFloatVar("fluid.blobStrength");

  if ((count % 10) == 0)
  {
    for (int i = 0; i < size; ++i)
    {
      for (int j = 0; j < size; ++j)
      {
        dForce[IX((FLUID_SIZE - i) / 2, (FLUID_SIZE - j) / 2)] = strength;
        vForce[IX((FLUID_SIZE - i) / 2, (FLUID_SIZE - j) / 2)] = strength;
      }
    }
  }

  ++count;
}

//------------------------------------------------------------------------------
void Fluid::FluidSim::AddForce(const FixedUpdateState& state, float* out, float* force)
{
  for (int i = 0; i < FLUID_SIZE_PADDED_SQ; ++i)
  {
    out[i] = state.delta * force[i];
  }
}

//------------------------------------------------------------------------------
void Fluid::FluidSim::Diffuse(int b, const FixedUpdateState& state, float diff, float* out, float* old)
{
  float a = state.delta * diff * FLUID_SIZE_SQ;

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
void Fluid::FluidSim::Advect(int b, const FixedUpdateState& state, float* out, float* old, float* u, float *v)
{

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

  BoundaryConditions(b, out);
}

//------------------------------------------------------------------------------
void Fluid::FluidSim::DensityStep(const FixedUpdateState& state)
{
  float diff = BLACKBOARD.GetFloatVar("fluid.diff");
  AddForce(state, dCur, dForce);
  swap(dCur, dOld);
  Diffuse(0, state, diff, dCur, dOld);
  swap(dCur, dOld);
  Advect(0, state, dCur, dOld, uCur, vCur);
}

//------------------------------------------------------------------------------
void Fluid::FluidSim::VelocityStep(const FixedUpdateState& state)
{
  float visc = BLACKBOARD.GetFloatVar("fluid.visc");
  AddForce(state, vCur, vForce);
  swap(uCur, uOld);
  swap(vCur, vOld);
  Diffuse(1, state, visc, uCur, uOld);
  Diffuse(2, state, visc, vCur, vOld);
  Project(uCur, vCur, uOld, vOld);

  swap(uCur, uOld);
  swap(vCur, vOld);
  Advect(1, state, uCur, uOld, uOld, vOld);
  Advect(2, state, vCur, vOld, uOld, vOld);
  Project(uCur, vCur, uOld, vOld);
}

//------------------------------------------------------------------------------
void Fluid::FluidSim::Project(float* u, float* v, float* p, float* div)
{
  float h = 1.0 / FLUID_SIZE;
  for (int i = 1; i <= FLUID_SIZE; i++)
  {
    for (int j = 1; j <= FLUID_SIZE; j++)
    {
      div[IX(i, j)] = -0.5f * h * (u[IX(i + 1, j)] - u[IX(i - 1, j)] + v[IX(i, j + 1)] - v[IX(i, j - 1)]);
      p[IX(i, j)] = 0;
    }
  }

  BoundaryConditions(0, div);
  BoundaryConditions(0, p);

  for (int k = 0; k < 20; k++)
  {
    for (int i = 1; i <= FLUID_SIZE; i++)
    {
      for (int j = 1; j <= FLUID_SIZE; j++)
      {
        p[IX(i, j)] =
            (div[IX(i, j)] + p[IX(i - 1, j)] + p[IX(i + 1, j)] + p[IX(i, j - 1)] + p[IX(i, j + 1)]) / 4;
      }
    }
    BoundaryConditions(0, p);
  }

  for (int i = 1; i <= FLUID_SIZE; i++)
  {
    for (int j = 1; j <= FLUID_SIZE; j++)
    {
      u[IX(i, j)] -= 0.5f * (p[IX(i + 1, j)] - p[IX(i - 1, j)]) / h;
      v[IX(i, j)] -= 0.5f * (p[IX(i, j + 1)] - p[IX(i, j - 1)]) / h;
    }
  }

  BoundaryConditions(1, u);
  BoundaryConditions(2, v);
}
//------------------------------------------------------------------------------
void Fluid::FluidSim::BoundaryConditions(int b, float* x)
{
  for (int i = 1; i <= FLUID_SIZE; i++)
  {
    x[IX(0, i)] = b == 1 ? -x[IX(1, i)] : x[IX(1, i)];
    x[IX(FLUID_SIZE + 1, i)] = b == 1 ? -x[IX(FLUID_SIZE, i)] : x[IX(FLUID_SIZE, i)];
    x[IX(i, 0)] = b == 2 ? -x[IX(i, 1)] : x[IX(i, 1)];
    x[IX(i, FLUID_SIZE + 1)] = b == 2 ? -x[IX(i, FLUID_SIZE)] : x[IX(i, FLUID_SIZE)];
  }
  x[IX(0, 0)] = 0.5f * (x[IX(1, 0)] + x[IX(0, 1)]);
  x[IX(0, FLUID_SIZE + 1)] = 0.5f * (x[IX(1, FLUID_SIZE + 1)] + x[IX(0, FLUID_SIZE)]);
  x[IX(FLUID_SIZE + 1, 0)] = 0.5f * (x[IX(FLUID_SIZE, 0)] + x[IX(FLUID_SIZE + 1, 1)]);
  x[IX(FLUID_SIZE + 1, FLUID_SIZE + 1)] =
      0.5f * (x[IX(FLUID_SIZE, FLUID_SIZE + 1)] + x[IX(FLUID_SIZE + 1, FLUID_SIZE)]);
}

//------------------------------------------------------------------------------
void Fluid::UpdateFluidTexture()
{
  u32* p = _ctx->MapWriteDiscard<u32>(_fluidTexture);
  for (int i = 0; i < FluidSim::FLUID_SIZE; ++i)
  {
    for (int j = 0; j < FluidSim::FLUID_SIZE; ++j)
    {
      float f = Clamp(0.f, 1.f, _sim.dCur[FluidSim::IX(j, i)]);
      u32 r = (u32)(255 * f);
      u32 g = 0x00;
      u32 b = 0x00;
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
