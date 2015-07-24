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
void Fluid::FluidSim::Update(const FixedUpdateState& state)
{
  DensityStep(state);
  VelocityStep(state);
}

//------------------------------------------------------------------------------
void Fluid::FluidSim::AddForces(const FixedUpdateState& state)
{
  for (int i = 0; i < FLUID_SIZE_PADDED_SQ; ++i)
  {
    d[i] = state.delta * forces[i];
  }
}

//------------------------------------------------------------------------------
void Fluid::FluidSim::Diffuse(const FixedUpdateState& state)
{
  float diffRate = 0.5f;
  float a = state.delta * diffRate * FLUID_SIZE_SQ;

  // Implicit technique for calcuating D(n+1).
  // Instead of D(t+1) = D(t) + dt * D(t, stuff), do
  // D(t) = D(t+1) - dt * D(t+1, stuff) 

  // Gauss-Seidel solver
  for (int k = 0; k < 20; ++k)
  {
    for (int y = 1; y <= FLUID_SIZE; ++y)
    {
      for (int x = 1; x <= FLUID_SIZE; ++x)
      {
        d[IX(x, y)] =
            (d0[IX(x, y)] + a * (d[IX(x - 1, y)] + d[IX(x + 1, y)] + d[IX(x, y - 1)] + d[IX(x, y + 1)])) /
            (1 + 4 * a);
      }
    }

    BoundaryConditions();
  }
}

//------------------------------------------------------------------------------
void Fluid::FluidSim::Advect(const FixedUpdateState& state)
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
      V2 p = V2(fx, fy) - dt * v[IX(x, y)];
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

      d[IX(x, y)] = lerp(
        lerp(d0[IX(x0, y0)], d0[IX(x1, y0)], s0),
        lerp(d0[IX(x0, y1)], d0[IX(x1, y1)], s1),
        t0);

      fx += 1;
    }
    fy += 1;
  }
}

//------------------------------------------------------------------------------
void Fluid::FluidSim::DensityStep(const FixedUpdateState& state)
{
  AddForces(state);
  swap(d0, d);
  Diffuse(state);
  swap(d0, d);
  Advect(state);
}

//------------------------------------------------------------------------------
void Fluid::FluidSim::VelocityStep(const FixedUpdateState& state)
{
}

//------------------------------------------------------------------------------
void Fluid::FluidSim::BoundaryConditions()
{
}

//------------------------------------------------------------------------------
void Fluid::UpdateFluidTexture()
{
  u32* p = _ctx->MapWriteDiscard<u32>(_fluidTexture);
  for (int i = 0; i < FluidSim::FLUID_SIZE; ++i)
  {
    for (int j = 0; j < FluidSim::FLUID_SIZE; ++j)
    {
      u32 r = 0xff;
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
  ImGui::Image((void*)&_fluidTexture, ImVec2((float)FluidSim::FLUID_SIZE, (float)FluidSim::FLUID_SIZE));

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
