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
#include "../scheduler.hpp"
#include "../mesh_utils.hpp"

using namespace tano;
using namespace bristol;
using namespace tano::scheduler;

static int NUM_GRIDS = FluidSim::FLUID_SIZE;

#define SHOW_GREETS 0

//------------------------------------------------------------------------------
Fluid::Fluid(const string& name, const string& config, u32 id) : BaseEffect(name, config, id)
{
}

//------------------------------------------------------------------------------
Fluid::~Fluid()
{
}

//------------------------------------------------------------------------------
bool Fluid::OnConfigChanged(const vector<char>& buf)
{
  _settings = ParseFluidSettings(InputBuffer(buf));
  return true;
}

//------------------------------------------------------------------------------
bool Fluid::Init()
{
  BEGIN_INIT_SEQUENCE();

#if SHOW_GREETS
  _fluidTexture = GRAPHICS.CreateTexture(64, 8, DXGI_FORMAT_R8G8B8A8_UNORM, nullptr);
#else
  _fluidTexture =
      GRAPHICS.CreateTexture(FluidSim::FLUID_SIZE, FluidSim::FLUID_SIZE, DXGI_FORMAT_R8G8B8A8_UNORM, nullptr);
#endif

  {
    // grid
    vector<D3D11_INPUT_ELEMENT_DESC> inputs = {
        CD3D11_INPUT_ELEMENT_DESC("SV_POSITION", DXGI_FORMAT_R32G32B32A32_FLOAT),
        CD3D11_INPUT_ELEMENT_DESC("TEXCOORD", DXGI_FORMAT_R32G32_FLOAT),
    };

    vector<u32> indices;
    GeneratePlaneIndices(NUM_GRIDS + 1, NUM_GRIDS + 1, &indices);
    // clang-format off
    INIT(_backgroundBundle.Create(BundleOptions()
      .RasterizerDesc(rasterizeDescCullNone)
      .DepthStencilDesc(depthDescDepthDisabled)
      .VertexShader("shaders/out/fluid.texture", "VsMain")
      .PixelShader("shaders/out/fluid.texture", "PsMain")
      .InputElements(inputs)
      .StaticIb((int)indices.size(), sizeof(u32), indices.data())
      .DynamicVb((NUM_GRIDS + 1) * (NUM_GRIDS + 1), sizeof(Pos4Tex))));
    // clang-format on

    INIT_RESOURCE(_backgroundTexture, RESOURCE_MANAGER.LoadTexture("gfx/abstract1.jpg"));
    InitBackgroundTexture();
  }

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool Fluid::Update(const UpdateState& state)
{
  UpdateCameraMatrix(state);
  UpdateFluidTexture();
  _sim.Update(state);

  UpdateBackgroundTexture(state.delta.TotalSecondsAsFloat());

  return true;
}

//------------------------------------------------------------------------------
bool Fluid::FixedUpdate(const FixedUpdateState& state)
{
  _camera.Update(state.delta);
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
#if !SHOW_GREETS
  {
    // background
    _ctx->SetBundleWithSamplers(_backgroundBundle, PixelShader);
    _ctx->SetShaderResource(_backgroundTexture);
    _ctx->DrawIndexed(6 * NUM_GRIDS * NUM_GRIDS, 0, 0);
  }
#endif
  return true;
}

//------------------------------------------------------------------------------
FluidSim::FluidSim()
{
  for (int i = 0; i < FLUID_SIZE_PADDED_SQ; ++i)
  {
    density0[i] = 0.f;
    density1[i] = 0.f;

    u0[i] = 0.f;
    u1[i] = 0.f;

    v0[i] = 0.f;
    v1[i] = 0.f;
  }
}

//------------------------------------------------------------------------------
void FluidSim::Update(const UpdateState& state)
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
      int x = FLUID_SIZE_PADDED / 2 - size / 2 + j;
      int y = FLUID_SIZE_PADDED / 2 - size / 2 + i;

      vec2 aa((float)(i - size / 2), (float)(j - size / 2));
      aa = Normalize(aa);
      vec2 bb = vec2(cosf(angle), sinf(angle));
      float d = Dot(aa, bb);

      dOld[IX(x, y)] += diffuseStrength; // / 2 + (diffuseStrength / 2) * sinf(2 * angle);
      uOld[IX(x, y)] += d * velocityStrength * aa.x;
      vOld[IX(x, y)] += d * velocityStrength * aa.y;
    }
  }

#if !SHOW_GREETS
  float dt = BLACKBOARD.GetFloatVar("fluid.timeScale") * state.delta.TotalSecondsAsFloat();
  DensityStep(dt);
  VelocityStep(dt);
#endif
}

//------------------------------------------------------------------------------
void FluidSim::AddForce(float dt, float* out, float* force)
{
  for (int i = 0; i < FLUID_SIZE_PADDED_SQ; ++i)
  {
    out[i] += dt * force[i];
  }
}

//------------------------------------------------------------------------------
void FluidSim::FluidKernelWorker(const TaskData& td)
{
  FluidKernelChunk* chunk = (FluidKernelChunk*)td.kernelData.data;
  int yStart = chunk->yStart;
  int yEnd = chunk->yEnd;
  float* out = chunk->out;
  float* old = chunk->old;

  float a = chunk->dt * chunk->diff * FLUID_SIZE_SQ;
  float r = 1.0f / (1 + 4 * a);

  for (int y = yStart; y < yEnd; ++y)
  {
    for (int x = 1; x <= FLUID_SIZE; ++x)
    {
      out[IX(x, y)] =
          r * (old[IX(x, y)]
                  + a * (out[IX(x - 1, y)] + out[IX(x + 1, y)] + out[IX(x, y - 1)] + out[IX(x, y + 1)]));
    }
  }
}

//------------------------------------------------------------------------------
void FluidSim::Diffuse(int b, float dt, float diff, float* out, float* old)
{
  float a = dt * diff * FLUID_SIZE_SQ;
  float r = 1.0f / (1 + 4 * a);

// clang-format off
  // The diffusion equation:
  // D(n+1)[i,j] = D(n)[i, j] + k * dt * (D(n)[i-1,j] + D(n)[i+1,j] + D(n)[i,j-1] + D(n)[i,j+1] - 4 * D(n)[i, j]) / h*h

  // Instead we set up an implicit equation:
  // D(n)[i,j] = D(n+1)[i, j] - k * dt * (D(n+1)[i-1,j] + D(n+1)[i+1,j] + D(n+1)[i,j-1] + D(n+1)[i,j+1] - 4 * D(n+1)[i, j]) / h*h
  // which has the form: b = A x (which x are the diffusion terms we want), and solve iteratively

    // The setup for both Jacobi and Gauss-Seidel is the same:

  // a11 * x1 + a12 * x2 + ... + a1n * xn = b1
  // solve for x1:
  // x1 = 1 / a11 * (b1 - a12 * x2 = a13 * x3 - ... - a12 * xn)
// clang-format on

#if 1
  enum
  {
    NUM_TASKS = 4,
    NUM_ITERATIONS = 10
  };
  int rows = FLUID_SIZE / NUM_TASKS;
  SimpleAppendBuffer<TaskId, NUM_ITERATIONS * NUM_TASKS> fluidTasks;

  // TODO: hmm, there is probably a crazy amount of false sharing and other badness going on

  // Gauss-Seidel solver
  for (int k = 0; k < NUM_ITERATIONS; ++k)
  {
    for (int i = 0; i < NUM_TASKS; ++i)
    {
      FluidKernelChunk* data = (FluidKernelChunk*)g_ScratchMemory.Alloc(sizeof(FluidKernelChunk));
      *data = FluidKernelChunk{1 + i * rows, 1 + (i + 1) * rows, out, old, dt, diff};
      KernelData kd;
      kd.data = data;
      kd.size = sizeof(FluidKernelChunk);
      fluidTasks.Append(SCHEDULER.AddTask(kd, FluidKernelWorker));
    }
  }

  for (const TaskId& taskId : fluidTasks)
    SCHEDULER.Wait(taskId);

  BoundaryConditions(b, out);

#else

  // Gauss-Seidel solver
  for (int k = 0; k < 10; ++k)
  {
    for (int y = 1; y <= FLUID_SIZE; ++y)
    {
      for (int x = 1; x <= FLUID_SIZE; ++x)
      {
        out[IX(x, y)] =
            r * (old[IX(x, y)]
                    + a * (out[IX(x - 1, y)] + out[IX(x + 1, y)] + out[IX(x, y - 1)] + out[IX(x, y + 1)]));
      }
    }

    BoundaryConditions(b, out);
  }

#endif
}

//------------------------------------------------------------------------------
void FluidSim::Advect(int b, float dt, float* out, float* old, float* u, float* v)
{
  float dt0 = dt * FLUID_SIZE;
  for (int j = 1; j <= FLUID_SIZE; j++)
  {
    for (int i = 1; i <= FLUID_SIZE; i++)
    {
      float x = i - dt0 * u[IX(i, j)];
      float y = j - dt0 * v[IX(i, j)];
      if (x < 0.5)
        x = 0.5;
      if (x > FLUID_SIZE + 0.5f)
        x = FLUID_SIZE + 0.5f;
      int i0 = (int)x;
      int i1 = i0 + 1;
      if (y < 0.5)
        y = 0.5;
      if (y > FLUID_SIZE + 0.5f)
        y = FLUID_SIZE + 0.5f;
      int j0 = (int)y;
      int j1 = j0 + 1;
      float s1 = x - i0;
      float s0 = 1 - s1;
      float t1 = y - j0;
      float t0 = 1 - t1;
      out[IX(i, j)] = s0 * (t0 * old[IX(i0, j0)] + t1 * old[IX(i0, j1)])
                      + s1 * (t0 * old[IX(i1, j0)] + t1 * old[IX(i1, j1)]);
    }
  }

  BoundaryConditions(b, out);
}

//------------------------------------------------------------------------------
void FluidSim::DensityStep(float dt)
{
  float diff = BLACKBOARD.GetFloatVar("fluid.diff");

  AddForce(dt, dCur, dOld);
  swap(dCur, dOld);
  Diffuse(0, dt, diff, dCur, dOld);
  swap(dCur, dOld);
  Advect(0, dt, dCur, dOld, uCur, vCur);
}

//------------------------------------------------------------------------------
void FluidSim::VelocityStep(float dt)
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
void FluidSim::Project(float* u, float* v, float* p, float* div)
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
void FluidSim::Verify()
{
  for (int i = 0; i < FLUID_SIZE_PADDED_SQ; ++i)
  {
    assert(!isnan(uCur[i]));
    assert(!isnan(uOld[i]));
  }
}

//------------------------------------------------------------------------------
void FluidSim::BoundaryConditions(int b, float* x)
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
void Fluid::InitBackgroundTexture()
{
  _textureU.resize(FluidSim::FLUID_SIZE_PADDED_SQ);
  _textureV.resize(FluidSim::FLUID_SIZE_PADDED_SQ);

  float xInc = 2.0f / (float)NUM_GRIDS;
  float yInc = 2.0f / (float)NUM_GRIDS;

  float uInc = 1.0f / (float)NUM_GRIDS;
  float vInc = 1.0f / (float)NUM_GRIDS;

  float y = 1.0f;
  float v = 0.f;
  for (int i = 0; i <= NUM_GRIDS; ++i)
  {
    float x = -1.0f;
    float u = 0.0f;
    for (int j = 0; j <= NUM_GRIDS; ++j)
    {
      _textureU[i * FluidSim::FLUID_SIZE_PADDED + 1 + j] = u;
      _textureV[i * FluidSim::FLUID_SIZE_PADDED + 1 + j] = v;

      x += xInc;
      u += uInc;
    }

    y -= yInc;
    v += vInc;
  }
}

//------------------------------------------------------------------------------
void Fluid::UpdateBackgroundTexture(float dt)
{
  ObjectHandle h = _backgroundBundle.objects._vb;
  Pos4Tex* verts = _ctx->MapWriteDiscard<Pos4Tex>(h);
  Pos4Tex* vertsOrg = verts;

  vector<float> tmpU(FluidSim::FLUID_SIZE_PADDED_SQ);
  vector<float> tmpV(FluidSim::FLUID_SIZE_PADDED_SQ);

  _sim.Advect(0, dt, tmpU.data(), _textureU.data(), _sim.uCur, _sim.vCur);
  _sim.Advect(0, dt, tmpV.data(), _textureV.data(), _sim.uCur, _sim.vCur);

  _textureU.swap(tmpU);
  _textureV.swap(tmpV);

  float xInc = 2.0f / (float)NUM_GRIDS;
  float yInc = 2.0f / (float)NUM_GRIDS;

  float uInc = 1.0f / (float)NUM_GRIDS;
  float vInc = 1.0f / (float)NUM_GRIDS;

  float y = 1.0f;
  float v = 0.f;
  for (int i = 0; i <= NUM_GRIDS; ++i)
  {
    float x = -1.0f;
    float u = 0.0f;
    for (int j = 0; j <= NUM_GRIDS; ++j)
    {
      verts->pos = Vector4(x, y, 0, 1);

      verts->tex = Vector2(_textureU[i * FluidSim::FLUID_SIZE_PADDED + 1 + j],
          _textureV[i * FluidSim::FLUID_SIZE_PADDED + 1 + j]);

      verts++;

      x += xInc;
      u += uInc;
    }

    y -= yInc;
    v += vInc;
  }

  _ctx->Unmap(h);
}

#if SHOW_GREETS
//------------------------------------------------------------------------------
void Fluid::UpdateFluidTexture()
{
  int pitch;
  u32* p = _ctx->MapWriteDiscard<u32>(_fluidTexture, &pitch);
  pitch /= 4;

  GreetsBlock::GreetsData* d = _greetsBlock._data[_greetsBlock.curText];
  for (int i = 0; i < d->height; ++i)
  {
    for (int j = 0; j < d->width; ++j)
    {
      int cnt = d->particleCount[i * d->width + j];

      float f = Clamp(0.f, 1.f, (float)cnt / 10.f);
      u32 r = (u32)(255 * f);
      u32 g = (u32)(255 * f);
      u32 b = (u32)(255 * f);
      p[i * pitch + j] = (0xff000000) | (b << 16) | (g << 8) | (r << 0);
    }
  }

  _ctx->Unmap(_fluidTexture);
}
#else
//------------------------------------------------------------------------------
void Fluid::UpdateFluidTexture()
{
  static int texture = 2;
  if (g_KeyUpTrigger.IsTriggered('B'))
    texture = (texture + 1) % 3;

  float s = BLACKBOARD.GetFloatVar("fluid.diffuseScale");
  int pitch;
  u32* p = _ctx->MapWriteDiscard<u32>(_fluidTexture, &pitch);
  pitch /= 4;
  for (int i = 0; i < FluidSim::FLUID_SIZE; ++i)
  {
    for (int j = 0; j < FluidSim::FLUID_SIZE; ++j)
    {
      float u = Clamp(0.f, 1.f, s * (0.5f + _sim.uCur[FluidSim::IX(j + 1, i + 1)]));
      float v = Clamp(0.f, 1.f, s * (0.5f + _sim.vCur[FluidSim::IX(j + 1, i + 1)]));
      float d = Clamp(0.f, 1.f, s * _sim.dCur[FluidSim::IX(j + 1, i + 1)]);

      float vals[] = {u, v, d};
      float f = vals[texture];

      u32 r = (u32)(255 * f);
      u32 g = (u32)(255 * f);
      u32 b = (u32)(255 * f);
      p[i * pitch + j] = (0xff000000) | (b << 16) | (g << 8) | (r << 0);
    }
  }
  _ctx->Unmap(_fluidTexture);
}
#endif

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Fluid::RenderParameterSet()
{
#if SHOW_GREETS
  ImGui::SliderInt("text", &_greetsBlock.curText, 0, 4);

  float scale = 8;
  ImGui::Image((void*)&_fluidTexture, ImVec2(scale * 64, scale * 8));
#else
  ImGui::Image(
      (void*)&_fluidTexture, ImVec2(4 * (float)FluidSim::FLUID_SIZE, 4 * (float)FluidSim::FLUID_SIZE));
#endif

  if (ImGui::Button("Reset"))
    Reset();
}
#endif

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Fluid::SaveParameterSet(bool inc)
{
  SaveSettings(_settings, inc);
}
#endif

//------------------------------------------------------------------------------
void Fluid::Reset()
{
  _camera._pos = vec3(0.f, 0.f, 0.f);
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
