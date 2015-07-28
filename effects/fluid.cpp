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
#include "../scheduler.hpp"
#include "../mesh_utils.hpp"

using namespace tano;
using namespace bristol;
using namespace tano::scheduler;

static int NUM_GRIDS = FluidSim::FLUID_SIZE;

#define SHOW_GREETS 1

static int GRID_DIRS[] = { -1, 0, 0, -1, 1, 0, 0, 1 };

//------------------------------------------------------------------------------
Fluid::Fluid(const string& name, const string& config, u32 id) : BaseEffect(name, config, id)
{
#if WITH_IMGUI
  PROPERTIES.Register(Name(), bind(&Fluid::RenderParameterSet, this), bind(&Fluid::SaveParameterSet, this));

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
GreetsBlock::~GreetsBlock()
{
  SeqDelete(&_data);
}

//------------------------------------------------------------------------------
GreetsBlock::GreetsData::GreetsData(int w, int h)
  : width(w)
  , height(h)
  , particleCount(w*h)
{
}

//------------------------------------------------------------------------------
GreetsBlock::GreetsData::~GreetsData()
{
  for (vector<PathElem*>& path : paths)
  {
    SeqDelete(&path);
  }
  paths.clear();
}

//------------------------------------------------------------------------------
void GreetsBlock::Render()
{
}

//------------------------------------------------------------------------------
void GreetsBlock::Update(const UpdateState& state)
{
  for (GreetsData* d : _data)
  {
    d->Update(state);
  }
}

//------------------------------------------------------------------------------
void GreetsBlock::GreetsData::Render()
{
}

//------------------------------------------------------------------------------
bool GreetsBlock::Init()
{
  BEGIN_INIT_SEQUENCE();

  SeqDelete(&_data);

  vector<char> buf;
  INIT(RESOURCE_MANAGER.LoadFile("gfx/greets.png", &buf));
  int w, h, c;
  const char* greetsBuf =
    (const char*)stbi_load_from_memory((const u8*)buf.data(), (int)buf.size(), &w, &h, &c, 4);

  int NUM_PARTICLES = BLACKBOARD.GetIntVar("fluid.particlesPerSegment");
  float speedMin = BLACKBOARD.GetFloatVar("fluid.speedMin");
  float speedMax = BLACKBOARD.GetFloatVar("fluid.speedMax");

  float speedMean = BLACKBOARD.GetFloatVar("fluid.speedMean");
  float speedVariance = BLACKBOARD.GetFloatVar("fluid.speedVariance");

  for (int i : {0, 10, 19, 29, 38})
  {
    GreetsData* d = new GreetsData(w, 7);
    d->CalcPath(w, 7, greetsBuf + i * w * 4);

    int total = (int)d->startingPoints.size() * NUM_PARTICLES;

    d->particles.resize(total);
    int idx = 0;
    for (const PathElem* p : d->startingPoints)
    {
      // find first valid starting dir
      int dir = 0;
      for (int i = 0; i < 4; ++i)
      {
        if (d->IsValid(p->x + GRID_DIRS[i*2+0], p->y + GRID_DIRS[i*2+1]))
        {
          dir = i;
          break;
        }
      }

      for (int i = idx; i < idx + NUM_PARTICLES; ++i)
      {
        d->particles[i].x = p->x;
        d->particles[i].y = p->y;
        float ll = GaussianRand(speedMean, speedVariance);
        d->particles[i].speed = ll;
        d->particles[i].cur = ll;
        //d->particles[i].dir = dir;
        d->particles[i].dir = rand() % 4;
      }

      idx += NUM_PARTICLES;
    }

    _data.push_back(d);
  }

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
void GreetsBlock::GreetsData::Update(const UpdateState& state)
{
  float dt = state.delta.TotalSecondsAsFloat();

  float changeProb = BLACKBOARD.GetFloatVar("fluid.changeProb");

  for (GreetsBlock::Particle& p : particles)
  {
    // check if it's time to move
    p.cur -= dt;
    if (p.cur > 0)
      continue;

    particleCount[p.y*width+p.x]--;

    p.cur = p.speed;

    // check if we can go in the current direction
    int forwardDirs[4];
    int allDirs[4];
    int numForwardDirs = 0;
    int numAllDirs = 0;

    int backdir = (p.dir + 2) % 4;

    bool curDirectionValid = false;
    for (int i = 0; i < 4; ++i)
    {
      if (IsValid(p.x + GRID_DIRS[i*2+0], p.y + GRID_DIRS[i*2+1]))
      {
        allDirs[numAllDirs++] = i;

        if (i == p.dir)
        {
          curDirectionValid = true;
        }
        else
        {
          if (i != backdir)
          {
            forwardDirs[numForwardDirs++] = i;
          }
        }
      }
    }

    if (numAllDirs == 0)
    {
      // TODO: ok, why does this happen? single block?
      return;
    }

    if (curDirectionValid)
    {
      if (numForwardDirs && randf(0.f, 1.0f) > changeProb)
      {
        p.dir = forwardDirs[rand() % numForwardDirs];
      }
    }
    else
    {
      if (numForwardDirs)
      {
        p.dir = forwardDirs[rand() % numForwardDirs];
      }
      else
      {
        p.dir = allDirs[rand() % numAllDirs];
      }
    }

    p.x += GRID_DIRS[p.dir*2+0];
    p.y += GRID_DIRS[p.dir*2+1];

    particleCount[p.y*width+p.x]++;
  }
}

//------------------------------------------------------------------------------
bool GreetsBlock::GreetsData::IsValid(int x, int y)
{
  return x >= 0 && x < width && y >= 0 && y < height && background[y*width+x] == 0;
}

//------------------------------------------------------------------------------
void GreetsBlock::GreetsData::CalcPath(int w, int h, const char* buf)
{
  background.resize(w*h);
  for (int i = 0; i <h; ++i)
  {
    for (int j = 0; j < w; ++j)
    {
      //background[i*w + j] = max(buf[4 * (i*w + j) + 0], max(buf[4 * (i*w + j) + 1], max(buf[4 * (i*w + j) + 2], buf[4 * (i*w + j) + 3])));
      background[i*w + j] = buf[4 * (i*w + j)];
    }
  }

  enum
  {
    FREE = 0,
    PENDING = 1,
    VISITED = 2
  };

  u8* visited = g_ScratchMemory.Alloc<u8>(w * h);
  memset(visited, FREE, w * h);

  struct Char
  {
    int xStart, xEnd;
  };

  vector<Char> chars;

  auto IsPosValid = [=](int x, int y, int w, int h)
  {
    return x >= 0 && x < w && y >= 0 && y < h && *(int*)&buf[4*(x+y*w)] != 0xffffffff;
  };

  auto IsEmptyCol = [=](int x)
  {
    for (int i = 0; i < h; ++i)
    {
      if (buf[(x + i * w) * 4] == 0)
        return false;
    }
    return true;
  };

  vector<pair<int, int>> starts;

  // find each starting point
  for (int i = 0; i < h; ++i)
  {
    for (int j = 0; j < w; ++j)
    {
      // bytes are stored as RGBA (from LSB to MSB)
      u32 rgba = *(u32*)&buf[4*(i*w+j)+0];
      if (rgba == 0xff0000ff)
      {
        starts.push_back(make_pair(j, i));
      }
    }
  }


  for (const pair<int, int>& s : starts)
  {
    vector<PathElem*> path;
    // this is pretty much a standard flood fill. the only interesting part
    // is using the tri-state visited flag, and I kind wonder how if made other
    // stuff that isn't broken without this :)
    deque<PathElem*> frontier;
    int x = s.first;
    int y = s.second;
    PathElem* p = new PathElem{ x, y};
    startingPoints.push_back(p);
    frontier.push_back(p);
    while (!frontier.empty())
    {
      PathElem* cur = frontier.front();
      frontier.pop_front();
      if (visited[cur->x + cur->y * w] == VISITED)
        continue;

      path.push_back(cur);

      visited[cur->x + cur->y * w] = VISITED;

      int ofs[] = { -1, 0, 0, -1, +1, 0, 0, +1 };
      for (int i = 0; i < 4; ++i)
      {
        int xx = cur->x + ofs[i * 2 + 0];
        int yy = cur->y + ofs[i * 2 + 1];
        if (IsPosValid(xx, yy, w, h) && visited[xx + yy * w] == FREE)
        {
          visited[xx + yy * w] = PENDING;
          PathElem* p = new PathElem{ xx, yy };
          frontier.push_front(p);
        }
      }
    }
    paths.push_back(path);
  }
}

//------------------------------------------------------------------------------
bool Fluid::Init()
{
  BEGIN_INIT_SEQUENCE();

#if SHOW_GREETS
  _fluidTexture =
    GRAPHICS.CreateTexture(64, 8, DXGI_FORMAT_R8G8B8A8_UNORM, nullptr);
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
    GeneratePlaneIndices(NUM_GRIDS+1, NUM_GRIDS+1, &indices);
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

  INIT(_greetsBlock.Init());

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool Fluid::Update(const UpdateState& state)
{
  UpdateCameraMatrix(state);
  UpdateFluidTexture();
  _sim.Update(state);

  UpdateBackgroundTexture(state.delta.TotalSecondsAsFloat());

  _greetsBlock.Update(state);

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

      V2 aa((float)(i - size / 2), (float)(j - size / 2));
      aa = Normalize(aa);
      V2 bb = V2(cosf(angle), sinf(angle));
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
          r * (old[IX(x, y)] +
                  a * (out[IX(x - 1, y)] + out[IX(x + 1, y)] + out[IX(x, y - 1)] + out[IX(x, y + 1)]));
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
            r * (old[IX(x, y)] +
                    a * (out[IX(x - 1, y)] + out[IX(x + 1, y)] + out[IX(x, y - 1)] + out[IX(x, y + 1)]));
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
      if (x > FLUID_SIZE + 0.5)
        x = FLUID_SIZE + 0.5;
      int i0 = (int)x;
      int i1 = i0 + 1;
      if (y < 0.5)
        y = 0.5;
      if (y > FLUID_SIZE + 0.5)
        y = FLUID_SIZE + 0.5;
      int j0 = (int)y;
      int j1 = j0 + 1;
      float s1 = x - i0;
      float s0 = 1 - s1;
      float t1 = y - j0;
      float t0 = 1 - t1;
      out[IX(i, j)] = s0 * (t0 * old[IX(i0, j0)] + t1 * old[IX(i0, j1)]) +
                      s1 * (t0 * old[IX(i1, j0)] + t1 * old[IX(i1, j1)]);
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

      verts->tex = Vector2(
        _textureU[i * FluidSim::FLUID_SIZE_PADDED + 1 + j],
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
      int cnt = d->particleCount[i*d->width+j];

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

      float vals[] = { u, v, d };
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

  _greetsBlock.Init();
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
