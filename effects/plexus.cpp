#include "Plexus.hpp"
#include "../tano.hpp"
#include "../graphics.hpp"
#include "../graphics_context.hpp"
#include "../demo_engine.hpp"
#include "../resource_manager.hpp"
#include "../init_sequence.hpp"
#include "../generated/demo.parse.hpp"
#include "../mesh_utils.hpp"
#include "../fullscreen_effect.hpp"
#include "../arena_allocator.hpp"
#include "../stop_watch.hpp"
#include "../perlin2d.hpp"
#include "../blackboard.hpp"
#include "../fixed_deque.hpp"

using namespace tano;
using namespace bristol;

extern "C" float stb_perlin_noise3(float x, float y, float z);

static int NOISE_WIDTH = 512;
static int NOISE_HEIGHT = 512;

static int MAX_GREETS_WIDTH = 64;
static int MAX_GREETS_HEIGHT = 8;

static int GRID_DIRS[] = { -1, 0, 0, -1, 1, 0, 0, 1 };

//------------------------------------------------------------------------------
GreetsBlock::~GreetsBlock()
{
  SeqDelete(&_data);
}

//------------------------------------------------------------------------------
GreetsBlock::GreetsData::GreetsData(int w, int h)
  : width(w)
  , height(h)
  , targetParticleCount(w*h)
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
void GreetsBlock::Update(const UpdateState& state)
{
  for (GreetsData* d : _data)
  {
    d->Update(state);
  }
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
        if (d->IsValid(p->x + GRID_DIRS[i * 2 + 0], p->y + GRID_DIRS[i * 2 + 1]))
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
        d->particles[i].dir = dir;
        //d->particles[i].dir = rand() % 4;

        d->targetParticleCount[p->y*d->width + p->x]++;
      }

      idx += NUM_PARTICLES;
    }

    d->curParticleCount = d->targetParticleCount;

    _data.push_back(d);
  }

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
void GreetsBlock::GreetsData::Update(const UpdateState& state)
{
  float dt = state.delta.TotalSecondsAsFloat();

  float changeProb = BLACKBOARD.GetFloatVar("fluid.changeProb");
  float updateSpeed = BLACKBOARD.GetFloatVar("fluid.updateSpeed");

  for (int i = 0; i < (int)targetParticleCount.size(); ++i)
  {
    float diff = targetParticleCount[i] - curParticleCount[i];
    diff *= updateSpeed * state.delta.TotalSecondsAsFloat();
    curParticleCount[i] += diff;
  }

  for (GreetsBlock::Particle& p : particles)
  {
    // check if it's time to move
    p.cur -= dt;
    if (p.cur > 0)
      continue;

    p.cur += p.speed;

    // check if we can go in the current direction
    int forwardDirs[4];
    int allDirs[4];
    int numForwardDirs = 0;
    int numAllDirs = 0;

    int backdir = (p.dir + 2) % 4;

    bool curDirectionValid = false;
    for (int i = 0; i < 4; ++i)
    {
      if (IsValid(p.x + GRID_DIRS[i * 2 + 0], p.y + GRID_DIRS[i * 2 + 1]))
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

    targetParticleCount[p.y*width + p.x]--;

    p.x += GRID_DIRS[p.dir * 2 + 0];
    p.y += GRID_DIRS[p.dir * 2 + 1];

    targetParticleCount[p.y*width + p.x]++;
  }
}

//------------------------------------------------------------------------------
bool GreetsBlock::GreetsData::IsValid(int x, int y)
{
  return x >= 0 && x < width && y >= 0 && y < height && background[y*width + x] == 0;
}

//------------------------------------------------------------------------------
void GreetsBlock::GreetsData::CalcPath(int w, int h, const char* buf)
{
  background.resize(w*h);
  for (int i = 0; i < h; ++i)
  {
    for (int j = 0; j < w; ++j)
    {
      background[i*w + j] = *(int*)&buf[4 * (i*w + j)] == 0xffffffff ? 1 : 0;
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
    return x >= 0 && x < w && y >= 0 && y < h && *(int*)&buf[4 * (x + y*w)] != 0xffffffff;
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
      u32 rgba = *(u32*)&buf[4 * (i*w + j) + 0];
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
    PathElem* p = new PathElem{ x, y };
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
template <typename T>
void BlurLine(const T* src, T* dst, int size, float r)
{
  // Blur the data
  float scale = 1.f / (2.0f * r + 1.f);
  int m = (int)r;
  float alpha = r - m;

  // set up initial pixel

  T sum = src[0];
  for (int i = 0; i < m; i++)
    sum += src[IntMod(-i, size)] + src[IntMod(i, size)];
  sum += alpha * (src[IntMod(-m, size)] + src[IntMod(m, size)]);

  for (int i = 0; i < size; ++i)
  {
    dst[i] = sum * scale;
    sum += lerp(src[IntMod(i + m + 1, size)], src[IntMod(i + m + 2, size)], alpha);
    sum -= lerp(src[IntMod(i - m, size)], src[IntMod(i - m - 1, size)], alpha);
  }
}

//------------------------------------------------------------------------------
Plexus::Plexus(const string& name, const string& config, u32 id) : BaseEffect(name, config, id)
{
}

//------------------------------------------------------------------------------
Plexus::~Plexus()
{
}

SimpleAppendBuffer<vec3, 1024> g_randomPoints;

//------------------------------------------------------------------------------
void GenRandomPoints(float kernelSize)
{
  vec3* tmp = g_ScratchMemory.Alloc<vec3>(g_randomPoints.Capacity());
  for (int i = 0; i < g_randomPoints.Capacity(); ++i)
  {
    vec3 v(randf(-1.f, 1.f), randf(-1.f, 1.f), randf(-1.f, 1.f));
    v = Normalize(v);
    tmp[i] = v;
  }

  g_randomPoints.Resize(g_randomPoints.Capacity());

  BlurLine(tmp, g_randomPoints.Data(), g_randomPoints.Capacity(), kernelSize);
}

//------------------------------------------------------------------------------
bool Plexus::OnConfigChanged(const vector<char>& buf)
{
  return ParsePlexusSettings(InputBuffer(buf), &_settings);
}

//------------------------------------------------------------------------------
bool Plexus::Init()
{
  BEGIN_INIT_SEQUENCE();

  _freeflyCamera.FromProtocol(_settings.camera);

  GenRandomPoints(_settings.deform.blur_kernel);

  INIT(_cbPlexus.Create());

  // clang-format off
  INIT(_plexusLineBundle.Create(BundleOptions()
    .VertexShader("shaders/out/plexus", "VsLines")
    .GeometryShader("shaders/out/plexus", "GsLines")
    .PixelShader("shaders/out/plexus", "PsLines")
    .VertexFlags(VF_POS)
    .RasterizerDesc(rasterizeDescCullNone)
    .BlendDesc(blendDescBlendOneOne)
    .DepthStencilDesc(depthDescDepthWriteDisabled)
    .DynamicVb(128 * 1024, sizeof(vec3))
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST)));

  INIT_FATAL(_greetsBundle.Create(BundleOptions()
    .InputElement(CD3D11_INPUT_ELEMENT_DESC("POSITION", DXGI_FORMAT_R32G32B32_FLOAT))
    .InputElement(CD3D11_INPUT_ELEMENT_DESC("NORMAL", DXGI_FORMAT_R32G32B32_FLOAT))
    .VertexShader("shaders/out/tunnel.greets", "VsGreets")
    .PixelShader("shaders/out/tunnel.greets", "PsGreets")
    .DynamicVb(MAX_GREETS_HEIGHT*MAX_GREETS_WIDTH * 24 * 2, 2 * sizeof(vec3))
    .StaticIb(GenerateCubeIndicesFaceted(MAX_GREETS_HEIGHT*MAX_GREETS_WIDTH))));

  INIT_FATAL(_compositeBundle.Create(BundleOptions()
    .BlendDesc(blendDescWeightedBlendResolve)
    .DepthStencilDesc(depthDescDepthDisabled)
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/plexus.compose", "PsComposite")));

  // clang-format on

  INIT_FATAL(_greetsBlock.Init());
  CalcPoints(true);

  INIT(_cbGreets.Create());
  INIT_FATAL(_cbComposite.Create());

  _perlinTexture = GRAPHICS.CreateTexture(NOISE_WIDTH, NOISE_HEIGHT, DXGI_FORMAT_R8G8B8A8_UNORM, nullptr);

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
void Plexus::UpdateNoise()
{
  static bool opened = false;
  static bool firstTime = true;

#if WITH_IMGUI
  if (!ImGui::Begin("Noise", &opened))
  {
    ImGui::End();
    return;
  }

  bool recalc = firstTime;
  recalc |= ImGui::SliderInt("layer-lock", &_settings.noise.layer_lock, -1, 10);
  recalc |= ImGui::SliderInt("layers", &_settings.noise.num_layers, 1, 10);
  recalc |= ImGui::SliderFloat("max-scale", &_settings.noise.max_scale, 1, 20);
  recalc |= ImGui::SliderFloat("scale-factor", &_settings.noise.scale_factor, 1.0f, 25.0f);
  recalc |= ImGui::SliderFloat("max opacity", &_settings.noise.max_opacity, 0, 1);
  recalc |= ImGui::SliderFloat("opacity factor", &_settings.noise.opacity_factor, 0.1f, 2.0f);
  recalc |= ImGui::SliderFloat("turbulence", &_settings.noise.turbulence, 1, 500);

  ImGui::Image((void*)&_perlinTexture, ImVec2((float)NOISE_WIDTH, (float)NOISE_HEIGHT));

  ImGui::End();
  firstTime = false;
  if (!recalc)
    return;
#endif

  float scale = _settings.noise.max_scale;
  float opacity = 0;
  float layerOpacity = _settings.noise.max_opacity;

  float* pixels = g_ScratchMemory.Alloc<float>(NOISE_HEIGHT * NOISE_WIDTH);
  memset(pixels, 0, NOISE_WIDTH * NOISE_HEIGHT * 4);

  int layerLock = _settings.noise.layer_lock;
  for (int layer = 0; layer < _settings.noise.num_layers; ++layer)
  {
    if (layerLock != -1 && layerLock != layer)
    {
      scale *= _settings.noise.scale_factor;
      layerOpacity *= _settings.noise.opacity_factor;
      continue;
    }

    opacity = min(1.0f, opacity + layerOpacity);
    u32 a = 0xff;

    for (int i = 0; i < NOISE_HEIGHT; ++i)
    {
      for (int j = 0; j < NOISE_WIDTH; ++j)
      {
        float f = 0.5f + 0.5f * Perlin2D::Value(scale * j / NOISE_WIDTH, scale * i / NOISE_HEIGHT);
        float v = layerOpacity * f;
        pixels[i * NOISE_WIDTH + j] += v;
      }
    }

    scale *= _settings.noise.scale_factor;
    layerOpacity *= _settings.noise.opacity_factor;
  }

#if 0
  const FullTonemapSettings& t = _settings.tonemap;
  u32* p = _ctx->MapWriteDiscard<u32>(_perlinTexture);
  for (int i = 0; i < NOISE_HEIGHT; ++i)
  {
    for (int j = 0; j < NOISE_WIDTH; ++j)
    {
      float f;
      if (_settings.tonemap.enabled)
      {
        f = 255 * Clamp(0.f,
                      1.f,
                      ToneMap(pixels[i * NOISE_WIDTH + j],
                            t.cross_over,
                            t.black_point,
                            t.white_point,
                            t.toe,
                            t.shoulder));
      }
      else
      {
        f = pixels[i * NOISE_WIDTH + j];
        float angle = 2 * XM_PI * f;
        float s = _settings.noise.turbulence;
        int xx = (int)(j + s * cos(angle));
        int yy = (int)(i + s * sin(angle));
        int x = IntMod(xx, NOISE_WIDTH);
        int y = IntMod(yy, NOISE_HEIGHT);
        f = 255 * Clamp(0.f, 1.f, pixels[y * NOISE_WIDTH + x]);
      }
      u32 val = (u32)f;
      p[i * NOISE_WIDTH + j] = (0xff000000) | (val << 16) | (val << 8) | (val << 0);
    }
  }
  _ctx->Unmap(_perlinTexture);
#endif
}

//------------------------------------------------------------------------------
void Plexus::PointsTest(const UpdateState& state)
{
  _points.Clear();

  float radiusInc = _settings.sphere.radius / _settings.sphere.layers;
  float phiInc = 2 * XM_PI / _settings.sphere.slices;
  float thetaInc = XM_PI / (_settings.sphere.stacks > 1 ? _settings.sphere.stacks - 1 : 1);

  float r = _settings.sphere.radius;
  float perlinScale = _settings.deform.perlin_scale;

  int layers = _settings.sphere.layers;
  int slices = _settings.sphere.slices;
  int stacks = _settings.sphere.stacks;

  int num = layers * slices * stacks;

  int nodeIdx = 0;

  for (int i = 0; i < layers; ++i)
  {
    bool edgeOut = i > 0;
    bool edgeIn = i < layers - 1;

    float phi = 0;
    for (int j = 0; j < slices; ++j)
    {
      float theta = 0;
      for (int k = 0; k < stacks; ++k)
      {
        bool edgeUp = k > 0;
        bool edgeDown = k < stacks - 1;

        vec3 pt = FromSpherical(r, phi, theta);

        float s = fabs(1024 * stb_perlin_noise3(pt.x / perlinScale, pt.y / perlinScale, pt.z / perlinScale));
        vec3 v = g_randomPoints[IntMod((int)s, 1024)];
        pt = pt + _settings.deform.noise_strength * v;

        _points.Append(pt);

        nodeIdx++;
        theta += thetaInc;
      }
      phi += phiInc;
    }
    r -= radiusInc;
  }
}

//------------------------------------------------------------------------------
void Plexus::CalcPoints(bool recalcEdges)
{
  struct Edge
  {
    int e[6];
  };
  SimpleAppendBuffer<Edge, MAX_POINTS> edges;

  _points.Clear();

  float radiusInc = _settings.sphere.radius / _settings.sphere.layers;
  float phiInc = 2 * XM_PI / _settings.sphere.slices;
  float thetaInc = XM_PI / (_settings.sphere.stacks > 1 ? _settings.sphere.stacks - 1 : 1);

  float r = _settings.sphere.radius;

  int layers = _settings.sphere.layers;
  int slices = _settings.sphere.slices;
  int stacks = _settings.sphere.stacks;

  int num = layers * slices * stacks;

  int nodeIdx = 0;

  for (int i = 0; i < layers; ++i)
  {
    bool edgeOut = i > 0;
    bool edgeIn = i < layers - 1;

    float phi = 0;
    for (int j = 0; j < slices; ++j)
    {
      float theta = 0;
      for (int k = 0; k < stacks; ++k)
      {
        bool edgeUp = k > 0;
        bool edgeDown = k < stacks - 1;

        vec3 pt = FromSpherical(r, phi, theta);
        float s = _settings.deform.noise_strength * stb_perlin_noise3(pt.x, pt.y, pt.z);
        pt = pt + s * pt;

        _points.Append(pt);

        // Create the edges. Every point basically has left/right/up/down/out/in and
        // then we need to take care of boundaries
        if (recalcEdges)
        {
          int edgeIdx = 2;
          int(&e)[6] = edges.Append(Edge()).e;
          e[0] = IntMod(nodeIdx - stacks, num);
          e[1] = IntMod(nodeIdx - stacks, num);

          if (edgeUp)
            e[edgeIdx++] = IntMod(nodeIdx - 1, num);
          if (edgeDown)
            e[edgeIdx++] = IntMod(nodeIdx + 1, num);

          if (edgeOut)
            e[edgeIdx++] = IntMod(nodeIdx - (slices * stacks), num);
          if (edgeIn)
            e[edgeIdx++] = IntMod(nodeIdx - (slices * stacks), num);

          // set a sentinal if needed
          if (edgeIdx != 6)
            e[edgeIdx - 1] = -1;
        }

        nodeIdx++;
        theta += thetaInc;
      }
      phi += phiInc;
    }
    r -= radiusInc;
  }

  if (recalcEdges)
  {
    // order the neighbours in depth-first edge order
    SAFE_ADELETE(_neighbours);
    _neighbours = new int[num * num];

    u8* visited = g_ScratchMemory.Alloc<u8>(num);

    for (int i = 0; i < num; ++i)
    {
      memset(visited, 0, num);
      FixedDequeue<int> frontier(g_ScratchMemory.Alloc<int>(num), num);
      frontier.PushFront(i);
      int idx = 0;
      while (!frontier.IsEmpty())
      {
        int cur = frontier.Front();
        _neighbours[i * num + idx] = cur;
        idx++;
        frontier.PopFront();
        visited[cur] = 1;

        for (int j = 0; j < 6; ++j)
        {
          int tmp = edges[cur].e[j];
          if (tmp == -1)
            break;
          if (!visited[tmp])
          {
            visited[tmp] = 1;
            frontier.PushBack(tmp);
          }
        }
      }
    }
  }
}

//------------------------------------------------------------------------------
void Plexus::UpdateGreets(const UpdateState& state)
{
  _greetsBlock.Update(state);
  _numGreetsCubes = 0;

  ObjectHandle handle = _greetsBundle.objects._vb;
  vec3* verts = _ctx->MapWriteDiscard<vec3>(handle);

  GreetsBlock::GreetsData* data = _greetsBlock._data[_greetsBlock.curText];
  int w = data->width;
  int h = data->height;
  float fw = (float)w;
  float fh = (float)h;

  float s = 10;
  vec3 pos(-fw / 2 * s + s / 2, fh / 2 * s - s / 2, 300);
  float orgX = pos.x;
  for (int i = 0; i < h; ++i)
  {
    pos.x = orgX;
    for (int j = 0; j < w; ++j)
    {
      float ss = s * Clamp(0.f, 1.f, (float)data->curParticleCount[i*w + j] / 10);
      if (ss > 0)
      {
        verts = AddCubeWithNormal(verts, pos, ss / 2);
        _numGreetsCubes++;
      }
      pos.x += s;
    }
    pos.y -= s;
  }

  _ctx->Unmap(handle);
}

//------------------------------------------------------------------------------
bool Plexus::Update(const UpdateState& state)
{

  float globalTime = state.globalTime.TotalSecondsAsFloat();
  BLACKBOARD.ClearNamespace();
  float lo = BLACKBOARD.GetFloatVar("Beat-Lo", globalTime);

  BLACKBOARD.SetNamespace("plexus");
  float base = BLACKBOARD.GetFloatVar("plexus.base");
  float scale = BLACKBOARD.GetFloatVar("plexus.scale");

  _settings.deform.noise_strength = base + scale * lo;
  CalcPoints(false);


  UpdateCameraMatrix(state);
  PointsTest(state);
  UpdateGreets(state);
  return true;
}

//------------------------------------------------------------------------------
bool Plexus::FixedUpdate(const FixedUpdateState& state)
{
  return true;
}

//------------------------------------------------------------------------------
void Plexus::UpdateCameraMatrix(const UpdateState& state)
{
  Matrix view = _freeflyCamera._view;
  Matrix proj = _freeflyCamera._proj;

  Matrix viewProj = view * proj;

  float rotXDiv = BLACKBOARD.GetFloatVar("plexus.rotXDivisor");
  float rotYDiv = BLACKBOARD.GetFloatVar("plexus.rotXDivisor");
  static float angle = 0;
  angle += state.delta.TotalMilliseconds();
  Matrix mtx = Matrix::CreateRotationX(angle / rotXDiv) * Matrix::CreateRotationY(angle / rotYDiv);
  // Matrix mtx = Matrix::Identity();
  _cbPlexus.gs0.world = mtx.Transpose();
  _cbPlexus.gs0.viewProj = viewProj.Transpose();
  _cbPlexus.gs0.cameraPos = _freeflyCamera._pos;

  _cbGreets.vs0.viewProj = viewProj.Transpose();
  _cbGreets.vs0.objWorld = Matrix::Identity();
  _cbGreets.ps0.camPos = _freeflyCamera._pos;
}

//------------------------------------------------------------------------------
bool Plexus::Render()
{
  rmt_ScopedCPUSample(Plexus_Render);

  static Color black(0, 0, 0, 0);
  FullscreenEffect* fullscreen = GRAPHICS.GetFullscreenEffect();

  ScopedRenderTarget rtColor(DXGI_FORMAT_R16G16B16A16_FLOAT);

  {
    // plexus
    _ctx->SetRenderTarget(rtColor, GRAPHICS.GetDepthStencil(), &black);

    _cbPlexus.gs0.dim = vec4((float)rtColor._desc.width, (float)rtColor._desc.height, 0, 0);
    vec3 params = BLACKBOARD.GetVec3Var("plexus.lineParams");
    _cbPlexus.ps0.lineParams = vec4(params.x, params.y, params.z, 1);
    _cbPlexus.Set(_ctx, 0);

    ObjectHandle vb = _plexusLineBundle.objects._vb;
    vec3* vtx = _ctx->MapWriteDiscard<vec3>(vb);
    int numLines = CalcPlexusGrouping(
      vtx, _points.Data(), _points.Size(), _neighbours, _points.Size(), _settings.plexus);
    _ctx->Unmap(vb);
    _ctx->SetBundle(_plexusLineBundle);
    _ctx->Draw(numLines, 0);
  }

  ScopedRenderTarget rtGreets(DXGI_FORMAT_R16G16B16A16_FLOAT);
  {
    // greets
    _ctx->SetRenderTarget(rtGreets, GRAPHICS.GetDepthStencil(), &black);

    _cbGreets.Set(_ctx, 0);
    _ctx->SetBundle(_greetsBundle);
    GreetsBlock::GreetsData* data = _greetsBlock._data[_greetsBlock.curText];
    _ctx->DrawIndexed(_numGreetsCubes * 36, 0, 0);
  }

  {
    // composite
    _cbComposite.ps0.tonemap = vec2(_settings.tonemap.exposure, _settings.tonemap.min_white);
    _cbComposite.Set(_ctx, 0);

    ObjectHandle inputs[] = { rtColor, rtGreets };
    fullscreen->Execute(inputs,
      2,
      GRAPHICS.GetBackBuffer(),
      GRAPHICS.GetBackBufferDesc(),
      GRAPHICS.GetDepthStencil(),
      _compositeBundle.objects._ps,
      false,
      true,
      &Color(0.1f, 0.1f, 0.1f, 0));
  }


  return true;
}

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Plexus::RenderParameterSet()
{
  ImGui::SliderInt("text", &_greetsBlock.curText, 0, 4);

  ImGui::Separator();
  ImGui::SliderFloat("Exposure", &_settings.tonemap.exposure, 0.1f, 2.0f);
  ImGui::SliderFloat("Min White", &_settings.tonemap.min_white, 0.1f, 2.0f);
  ImGui::Separator();

  static bool showPlexusSettings = false;
  ImGui::Checkbox("Plexus config", &showPlexusSettings);
  if (showPlexusSettings)
  {
    bool recalc = false;
    bool recalcEdges = true;
    if (ImGui::SliderFloat("radius", &_settings.sphere.radius, 50, 2000))
    {
      recalc = true;
      recalcEdges = false;
    }

    if (ImGui::SliderFloat("noise-strength", &_settings.deform.noise_strength, -2500, 2500))
    {
      recalc = true;
      recalcEdges = false;
    }

    ImGui::SliderFloat("perlin-scale", &_settings.deform.perlin_scale, 0, 1000);

    recalc |= ImGui::SliderFloat("eps", &_settings.plexus.eps, 0.1f, 25.0f);
    recalc |= ImGui::SliderFloat("min-dist", &_settings.plexus.min_dist, 0.1f, 25.0f);
    recalc |= ImGui::SliderFloat("max-dist", &_settings.plexus.max_dist, 10.0, 150.0f);
    recalc |= ImGui::SliderInt("slices", &_settings.sphere.slices, 1, 50);
    recalc |= ImGui::SliderInt("stacks", &_settings.sphere.stacks, 1, 50);
    recalc |= ImGui::SliderInt("layers", &_settings.sphere.layers, 1, 50);
    recalc |= ImGui::SliderInt("num-nearest", &_settings.plexus.num_nearest, 1, 20);
    recalc |= ImGui::SliderInt("num-neighbours", &_settings.plexus.num_neighbours, 1, 100);

    if (ImGui::SliderFloat("blur-kernel", &_settings.deform.blur_kernel, 1, 250))
      GenRandomPoints(_settings.deform.blur_kernel);

    if (recalc)
    {
      CalcPoints(recalcEdges);
    }
  }


  if (ImGui::Button("Reset"))
    Reset();
}
#endif

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Plexus::SaveParameterSet(bool inc)
{
  _freeflyCamera.ToProtocol(&_settings.camera);
  SaveSettings(_settings);
}
#endif

//------------------------------------------------------------------------------
void Plexus::Reset()
{
//  _greetsBlock.Reset();
  _freeflyCamera._pos = vec3(0.f, 0.f, 0.f);
  _freeflyCamera._pitch = _freeflyCamera._yaw = _freeflyCamera._roll = 0.f;
}

//------------------------------------------------------------------------------
bool Plexus::Close()
{
  return true;
}

//------------------------------------------------------------------------------
BaseEffect* Plexus::Create(const char* name, const char* config, u32 id)
{
  return new Plexus(name, config, id);
}

//------------------------------------------------------------------------------
const char* Plexus::Name()
{
  return "plexus";
}

//------------------------------------------------------------------------------
void Plexus::Register()
{
  DEMO_ENGINE.RegisterFactory(Name(), Plexus::Create);
}
