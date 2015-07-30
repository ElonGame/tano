#include "tunnel.hpp"
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
#include "../fullscreen_effect.hpp"
#include "../mesh_utils.hpp"
#include "../mesh_loader.hpp"

using namespace tano;
using namespace bristol;

static float Z_SPACING = 50;
static int CAMERA_STEP = 10;
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
    //d->DiffuseUpdate(state);
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
Tunnel::Tunnel(const string& name, const string& config, u32 id) : BaseEffect(name, config, id)
{
#if WITH_IMGUI
  PROPERTIES.Register(Name(), bind(&Tunnel::RenderParameterSet, this), bind(&Tunnel::SaveParameterSet, this));

  PROPERTIES.SetActive(Name());
#endif

  //_neighbours = new int[16*1024*16];
}

//------------------------------------------------------------------------------
Tunnel::~Tunnel()
{
  // SAFE_ADELETE(_neighbours);
}

//------------------------------------------------------------------------------
bool Tunnel::OnConfigChanged(const vector<char>& buf)
{
  return ParseTunnelSettings(InputBuffer(buf), &_settings);
}

//------------------------------------------------------------------------------
bool Tunnel::Init()
{
  BEGIN_INIT_SEQUENCE();

  {
    // create spline
    int numPoints = 10000;
    float s = 20;

    V3 cur(0, 0, 0);
    vector<V3> controlPoints;
    vector<V3> cameraControlPoints;

    for (int i = 0; i < numPoints; ++i)
    {
      float xOfs = randf(-s, +s);
      float yOfs = randf(-s, +s);

      controlPoints.push_back(cur);
      cur += V3(xOfs, yOfs, Z_SPACING);

      if ((i % CAMERA_STEP) == 0)
        cameraControlPoints.push_back(cur);
    }

    _spline.Create(controlPoints.data(), (int)controlPoints.size());
    _cameraSpline.Create(cameraControlPoints.data(), (int)cameraControlPoints.size());
  }

  // clang-format off
  INIT(_linesBundle.Create(BundleOptions()
    .VertexShader("shaders/out/tunnel.lines", "VsTunnelLines")
    .GeometryShader("shaders/out/tunnel.lines", "GsTunnelLines")
    .PixelShader("shaders/out/tunnel.lines", "PsTunnelLines")
    .VertexFlags(VF_POS)
    .RasterizerDesc(rasterizeDescCullNone)
    .BlendDesc(blendDescBlendOneOne)
    .DepthStencilDesc(depthDescDepthDisabled)
    .DynamicVb(128 * 1024, sizeof(V3))
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST)));

  INIT(_compositeBundle.Create(BundleOptions()
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/tunnel.composite", "PsComposite")));

  INIT(_meshBundle.Create(BundleOptions()
    .VertexShader("shaders/out/tunnel.mesh", "VsMesh")
    .InputElement(CD3D11_INPUT_ELEMENT_DESC("SV_POSITION", DXGI_FORMAT_R32G32B32_FLOAT))
    .InputElement(CD3D11_INPUT_ELEMENT_DESC("NORMAL", DXGI_FORMAT_R32G32B32_FLOAT))
    .InputElement(CD3D11_INPUT_ELEMENT_DESC("TEXCOORD", DXGI_FORMAT_R32G32_FLOAT))
    .PixelShader("shaders/out/tunnel.mesh", "PsMesh")));

  INIT(_greetsBundle.Create(BundleOptions()
    .InputElement(CD3D11_INPUT_ELEMENT_DESC("POSITION", DXGI_FORMAT_R32G32B32_FLOAT))
    .InputElement(CD3D11_INPUT_ELEMENT_DESC("NORMAL", DXGI_FORMAT_R32G32B32_FLOAT))
    .VertexShader("shaders/out/tunnel.greets", "VsGreets")
    .PixelShader("shaders/out/tunnel.greets", "PsGreets")
    .DynamicVb(MAX_GREETS_HEIGHT*MAX_GREETS_WIDTH * 24 * 2, 2 * sizeof(V3))
    .StaticIb(GenerateCubeIndicesFaceted(MAX_GREETS_HEIGHT*MAX_GREETS_WIDTH)))
  );
  // clang-format on

  INIT(_cbLines.Create());
  INIT(_cbComposite.Create());
  INIT(_cbMesh.Create());
  INIT(_cbGreets.Create());

  MeshLoader loader;
  INIT(loader.Load("gfx/newblob1.boba"));
  INIT(CreateScene(loader, SceneOptions(), &_scene));

  INIT(_greetsBlock.Init());

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
void Tunnel::UpdateGreets(const UpdateState& state)
{
  _greetsBlock.Update(state);
  _numGreetsCubes = 0;

  ObjectHandle handle = _greetsBundle.objects._vb;
  V3* verts = _ctx->MapWriteDiscard<V3>(handle);

  GreetsBlock::GreetsData* data = _greetsBlock._data[_greetsBlock.curText];
  int w = data->width;
  int h = data->height;
  float fw = (float)w;
  float fh = (float)h;

  float s = 10;
  V3 pos(-fw/2*s+s/2, fh/2*s-s/2, 300);
  float orgX = pos.x;
  for (int i = 0; i < h; ++i)
  {
    pos.x = orgX;
    for (int j = 0; j < w; ++j)
    {
      //float ss = s * Clamp(0.f, 1.f, (float)data->targetParticleCount[i*w + j] / 10);
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
bool Tunnel::Update(const UpdateState& state)
{
  V3 pos(V3(_camera._pos));

  V2 cameraParams = BLACKBOARD.GetVec2Var("tunnel.cameraParams");
  _tunnelVerts.Clear();

  // NormalUpdate(state);
  PlexusUpdate(state);
  UpdateCameraMatrix(state);
  UpdateGreets(state);
  return true;
}

//------------------------------------------------------------------------------
void Tunnel::PlexusUpdate(const UpdateState& state)
{
  float radius = BLACKBOARD.GetFloatVar("tunnel.radius", state.localTime.TotalSecondsAsFloat());
  int numSegments = BLACKBOARD.GetIntVar("tunnel.segments");

  V3* points = g_ScratchMemory.Alloc<V3>(16 * 1024);
  int MAX_N = 16;
  int* neighbours = g_ScratchMemory.Alloc<int>(16 * 1024 * MAX_N);

  float START_OFS = -2;
  int tmp = (int)((_camera._pos.z / Z_SPACING) / Z_SPACING * Z_SPACING);
  float distClamped = (float)tmp;
  int depth = 50;

  int num = depth * numSegments;

  int idx = 0;
  for (int j = 0; j < depth; ++j)
  {
    V3 pos = _spline.Interpolate(distClamped + (float)j + START_OFS);

    float angle = 0;
    float angleInc = 2 * XM_PI / numSegments;
    for (int i = 0; i < numSegments; ++i)
    {
      V3 p = pos + radius * V3(cosf(angle), sinf(angle), 0);
      points[idx] = p;
      // add neighbours
      int n = 0;

      auto dn = [=](int n) { int tmp = i - n; return tmp < 0 ? tmp + numSegments : tmp; };
      auto up = [=](int n) { int tmp = i + n; return tmp % numSegments; };
      int dn1 = dn(1), dn2 = dn(2);
      int up1 = up(1), up2 = up(2);

      neighbours[idx * MAX_N + n++] = (j * numSegments) + dn1;
      neighbours[idx * MAX_N + n++] = (j * numSegments) + up1;

      if (j > 0)
      {
        neighbours[idx * MAX_N + n++] = ((j - 1) * numSegments) + i;
        neighbours[idx * MAX_N + n++] = ((j - 1) * numSegments) + dn1;
        //neighbours[idx * MAX_N + n++] = ((j - 1) * numSegments) + dn2;
        neighbours[idx * MAX_N + n++] = ((j - 1) * numSegments) + up1;
        //neighbours[idx * MAX_N + n++] = ((j - 1) * numSegments) + up2;
      }

      if (j < depth - 1)
      {
        neighbours[idx * MAX_N + n++] = ((j + 1) * numSegments) + i;
        neighbours[idx * MAX_N + n++] = ((j + 1) * numSegments) + dn1;
        //neighbours[idx * MAX_N + n++] = ((j + 1) * numSegments) + dn2;
        neighbours[idx * MAX_N + n++] = ((j + 1) * numSegments) + up1;
        //neighbours[idx * MAX_N + n++] = ((j + 1) * numSegments) + up2;
      }

      neighbours[idx * MAX_N + n] = -1;

      idx++;
      angle += angleInc;
    }
  }

  int numVerts = CalcPlexusGrouping(_tunnelVerts.Data(), points, idx, neighbours, MAX_N, _settings.plexus);
  _tunnelVerts.Resize(numVerts);
}

//------------------------------------------------------------------------------
void Tunnel::NormalUpdate(const UpdateState& state)
{
  float radius = BLACKBOARD.GetFloatVar("tunnel.radius", state.localTime.TotalSecondsAsFloat());
  int numSegments = BLACKBOARD.GetIntVar("tunnel.segments");

  SimpleAppendBuffer<V3, 512> ring1, ring2;
  SimpleAppendBuffer<V3, 512>* rings[] = {&ring1, &ring2};

  float START_OFS = -2;
  int tmp = (int)((_camera._pos.z / Z_SPACING) / Z_SPACING * Z_SPACING);
  float distClamped = (float)tmp;
  // create the first 2 rings
  for (int j = 0; j < 2; ++j)
  {
    V3 pos = _spline.Interpolate(distClamped + (float)j + START_OFS);
    float angle = 0;
    float angleInc = 2 * XM_PI / numSegments;
    V3 prev = pos + radius * V3(cosf(angle), sinf(angle), 0);
    for (int i = 0; i < numSegments; ++i)
    {
      angle += angleInc;
      V3 p = pos + radius * V3(cosf(angle), sinf(angle), 0);
      rings[j]->Append(prev);
      rings[j]->Append(p);
      prev = p;
    }
  }

  for (int j = 0; j < 50; ++j)
  {
    // copy out the first ring, add lines to the second, fill the first, and
    // flip the buffers

    auto& ring0 = *rings[0];
    auto& ring1 = *rings[1];
    for (int i = 0; i < numSegments * 2; ++i)
    {
      _tunnelVerts.Append(ring0[i]);
    }

    for (int i = 0; i < numSegments * 2; i += 2)
    {
      _tunnelVerts.Append(ring0[i]);
      _tunnelVerts.Append(ring1[i]);
    }

    ring0.Clear();
    V3 pos = _spline.Interpolate(distClamped + (float)j + START_OFS);

    float angle = 0;
    float angleInc = 2 * XM_PI / numSegments;
    V3 prev = pos + radius * V3(cosf(angle), sinf(angle), 0);
    for (int i = 0; i < numSegments; ++i)
    {
      angle += angleInc;
      V3 p = pos + radius * V3(cosf(angle), sinf(angle), 0);
      ring0.Append(prev);
      ring0.Append(p);
      prev = p;
    }

    swap(rings[0], rings[1]);
  }
}

//------------------------------------------------------------------------------
bool Tunnel::FixedUpdate(const FixedUpdateState& state)
{
  // Update how far along the spline we've travelled
  float t = state.localTime.TotalSecondsAsFloat();
  float speed = BLACKBOARD.GetFloatVar("tunnel.speed", t);
  float dirScale = BLACKBOARD.GetFloatVar("tunnel.dirScale");
  _dist += state.delta * speed;

  V3 pos = _cameraSpline.Interpolate(_dist / CAMERA_STEP);

  static bool useFreeFly = true;
  if (!useFreeFly)
  {
    _camera._pos = ToVector3(pos) + Vector3(5 * sinf(state.localTime.TotalSecondsAsFloat()),
      5 * cosf(state.localTime.TotalSecondsAsFloat()),
      0);

    _camera._dir = Vector3(sinf(state.localTime.TotalSecondsAsFloat()) * dirScale,
      cosf(state.localTime.TotalSecondsAsFloat()) * dirScale,
      1);
  }

  if (g_KeyUpTrigger.IsTriggered('1'))
    useFreeFly = !useFreeFly;

  //_camera.SetFollowTarget(ToVector3(pos));
  _camera.Update(state);
  return true;
}

//------------------------------------------------------------------------------
void Tunnel::UpdateCameraMatrix(const UpdateState& state)
{
  Matrix view = _camera._view;
  Matrix proj = _camera._proj;

  Matrix viewProj = view * proj;
  _cbLines.gs0.world = Matrix::Identity();
  _cbLines.gs0.viewProj = viewProj.Transpose();
  _cbLines.gs0.cameraPos = _camera._pos;

  _cbMesh.vs0.viewProj = viewProj.Transpose();

  _cbGreets.vs0.viewProj = viewProj.Transpose();
  _cbGreets.vs0.objWorld = Matrix::Identity();
  _cbGreets.vs0.camPos = _camera._pos;
}

//------------------------------------------------------------------------------
bool Tunnel::Render()
{
  rmt_ScopedCPUSample(Tunnel_Render);

  static Color black(0, 0, 0, 0);
  FullscreenEffect* fullscreen = GRAPHICS.GetFullscreenEffect();

  _ctx->SetSwapChain(GRAPHICS.DefaultSwapChain(), black);

  ScopedRenderTargetFull rtColor(
      DXGI_FORMAT_R16G16B16A16_FLOAT, BufferFlag::CreateSrv, BufferFlag::CreateSrv);
  _ctx->SetRenderTarget(rtColor._rtHandle, rtColor._dsHandle, &black);

  {
    // greets
    _cbGreets.Set(_ctx, 0);
    _ctx->SetBundle(_greetsBundle);
    GreetsBlock::GreetsData* data = _greetsBlock._data[_greetsBlock.curText];
    //_ctx->DrawIndexed(data->width*data->height * 6, 0, 0);
    _ctx->DrawIndexed(_numGreetsCubes * 36, 0, 0);
  }

#if 0
  {
    // tunnel
    _cbLines.gs0.dim = Vector4((float)rtColor._desc.width, (float)rtColor._desc.height, 0, 0);
    V3 params = BLACKBOARD.GetVec3Var("tunnel.lineParams");
    _cbLines.ps0.lineParams = Vector4(params.x, params.y, params.z, 1);
    _cbLines.Set(_ctx, 0);

    ObjectHandle h = _linesBundle.objects._vb;
    V3* verts = _ctx->MapWriteDiscard<V3>(h);
    memcpy(verts, _tunnelVerts.Data(), _tunnelVerts.DataSize());
    _ctx->Unmap(h);

    int numVerts = _tunnelVerts.Size();
    _ctx->SetBundle(_linesBundle);
    _ctx->Draw(numVerts, 0);
  }

  {
    // mesh
    _cbMesh.Set(_ctx, 0);
    _ctx->SetBundle(_meshBundle);

    for (scene::MeshBuffer* buf : _scene.meshBuffers)
    {
      _ctx->SetVertexBuffer(buf->vb);
      _ctx->SetIndexBuffer(buf->ib);

      for (scene::Mesh* mesh : buf->meshes)
      {
        Matrix mtx = mesh->mtxLocal;
        if (mesh->parentPtr)
        {
          mtx = mtx * mesh->parentPtr->mtxLocal;
        }

        _cbMesh.vs1.objWorld = mtx.Transpose();
        _cbMesh.Set(_ctx, 1);
        _ctx->DrawIndexed(mesh->indexCount, mesh->startIndexLocation, mesh->baseVertexLocation);
      }
    }
  }
#endif
  {
    // composite
    _cbComposite.ps0.tonemap = Vector2(_settings.tonemap.exposure, _settings.tonemap.min_white);
    _cbComposite.Set(_ctx, 0);

    ObjectHandle inputs[] = {rtColor, rtColor._dsHandle};
    fullscreen->Execute(inputs,
        2,
        GRAPHICS.GetBackBuffer(),
        GRAPHICS.GetBackBufferDesc(),
        GRAPHICS.GetDepthStencil(),
        _compositeBundle.objects._ps,
        false);
  }

  return true;
}

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Tunnel::RenderParameterSet()
{
  ImGui::SliderInt("text", &_greetsBlock.curText, 0, 4);

  ImGui::SliderFloat("Exposure", &_settings.tonemap.exposure, 0.1f, 20.0f);
  ImGui::SliderFloat("Min White", &_settings.tonemap.min_white, 0.1f, 20.0f);

  ImGui::SliderFloat("eps", &_settings.plexus.eps, 0.1f, 25.0f);
  ImGui::SliderFloat("min-dist", &_settings.plexus.min_dist, 0.1f, 25.0f);
  ImGui::SliderFloat("max-dist", &_settings.plexus.max_dist, 10.0, 150.0f);
  ImGui::SliderInt("num-nearest", &_settings.plexus.num_nearest, 1, 20);
  ImGui::SliderInt("num-neighbours", &_settings.plexus.num_neighbours, 1, 100);

  if (ImGui::Button("Reset"))
    Reset();
}
#endif

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Tunnel::SaveParameterSet()
{
  SaveSettings(_settings);
}
#endif

//------------------------------------------------------------------------------
void Tunnel::Reset()
{
  _camera._pos = Vector3(0.f, 0.f, 0.f);
  //_camera._pitch = _camera._yaw = _camera._roll = 0.f;

  _greetsBlock.Init();
}

//------------------------------------------------------------------------------
bool Tunnel::Close()
{
  return true;
}

//------------------------------------------------------------------------------
BaseEffect* Tunnel::Create(const char* name, const char* config, u32 id)
{
  return new Tunnel(name, config, id);
}

//------------------------------------------------------------------------------
const char* Tunnel::Name()
{
  return "tunnel";
}

//------------------------------------------------------------------------------
void Tunnel::Register()
{
  DEMO_ENGINE.RegisterFactory(Name(), Tunnel::Create);
}
