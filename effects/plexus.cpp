#include "Plexus.hpp"
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
#include "../fullscreen_effect.hpp"
#include "../arena_allocator.hpp"
#include "../stop_watch.hpp"
#include "../perlin2d.hpp"

using namespace tano;
using namespace bristol;

extern "C" float stb_perlin_noise3(float x, float y, float z);

static int NOISE_WIDTH = 512;
static int NOISE_HEIGHT = 512;

#define WITH_TEXT 0

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
Plexus::Plexus(const string &name, u32 id)
  : BaseEffect(name, id)
{
#if WITH_IMGUI
  PROPERTIES.Register(Name(),
    bind(&Plexus::RenderParameterSet, this),
    bind(&Plexus::SaveParameterSet, this));

  PROPERTIES.SetActive(Name());
#endif
}

//------------------------------------------------------------------------------
Plexus::~Plexus()
{
}

SimpleAppendBuffer<V3, 1024> g_randomPoints;

//------------------------------------------------------------------------------
void GenRandomPoints(float kernelSize)
{
  V3* tmp = g_ScratchMemory.Alloc<V3>(g_randomPoints.Capacity());
  for (int i = 0; i < g_randomPoints.Capacity(); ++i)
  {
    V3 v(randf(-1.f, 1.f), randf(-1.f, 1.f), randf(-1.f, 1.f));
    v = Normalize(v);
    tmp[i] = v;
  }

  g_randomPoints.Resize(g_randomPoints.Capacity());

  BlurLine(tmp, g_randomPoints.Data(), g_randomPoints.Capacity(), kernelSize);
}

//------------------------------------------------------------------------------
bool Plexus::Init(const char* configFile)
{
  BEGIN_INIT_SEQUENCE();

  _configName = configFile;
  vector<char> buf;
  INIT_FATAL(RESOURCE_MANAGER.LoadFile(configFile, &buf));

  INIT(ParsePlexusSettings(InputBuffer(buf), &_settings));
  _camera._pitch = _settings.camera.pitch;
  _camera._yaw = _settings.camera.yaw;
  _camera._roll = _settings.camera.roll;
  _camera._pos = _settings.camera.pos;

  GenRandomPoints(_settings.sphere.blur_kernel);

  INIT(_cbPerFrame.Create());
  INIT(_cbBasic.Create());

  INIT(_pointBundle.Create(BundleOptions()
    .VertexShader("shaders/out/basic", "VsPos")
    .VertexFlags(VF_POS)
    .PixelShader("shaders/out/basic", "PsPos")
    .DynamicVb(128 * 1024, sizeof(V3))
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_POINTLIST)));

  INIT(_lineBundle.Create(BundleOptions()
    .VertexShader("shaders/out/basic", "VsPos")
    .VertexFlags(VF_POS)
    .PixelShader("shaders/out/basic", "PsPos")
    .DynamicVb(128 * 1024, sizeof(V3))
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST)));

  INIT(_line2Bundle.Create(BundleOptions()
    .VertexShader("shaders/out/plexus", "VsLines")
    .GeometryShader("shaders/out/plexus", "GsLines")
    .PixelShader("shaders/out/plexus", "PsLines")
    .VertexFlags(VF_POS)
    .RasterizerDesc(rasterizeDescCullNone)
    .BlendDesc(blendDescBlendOneOne)
    .DepthStencilDesc(depthDescDepthWriteDisabled)
    .DynamicVb(128 * 1024, sizeof(V3))
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST)));

  INIT(_textWriter.Init("gfx/text1.boba"));
  _textWriter.GenerateIndexedTris("neurotica efs", TextWriter::TextOutline, &_textVerts, &_textIndices);

#if WITH_TEXT
  CalcText();
#else
  CalcPoints(true);
#endif

  _perlinTexture = GRAPHICS.CreateTexture(NOISE_WIDTH, NOISE_HEIGHT, DXGI_FORMAT_R8G8B8A8_UNORM, nullptr);

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
void Plexus::UpdateNoise()
{
  static bool opened = false;
  static bool firstTime = true;
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

  ImGui::Separator();
  recalc |= ImGui::Checkbox("tonemap enabled", &_settings.tonemap.enabled);
  if (_settings.tonemap.enabled)
  {
    recalc |= ImGui::SliderFloat("black-point", &_settings.tonemap.black_point, 0, 1);
    recalc |= ImGui::SliderFloat("white-point", &_settings.tonemap.white_point, 0.5, 20);
    recalc |= ImGui::SliderFloat("cross-over", &_settings.tonemap.cross_over, 0, 20);
    recalc |= ImGui::SliderFloat("toe", &_settings.tonemap.toe, 0, 1);
    recalc |= ImGui::SliderFloat("shoulder", &_settings.tonemap.shoulder, 0, 1);
  }

  ImGui::Image((void*)&_perlinTexture, ImVec2((float)NOISE_WIDTH, (float)NOISE_HEIGHT));

  ImGui::End();

  firstTime = false;
  if (!recalc)
    return;

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
        pixels[i*NOISE_WIDTH+j] += v;
      }
    }

    scale *= _settings.noise.scale_factor;
    layerOpacity *= _settings.noise.opacity_factor;
  }

  const FullTonemapSettings& t = _settings.tonemap;
  u32* p = _ctx->MapWriteDiscard<u32>(_perlinTexture);
  for (int i = 0; i < NOISE_HEIGHT; ++i)
  {
    for (int j = 0; j < NOISE_WIDTH; ++j)
    {
      float f;
      if (_settings.tonemap.enabled)
      {
        f = 255 * Clamp(0.f, 1.f,
          ToneMap(pixels[i*NOISE_WIDTH + j], t.cross_over, t.black_point, t.white_point, t.toe, t.shoulder));
      }
      else
      {
        f = pixels[i*NOISE_WIDTH + j];
        float angle = 2 * XM_PI * f;
        float s = _settings.noise.turbulence;
        int xx = (int)(j + s * cos(angle));
        int yy = (int)(i + s * sin(angle));
        int x = IntMod(xx, NOISE_WIDTH);
        int y = IntMod(yy, NOISE_HEIGHT);
        f = 255 * Clamp(0.f, 1.f, pixels[y*NOISE_WIDTH + x]);
      }
      u32 val = (u32)f;
      p[i*NOISE_WIDTH+j] = (0xff000000) | (val << 16) | (val << 8) | (val << 0);
    }
  }
  _ctx->Unmap(_perlinTexture);
}

//------------------------------------------------------------------------------
void CalcNeighbours(int num, const vector<int>& tris, int* neighbours)
{
  unordered_map<int, vector<int>> tmp;

  int numTris = (int)tris.size() / 3;
  for (int i = 0; i < numTris; ++i)
  {
    int a = tris[i * 3 + 0];
    int b = tris[i * 3 + 1];
    int c = tris[i * 3 + 2];

    tmp[a].push_back(b);
    tmp[a].push_back(c);

    tmp[b].push_back(a);
    tmp[b].push_back(c);

    tmp[c].push_back(a);
    tmp[c].push_back(b);
  }

  for (int i = 0; i < num; ++i)
  {
    const vector<int>& n = tmp[i];
    int cnt = (u32)n.size();
    for (int j = 0; j < cnt; ++j)
    {
      neighbours[i*num+j] = n[j];
    }
    // terminate
    neighbours[i*num+cnt] = -1;
  }

}

//------------------------------------------------------------------------------
void Plexus::TextTest(const UpdateState& state)
{
  float perlinScale = _settings.sphere.perlin_scale;

  _points.Clear();
  int num = (u32)_textVerts.size();
  for (int i = 0; i < num; ++i)
  {
    V3 pt(_textVerts[i]);
    float s = fabs(1024 * stb_perlin_noise3(pt.x / perlinScale, pt.y / perlinScale, pt.z / perlinScale));
    V3 v = g_randomPoints[IntMod((int)s, 1024)];
    _points.Append(pt + _settings.sphere.noise_strength * v);
  }
}

//------------------------------------------------------------------------------
void Plexus::CalcText()
{
  _points.Clear();
  int num = (u32)_textVerts.size();
  for (int i = 0; i < num; ++i)
  {
    _points.Append(_textVerts[i]);
  }

  SAFE_ADELETE(_neighbours);
  _neighbours = new int[num*num];
  memset(_neighbours, 0xff, num*num*sizeof(int));
  CalcNeighbours(num, _textIndices, _neighbours);
}

//------------------------------------------------------------------------------
void Plexus::PointsTest(const UpdateState& state)
{
  _points.Clear();

  float radiusInc = _settings.sphere.radius / _settings.sphere.layers;
  float phiInc = 2 * XM_PI / _settings.sphere.slices;
  float thetaInc = XM_PI / (_settings.sphere.stacks > 1 ? _settings.sphere.stacks - 1 : 1);

  float r = _settings.sphere.radius;
  float perlinScale = _settings.sphere.perlin_scale;

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

        V3 pt = FromSpherical(r, phi, theta);

        float s = fabs(1024 * stb_perlin_noise3(pt.x / perlinScale, pt.y / perlinScale, pt.z / perlinScale));
        V3 v = g_randomPoints[IntMod((int)s, 1024)];
        pt = pt + _settings.sphere.noise_strength * v;

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

  for (int i = 0 ; i < layers; ++i)
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

        V3 pt = FromSpherical(r, phi, theta);
        float s = _settings.sphere.noise_strength * stb_perlin_noise3(pt.x, pt.y, pt.z);
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

          if (edgeUp) e[edgeIdx++] = IntMod(nodeIdx - 1, num);
          if (edgeDown) e[edgeIdx++] = IntMod(nodeIdx + 1, num);

          if (edgeOut) e[edgeIdx++] = IntMod(nodeIdx - (slices * stacks), num);
          if (edgeIn) e[edgeIdx++] = IntMod(nodeIdx - (slices * stacks), num);

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
    _neighbours = new int[num*num];

    u8* visited = g_ScratchMemory.Alloc<u8>(num);
    for (int i = 0; i < num; ++i)
    {
      memset(visited, 0, num);
      deque<int> frontier;
      frontier.push_front(i);
      int idx = 0;
      while (!frontier.empty())
      {
        int cur = frontier.front();
        _neighbours[i*num + idx] = cur;
        idx++;
        frontier.pop_front();
        visited[cur] = 1;

        for (int j = 0; j < 6; ++j)
        {
          int tmp = edges[cur].e[j];
          if (tmp == -1)
            break;
          if (!visited[tmp])
          {
            visited[tmp] = 1;
            frontier.push_back(tmp);
          }
        }
      }
    }
  }
}

//------------------------------------------------------------------------------
int Plexus::CalcLines(V3* vtx)
{
  static AvgStopWatch stopWatch;
  stopWatch.Start();

  int num = _points.Size();
  u8* connected = g_ScratchMemory.Alloc<u8>(num*num);
  memset(connected, 0, num*num);

  struct DistEntry { float dist; int idx; };
  DistEntry* dist = g_ScratchMemory.Alloc<DistEntry>(num);
  int* idx = g_ScratchMemory.Alloc<int>(num);
  u8* degree = g_ScratchMemory.Alloc<u8>(num);
  memset(degree, 0, num);

  V3* orgVtx = vtx;

  float minDist = _settings.sphere.min_dist;
  float maxDist = _settings.sphere.max_dist;
  int numNeighbours = _settings.sphere.num_neighbours;

  float eps = _settings.sphere.eps;

  auto fnSort = [&](int a, int b) { 
    float da = dist[a].dist;
    float db = dist[b].dist;
    float d = da - db;
    if (d < 0) d *= -1;
    if (d < eps)
       return degree[a] < degree[b];
    return da < db;
  };

  for (int i = 0; i < num; ++i)
    idx[i] = i;

  for (int i = 0; i < num; ++i)
  {
    int numValid = 0;
    for (int j = 0; j < numNeighbours; ++j)
    {
      int curIdx = _neighbours[i*num+j];
      if (curIdx == -1)
        break;

      float d = Distance(_points[i], _points[curIdx]);

      if (curIdx == i || d < minDist || d > maxDist || connected[i*num + curIdx] || connected[curIdx*num + i])
        continue;

      idx[numValid] = numValid;
      dist[numValid].dist = d;
      dist[numValid].idx = curIdx;
      numValid++;
    }

    sort(idx, idx+numValid, fnSort);

    int left = _settings.sphere.num_nearest;
    for (int j = 0; j < numValid; ++j)
    {
      int curIdx = dist[idx[j]].idx;

      vtx[0] = _points[i];
      vtx[1] = _points[curIdx];
      vtx += 2;

      connected[i*num+curIdx] = 1;
      degree[i]++;
      degree[curIdx]++;

      if (--left == 0)
        break;
    }
  }

  double avg = stopWatch.Stop();
  TANO.AddPerfCallback([=]() {
    ImGui::Text("Update time: %.3fms", 1000 * avg);
  });

  return (int)(vtx - orgVtx);
}

//------------------------------------------------------------------------------
bool Plexus::Update(const UpdateState& state)
{
  UpdateCameraMatrix(state);
#if WITH_TEXT
  TextTest(state);
#else
  PointsTest(state);
#endif
  return true;
}

//------------------------------------------------------------------------------
Vector4 Expand(const Vector3& v, float x)
{
  return Vector4(v.x, v.y, v.z, x);
}

//------------------------------------------------------------------------------
void Plexus::UpdateCameraMatrix(const UpdateState& state)
{
  _camera.Update(state);

  Matrix view = _camera._view;
  Matrix proj = _camera._proj;

  Matrix viewProj = view * proj;

  _cbPerFrame.world = Matrix::Identity();
  _cbPerFrame.view = view.Transpose();
  _cbPerFrame.proj = proj.Transpose();
  _cbPerFrame.viewProj = viewProj.Transpose();
  _cbPerFrame.cameraPos = _camera._pos;
  _cbPerFrame.cameraLookAt = _camera._target;
  _cbPerFrame.cameraUp = _camera._up;

  //static float angle = 0;
  //angle += state.numTicks / 100.0f;
  //Matrix mtx = Matrix::CreateRotationY(angle);
  Matrix mtx = Matrix::Identity();

  _cbBasic.world = mtx.Transpose();
  _cbBasic.view = view.Transpose();
  _cbBasic.proj = proj.Transpose();
  _cbBasic.viewProj = viewProj.Transpose();
  _cbBasic.cameraPos = Expand(_camera._pos, 0);
}

//------------------------------------------------------------------------------
bool Plexus::Render()
{
  rmt_ScopedCPUSample(Plexus_Render);

  static Color black(0, 0, 0, 0);

  _ctx->SetSwapChain(GRAPHICS.DefaultSwapChain(), black);

  RenderTargetDesc desc = GRAPHICS.GetBackBufferDesc();
  _cbBasic.dim = Vector4((float)desc.width, (float)desc.height, 0, 0);

  _ctx->SetConstantBuffer(_cbBasic, VertexShader | GeometryShader | PixelShader, 0);

  if (_renderPoints)
  {
    ObjectHandle vb = _pointBundle.objects._vb;
    V3* vtx = _ctx->MapWriteDiscard<V3>(vb);
    memcpy(vtx, _points.Data(), _points.DataSize());
    _ctx->Unmap(vb);
    _ctx->SetBundle(_pointBundle);
    _ctx->Draw(_points.Size(), 0);
  }
  else
  {
    ObjectHandle vb = _line2Bundle.objects._vb;
    V3* vtx = _ctx->MapWriteDiscard<V3>(vb);
    int numLines = CalcLines(vtx);
    _ctx->Unmap(vb);
    _ctx->SetBundle(_line2Bundle);
    _ctx->Draw(numLines, 0);
  }

  return true;
}

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Plexus::RenderParameterSet()
{
  ImGui::Checkbox("points-only", &_renderPoints);
  ImGui::Separator();

  bool recalc = false;
  bool recalcEdges = true;
  if (ImGui::SliderFloat("radius", &_settings.sphere.radius, 50, 2000))
  {
    recalc = true;
    recalcEdges = false;
  }

  if (ImGui::SliderFloat("noise-strength", &_settings.sphere.noise_strength, -2500, 2500))
  {
    recalc = true;
    recalcEdges = false;
  }

  ImGui::SliderFloat("perlin-scale", &_settings.sphere.perlin_scale, 0, 1000);

  recalc |= ImGui::SliderFloat("eps", &_settings.sphere.eps, 0.1f, 25.0f);
  recalc |= ImGui::SliderFloat("min-dist", &_settings.sphere.min_dist, 0.1f, 25.0f);
  recalc |= ImGui::SliderFloat("max-dist", &_settings.sphere.max_dist, 10.0, 150.0f);
  recalc |= ImGui::SliderInt("slices", &_settings.sphere.slices, 1, 50);
  recalc |= ImGui::SliderInt("stacks", &_settings.sphere.stacks, 1, 50);
  recalc |= ImGui::SliderInt("layers", &_settings.sphere.layers, 1, 50);
  recalc |= ImGui::SliderInt("num-nearest", &_settings.sphere.num_nearest, 1, 20);
  recalc |= ImGui::SliderInt("num-neighbours", &_settings.sphere.num_neighbours, 1, 100);

  if (ImGui::SliderFloat("blur-kernel", &_settings.sphere.blur_kernel, 1, 250))
    GenRandomPoints(_settings.sphere.blur_kernel);

  if (recalc)
  {
#if WITH_TEXT
    CalcText();
#else
    CalcPoints(recalcEdges);
#endif
  }

  if (ImGui::Button("Reset"))
    Reset();
}
#endif

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Plexus::SaveParameterSet()
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
void Plexus::Reset()
{
  _camera._pos = Vector3(0.f, 0.f, 0.f);
  _camera._pitch = _camera._yaw = _camera._roll = 0.f;
}

//------------------------------------------------------------------------------
bool Plexus::Close()
{
  return true;
}

//------------------------------------------------------------------------------
BaseEffect* Plexus::Create(const char* name, u32 id)
{
  return new Plexus(name, id);
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
