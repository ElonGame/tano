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
#include "../blackboard.hpp"

using namespace tano;
using namespace bristol;

extern "C" float stb_perlin_noise3(float x, float y, float z);

static int NOISE_WIDTH = 512;
static int NOISE_HEIGHT = 512;

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
bool Plexus::OnConfigChanged(const vector<char>& buf)
{
  return ParsePlexusSettings(InputBuffer(buf), &_settings);
}

//------------------------------------------------------------------------------
bool Plexus::Init()
{
  BEGIN_INIT_SEQUENCE();

  _camera.FromProtocol(_settings.camera);

  GenRandomPoints(_settings.deform.blur_kernel);

  INIT(_cbPlexus.Create());

  // clang-format off
  INIT(_pointBundle.Create(BundleOptions()
    .VertexShader("shaders/out/basic", "VsPos")
    .VertexFlags(VF_POS)
    .PixelShader("shaders/out/basic", "PsPos")
    .DynamicVb(128 * 1024, sizeof(V3))
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_POINTLIST)));

  INIT(_plexusLineBundle.Create(BundleOptions()
    .VertexShader("shaders/out/plexus", "VsLines")
    .GeometryShader("shaders/out/plexus", "GsLines")
    .PixelShader("shaders/out/plexus", "PsLines")
    .VertexFlags(VF_POS)
    .RasterizerDesc(rasterizeDescCullNone)
    .BlendDesc(blendDescBlendOneOne)
    .DepthStencilDesc(depthDescDepthWriteDisabled)
    .DynamicVb(128 * 1024, sizeof(V3))
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST)));
  // clang-format on

  CalcPoints(true);

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

        V3 pt = FromSpherical(r, phi, theta);

        float s = fabs(1024 * stb_perlin_noise3(pt.x / perlinScale, pt.y / perlinScale, pt.z / perlinScale));
        V3 v = g_randomPoints[IntMod((int)s, 1024)];
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

        V3 pt = FromSpherical(r, phi, theta);
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
      deque<int> frontier;
      frontier.push_front(i);
      int idx = 0;
      while (!frontier.empty())
      {
        int cur = frontier.front();
        _neighbours[i * num + idx] = cur;
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
bool Plexus::Update(const UpdateState& state)
{
  UpdateCameraMatrix(state);
  PointsTest(state);
  return true;
}

//------------------------------------------------------------------------------
bool Plexus::FixedUpdate(const FixedUpdateState& state)
{
  _camera.Update(state);
  return true;
}

//------------------------------------------------------------------------------
void Plexus::UpdateCameraMatrix(const UpdateState& state)
{
  Matrix view = _camera._view;
  Matrix proj = _camera._proj;

  Matrix viewProj = view * proj;

  _settings.deform.noise_strength =
      BLACKBOARD.GetFloatVar("plexus.strength", state.localTime.TotalMilliseconds() / 1000.f);
  CalcPoints(false);

  float rotXDiv = BLACKBOARD.GetFloatVar("plexus.rotXDivisor");
  float rotYDiv = BLACKBOARD.GetFloatVar("plexus.rotXDivisor");
  static float angle = 0;
  angle += state.delta.TotalMilliseconds();
  Matrix mtx = Matrix::CreateRotationX(angle / rotXDiv) * Matrix::CreateRotationY(angle / rotYDiv);
  // Matrix mtx = Matrix::Identity();
  _cbPlexus.gs0.world = mtx.Transpose();
  _cbPlexus.gs0.viewProj = viewProj.Transpose();
  _cbPlexus.gs0.cameraPos = _camera._pos;
}

//------------------------------------------------------------------------------
bool Plexus::Render()
{
  rmt_ScopedCPUSample(Plexus_Render);

  static Color black(0, 0, 0, 0);

  _ctx->SetSwapChain(GRAPHICS.DefaultSwapChain(), black);

  RenderTargetDesc desc = GRAPHICS.GetBackBufferDesc();
  _cbPlexus.gs0.dim = Vector4((float)desc.width, (float)desc.height, 0, 0);
  V3 params = BLACKBOARD.GetVec3Var("plexus.lineParams");
  _cbPlexus.ps0.lineParams = Vector4(params.x, params.y, params.z, 1);
  _cbPlexus.Set(_ctx, 0);

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
    ObjectHandle vb = _plexusLineBundle.objects._vb;
    V3* vtx = _ctx->MapWriteDiscard<V3>(vb);
    int numLines = CalcPlexusGrouping(
        vtx, _points.Data(), _points.Size(), _neighbours, _points.Size(), _settings.plexus);
    _ctx->Unmap(vb);
    _ctx->SetBundle(_plexusLineBundle);
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

  if (ImGui::Button("Reset"))
    Reset();
}
#endif

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Plexus::SaveParameterSet(bool inc)
{
  _camera.ToProtocol(&_settings.camera);
  SaveSettings(_settings);
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
