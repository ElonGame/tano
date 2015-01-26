#include "particle_tunnel.hpp"
#include "../tano.hpp"
#include "../graphics.hpp"
#include "../deferred_context.hpp"
#include "../demo_engine.hpp"
#include "../resource_manager.hpp"
#include "../init_sequence.hpp"
#include "../generated/demo.parse.hpp"
#include "../generated/input_buffer.hpp"
#include "../generated/output_buffer.hpp"
#include "../mesh_loader.hpp"

using namespace tano;
using namespace bristol;

//------------------------------------------------------------------------------
void ParticleTunnel::Particles::Create(int num)
{
  numParticles = num;
  x = new float[numParticles];
  y = new float[numParticles];
  z = new float[numParticles];

  vx = new float[numParticles];
  vy = new float[numParticles];
  vz = new float[numParticles];

  lifetime = new Lifetime[numParticles];
  scale = new float[numParticles];
  deadParticles = new int[numParticles];

  float s = 100;
  for (int i = 0; i < numParticles; ++i)
  {
    CreateParticle(i, s);
  }

  for (int i = 0; i < 1000; ++i)
  {
    Update(0.01f);
  }
}

//------------------------------------------------------------------------------
void ParticleTunnel::Particles::CreateParticle(int idx, float s)
{
  x[idx] = randf(-s, s);
  y[idx] = randf(-s, s);
  z[idx] = randf(1500.f, 2000.f);

  vx[idx] = randf(-s, s);
  vy[idx] = randf(-s, s);
  vz[idx] = -randf(10.f, 200.f);

  int ll = randf(2000, 3000);
  lifetime[idx].left = ll;
  lifetime[idx].total = ll;
  scale[idx] = randf(1.f, 5.f);
}

//------------------------------------------------------------------------------
void ParticleTunnel::Particles::Destroy()
{
  SAFE_ADELETE(x);  SAFE_ADELETE(y);  SAFE_ADELETE(z);
  SAFE_ADELETE(vx); SAFE_ADELETE(vy); SAFE_ADELETE(vz);
  SAFE_ADELETE(lifetime);
  SAFE_ADELETE(scale);
  SAFE_ADELETE(deadParticles);
}

//------------------------------------------------------------------------------
void ParticleTunnel::Particles::Update(float dt)
{
  float* xx = x;
  float* yy = y;
  float* zz = z;

  float *vvx = vx;
  float *vvy = vy;
  float *vvz = vz;

  Lifetime* ll = lifetime;

  int numDead = 0;
  int* dead = deadParticles;

  float s = 100;
  for (int i = 0, e = numParticles; i < e; ++i)
  {
    *xx += dt * *vvx;
    *yy += dt * *vvy;
    *zz += dt * *vvz;
    --ll->left;

    if (*zz < -500)
    {
      dead[numDead++] = i;
    }

    xx++; yy++; zz++;
    vvx++; vvy++; vvz++;
    ll++;
  }

  for (int i = 0; i < numDead; ++i)
  {
    CreateParticle(deadParticles[i], s);
  }
}

//------------------------------------------------------------------------------
void ParticleTunnel::TextParticles::Create(const vector<Vector3>& verts, float targetTime)
{
  numParticles = (int)verts.size();
  if (!x)
  {
    x = new float[numParticles];
    y = new float[numParticles];
    z = new float[numParticles];

    vx = new float[numParticles];
    vy = new float[numParticles];
    vz = new float[numParticles];
  }

  float dist = randf(settings.text_min_dist, settings.text_max_dist);

  // project each vertex outwards along the hemisphere in the z-plane
  for (int i = 0; i < numParticles; ++i)
  {
    Vector3 dir(randf(-1.f, 1.f), randf(-1.f, 1.f), randf(0.f, 1.f));
    dir.Normalize();

    vx[i] = dist * dir.x / targetTime;
    vy[i] = dist * dir.y / targetTime;
    vz[i] = dist * dir.z / targetTime;

    x[i] = verts[i].x - dir.x * dist;
    y[i] = verts[i].y - dir.y * dist;
    z[i] = verts[i].z - dir.z * dist;
  }

  selectedTris.clear();
  selectedTris.reserve(verts.size());
  for (int i = 0; i < verts.size(); ++i)
  {
    float p = randf(0.f, 1.f);
    if (p > settings.text_triangle_prob)
      selectedTris.push_back(i);
  }

}

//------------------------------------------------------------------------------
void ParticleTunnel::TextParticles::Destroy()
{
  SAFE_ADELETE(x);  SAFE_ADELETE(y);  SAFE_ADELETE(z);
  SAFE_ADELETE(vx); SAFE_ADELETE(vy); SAFE_ADELETE(vz);
}

//------------------------------------------------------------------------------
void ParticleTunnel::TextParticles::Update(float dt)
{
  float* xx = x;
  float* yy = y;
  float* zz = z;

  float *vvx = vx;
  float *vvy = vy;
  float *vvz = vz;

  for (int i = 0, e = numParticles; i < e; ++i)
  {
    xx[i] += dt * vvx[i];
    yy[i] += dt * vvy[i];
    zz[i] += dt * vvz[i];
  }
}

//------------------------------------------------------------------------------
ParticleTunnel::ParticleTunnel(const string &name, u32 id)
  : Effect(name, id)
  , _textParticles(_settings)
{
  PROPERTIES.Register("particle tunnel", 
    bind(&ParticleTunnel::RenderParameterSet, this),
    bind(&ParticleTunnel::SaveParameterSet, this));
}

//------------------------------------------------------------------------------
ParticleTunnel::~ParticleTunnel()
{
  _particles.Destroy();
}

//------------------------------------------------------------------------------
bool ParticleTunnel::Init(const char* configFile)
{
  BEGIN_INIT_SEQUENCE();

  _configName = configFile;
  vector<char> buf;
  if (!RESOURCE_MANAGER.LoadFile(configFile, &buf))
    return false;

  INIT(ParseParticleTunnelSettings(InputBuffer(buf), &_settings));

  // Background state setup
  INIT(_backgroundGpuObjects.LoadShadersFromFile("shaders/particle_tunnel", "VsQuad", nullptr, "PsBackground"));
  INIT(_backgroundState.Create());

  // Particle state setup
  INIT_RESOURCE(_particleTexture, RESOURCE_MANAGER.LoadTexture(_settings.texture.c_str()));
  INIT(_particleGpuObjects.LoadShadersFromFile(
    "shaders/particle_tunnel", "VsParticle", nullptr, "PsParticle", VertexFlags::VF_POS | VertexFlags::VF_TEX3_0));
  INIT(_particleGpuObjects.CreateDynamicVb(sizeof(PosTex3) * 6 * _settings.num_particles, sizeof(PosTex3)));

  {
    CD3D11_RASTERIZER_DESC rssDesc = CD3D11_RASTERIZER_DESC(CD3D11_DEFAULT());
    rssDesc.CullMode = D3D11_CULL_NONE;

    CD3D11_DEPTH_STENCIL_DESC dsDesc = CD3D11_DEPTH_STENCIL_DESC(CD3D11_DEFAULT());
    dsDesc.DepthEnable = FALSE;

    CD3D11_BLEND_DESC blendDesc = CD3D11_BLEND_DESC(CD3D11_DEFAULT());

    // pre-multiplied alpha
    D3D11_RENDER_TARGET_BLEND_DESC& b = blendDesc.RenderTarget[0];
    b.BlendEnable = TRUE;
    b.DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
    b.DestBlendAlpha = D3D11_BLEND_INV_SRC_ALPHA;
    INIT(_particleState.Create(&dsDesc, &blendDesc, &rssDesc));
  }

  INIT_RESOURCE(_particleSamplerState, GRAPHICS.CreateSamplerState(CD3D11_SAMPLER_DESC(CD3D11_DEFAULT())));

  _particles.Create(_settings.num_particles);

  // Composite state setup
  INIT(_compositeGpuObjects.LoadShadersFromFile("shaders/particle_tunnel", "VsQuad", nullptr, "PsComposite"));
  INIT(_compositeState.Create());

  // Text setup
  INIT(_textWriter.Init("gfx/text1.boba"));
  _textWriter.GenerateTris("neurotica efs", &_neuroticaTris);
  _textParticles.Create(_neuroticaTris, 5.f);
  {
    CD3D11_RASTERIZER_DESC rssDesc = CD3D11_RASTERIZER_DESC(CD3D11_DEFAULT());
    rssDesc.CullMode = D3D11_CULL_NONE;

    CD3D11_DEPTH_STENCIL_DESC dsDesc = CD3D11_DEPTH_STENCIL_DESC(CD3D11_DEFAULT());
    dsDesc.DepthEnable = FALSE;

    CD3D11_BLEND_DESC blendDesc = CD3D11_BLEND_DESC(CD3D11_DEFAULT());

    // pre-multiplied alpha
    D3D11_RENDER_TARGET_BLEND_DESC& b = blendDesc.RenderTarget[0];
    b.BlendEnable = TRUE;
    b.DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
    b.DestBlendAlpha = D3D11_BLEND_INV_SRC_ALPHA;

    INIT(_textState.Create(&dsDesc, &blendDesc, &rssDesc));
  }
  INIT(_textGpuObjects.CreateDynamicVb((u32)_neuroticaTris.size() * sizeof(Vector3), sizeof(Vector3)));
  INIT(_textGpuObjects.LoadShadersFromFile("shaders/particle_tunnel", "VsText", nullptr, "PsText", VertexFlags::VF_POS));

  {
    CD3D11_RASTERIZER_DESC rssDesc = CD3D11_RASTERIZER_DESC(CD3D11_DEFAULT());
    rssDesc.CullMode = D3D11_CULL_NONE;

    CD3D11_DEPTH_STENCIL_DESC dsDesc = CD3D11_DEPTH_STENCIL_DESC(CD3D11_DEFAULT());
    dsDesc.DepthEnable = FALSE;

    CD3D11_BLEND_DESC blendDesc = CD3D11_BLEND_DESC(CD3D11_DEFAULT());

    // pre-multiplied alpha
    D3D11_RENDER_TARGET_BLEND_DESC& b = blendDesc.RenderTarget[0];
    b.BlendEnable = TRUE;
    b.DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
    b.DestBlendAlpha = D3D11_BLEND_INV_SRC_ALPHA;

    INIT(_linesState.Create(&dsDesc, &blendDesc, &rssDesc));
  }
  INIT(_linesGpuObjects.CreateDynamicVb((u32)_neuroticaTris.size() * sizeof(Vector3) * 3 * 2, sizeof(Vector3)));
  INIT(_linesGpuObjects.LoadShadersFromFile("shaders/particle_tunnel", "VsLines", "GsLines", "PsLines", VertexFlags::VF_POS));
  _linesGpuObjects._topology = D3D11_PRIMITIVE_TOPOLOGY_LINELIST;

  // Generic setup
  INIT(_cbPerFrame.Create());
  _cbPerFrame.world = Matrix::Identity();
  Matrix view = Matrix::CreateLookAt(Vector3(0, 0, -500), Vector3(0, 0, 0), Vector3(0, 1, 0));
  Matrix proj = Matrix::CreatePerspectiveFieldOfView(XMConvertToRadians(45), 16/10.f, 0.1f, 2000.f);
  Matrix viewProj = view * proj;
  _cbPerFrame.viewProj = viewProj.Transpose();
  _cbPerFrame.tint = _settings.tint;
  _cbPerFrame.inner = _settings.inner_color;
  _cbPerFrame.outer = _settings.outer_color;

  int w, h;
  GRAPHICS.GetBackBufferSize(&w, &h);
  _cbPerFrame.dim.x = (float)w;
  _cbPerFrame.dim.y = (float)h;

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool ParticleTunnel::Update(const UpdateState& state)
{
  rmt_ScopedCPUSample(ParticleTunnel_Update);

  if (state.numTicks == 0)
    return true;

  float dt = 1.f / state.frequency;
  {
    rmt_ScopedCPUSample(Particles_Update);
    for (int tick = 0; tick < state.numTicks; ++tick)
    {
      _particles.Update(dt);
    }
  }

  {
    rmt_ScopedCPUSample(TextParticles_Update);
    for (int tick = 0; tick < state.numTicks; ++tick)
    {
      _textParticles.Update(dt);
    }
  }

  {
    // Blit text
    rmt_ScopedCPUSample(TextParticle_Map);

    float* xx = _textParticles.x;
    float* yy = _textParticles.y;
    float* zz = _textParticles.z;

    Vector3* vtx = _ctx->MapWriteDiscard<Vector3>(_textGpuObjects._vb);

    for (int i = 0, e = (int)_textParticles.selectedTris.size(); i < e; ++i)
    {
      int triIdx = _textParticles.selectedTris[i];
      for (int j = 0; j < 3; ++j)
      {
        vtx->x = xx[3 * triIdx + j];
        vtx->y = yy[3 * triIdx + j];
        vtx->z = zz[3 * triIdx + j];

        vtx++;
      }
    }
    _ctx->Unmap(_textGpuObjects._vb);
  }

  {
    // Blit lines

    float* xx = _textParticles.x;
    float* yy = _textParticles.y;
    float* zz = _textParticles.z;

    Vector3* vtx = _ctx->MapWriteDiscard<Vector3>(_linesGpuObjects._vb);
    for (int i = 0, e = (int)_textParticles.selectedTris.size(); i < e; ++i)
    {
      int triIdx = _textParticles.selectedTris[i];
      for (int j = 0; j < 3; ++j)
      {
        int idx0 = 3 * triIdx + j;
        int idx1 = 3 * triIdx + (j + 1) % 3;

        vtx->x = xx[idx0];
        vtx->y = yy[idx0];
        vtx->z = zz[idx0];
        vtx++;

        vtx->x = xx[idx1];
        vtx->y = yy[idx1];
        vtx->z = zz[idx1];
        vtx++;
      }
    }

    _ctx->Unmap(_linesGpuObjects._vb);
  }

  // TODO: all this is pretty stupid. we should store as much static data as
  // possible, and do a big memcpy. also, texture coords can be created in the VS,
  // and we should be using indexed tris.
  {
    rmt_ScopedCPUSample(MapWriteDiscard);

    PosTex3* vtx = _ctx->MapWriteDiscard<PosTex3>(_particleGpuObjects._vb);
    if (!vtx)
      return false;

    float* xx = _particles.x;
    float* yy = _particles.y;
    float* zz = _particles.z;

    float* ss = _particles.scale;
    Particles::Lifetime* ll = _particles.lifetime;

    for (int i = 0, e = _settings.num_particles; i < e; ++i)
    {
      float s = 10.f * ss[i];

      // 0--1
      // 2--3

      float lifetime = (float)ll[i].left / ll[i].total;

      PosTex3 v0 = { Vector3(-s, +s, s), Vector3(0, 0, lifetime) };
      PosTex3 v1 = { Vector3(+s, +s, s), Vector3(1, 0, lifetime) };
      PosTex3 v2 = { Vector3(-s, -s, s), Vector3(0, 1, lifetime) };
      PosTex3 v3 = { Vector3(+s, -s, s), Vector3(1, 1, lifetime) };

      Vector3 v(xx[i], yy[i], zz[i]);

      v0.pos += v;
      v1.pos += v;
      v2.pos += v;
      v3.pos += v;

      // 0, 1, 2
      // 2, 1, 3
      vtx[0] = v0;
      vtx[1] = v1;
      vtx[2] = v2;

      vtx[3] = v2;
      vtx[4] = v1;
      vtx[5] = v3;

      vtx += 6;
    }

    _ctx->Unmap(_particleGpuObjects._vb);
  }

  return true;
}

//------------------------------------------------------------------------------
bool ParticleTunnel::Render()
{
  static Color black(0,0,0,0);
  ScopedRenderTarget rt(DXGI_FORMAT_R16G16B16A16_FLOAT);
  _ctx->SetRenderTarget(rt.h, &black);

  _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::VertexShader, 0);
  _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::GeometryShader, 0);
  _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::PixelShader, 0);

  // Render the background
  _ctx->SetGpuObjects(_backgroundGpuObjects);
  _ctx->SetGpuState(_backgroundState);
  _ctx->Draw(3, 0);

  // Render particles
  _ctx->SetGpuObjects(_particleGpuObjects);
  _ctx->SetGpuState(_particleState);

  _ctx->SetSamplerState(_particleSamplerState, 0, ShaderType::PixelShader);
  _ctx->SetShaderResource(_particleTexture, ShaderType::PixelShader);
  _ctx->Draw(6 * _settings.num_particles, 0);

  // text
//   _ctx->SetGpuObjects(_textGpuObjects);
//   _ctx->SetGpuState(_textState);
//   _ctx->Draw((u32)_textParticles.selectedTris.size(), 0);

  // lines
  _ctx->SetGpuObjects(_linesGpuObjects);
  _ctx->SetGpuState(_linesState);
  _ctx->Draw(_textParticles.selectedTris.size(), 0);

  // compose final image on default swap chain
  _ctx->SetSwapChain(GRAPHICS.DefaultSwapChain(), Color(0.1f, 0.1f, 0.1f, 0));

  _ctx->SetSamplerState(_particleSamplerState, 0, ShaderType::PixelShader);
  _ctx->SetShaderResource(rt.h, ShaderType::PixelShader);
  _ctx->SetGpuObjects(_compositeGpuObjects);
  _ctx->SetGpuState(_compositeState);
  _ctx->Draw(3, 0);

  return true;
}

//------------------------------------------------------------------------------
void ParticleTunnel::RenderParameterSet()
{
  if (ImGui::ColorEdit4("Tint", &_settings.tint.x)) _cbPerFrame.tint = _settings.tint;
  if (ImGui::ColorEdit4("Inner", &_settings.inner_color.x)) _cbPerFrame.inner = _settings.inner_color;
  if (ImGui::ColorEdit4("Outer", &_settings.outer_color.x)) _cbPerFrame.outer = _settings.outer_color;
  ImGui::Separator();
  ImGui::InputInt("# particles", &_settings.num_particles, 25, 100);

  if (ImGui::SliderFloat("min dist", &_settings.text_min_dist, 1, 100)) Reset();
  if (ImGui::SliderFloat("max dist", &_settings.text_max_dist, 100, 2000)) Reset();
  if (ImGui::SliderFloat("triangle prob", &_settings.text_triangle_prob, 0.f, 1.f)) Reset();

  if (ImGui::Button("Reset"))
    Reset();
}

//------------------------------------------------------------------------------
void ParticleTunnel::SaveParameterSet()
{
  OutputBuffer buf;
  Serialize(buf, _settings);
  if (FILE* f = fopen(_configName.c_str(), "wt"))
  {
    fwrite(buf._buf.data(), 1, buf._ofs, f);
    fclose(f);
  }
}

//------------------------------------------------------------------------------
void ParticleTunnel::Reset()
{
  _textParticles.Create(_neuroticaTris, 5.f);
}

//------------------------------------------------------------------------------
bool ParticleTunnel::Close()
{
  return true;
}

//------------------------------------------------------------------------------
Effect* ParticleTunnel::Create(const char* name, u32 id)
{
  return new ParticleTunnel(name, id);
}

//------------------------------------------------------------------------------
const char* ParticleTunnel::Name()
{
  return "particle_tunnel";
}

//------------------------------------------------------------------------------
void ParticleTunnel::Register()
{
  DEMO_ENGINE.RegisterFactory(Name(), ParticleTunnel::Create);
}
