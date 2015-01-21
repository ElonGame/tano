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

using namespace tano;
using namespace bristol;

//------------------------------------------------------------------------------
ParticleTunnel::ParticleTunnel(const string &name, u32 id)
  : Effect(name, id)
{
  PROPERTIES.Register("particle tunnel", 
    bind(&ParticleTunnel::RenderParameterSet, this),
    bind(&ParticleTunnel::SaveParameterSet, this));
}

//------------------------------------------------------------------------------
ParticleTunnel::~ParticleTunnel()
{
}

//------------------------------------------------------------------------------
bool ParticleTunnel::Init(const char* configFile)
{
  BEGIN_INIT_SEQUENCE();

  _configName = configFile;
  bool res = true;
  FileWatcher& watcher = DEMO_ENGINE.GetFileWatcher();
  vector<char> buf;
  if (!RESOURCE_MANAGER.LoadFile(configFile, &buf))
    return false;

  if (!ParseParticleTunnelSettings(InputBuffer(buf), &_settings))
    return false;

  INIT(res);

  watcher.AddFileWatch(_settings.texture, nullptr, true, &res, [this](const string& filename, void* ctx) {
    _particleTexture = RESOURCE_MANAGER.LoadTexture(filename.c_str());
    return _particleTexture.IsValid();
  });

  u32 vertexFlags = VertexFlags::VF_POS | VertexFlags::VF_TEX0;
  INIT(_particleTexture.IsValid())
  INIT(_particleGpuObjects.LoadShadersFromFile("shaders/particle_tunnel", "VsMain", "PsMain", vertexFlags));
  INIT(_particleGpuObjects.CreateDynamicVb(16 * 1024 * 1024, sizeof(PosTex)));

  INIT(_backgroundGpuObjects.LoadShadersFromFile("shaders/particle_tunnel", "VsBackground", "PsBackground"));
  INIT(_backgroundState.Create());

  {
    CD3D11_RASTERIZER_DESC rssDesc = CD3D11_RASTERIZER_DESC(CD3D11_DEFAULT());
    rssDesc.CullMode = D3D11_CULL_NONE;

    CD3D11_DEPTH_STENCIL_DESC dsDesc = CD3D11_DEPTH_STENCIL_DESC(CD3D11_DEFAULT());
    dsDesc.DepthEnable = FALSE;

    CD3D11_BLEND_DESC blendDesc = CD3D11_BLEND_DESC(CD3D11_DEFAULT());

    // pre-multiplied alpha
    D3D11_RENDER_TARGET_BLEND_DESC& b = blendDesc.RenderTarget[0];
    b.BlendEnable = TRUE;
    b.BlendOp = D3D11_BLEND_OP_ADD;
    b.SrcBlend = D3D11_BLEND_ONE;
    b.DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
    b.BlendOpAlpha = D3D11_BLEND_OP_ADD;
    b.SrcBlendAlpha = D3D11_BLEND_ONE;
    b.DestBlendAlpha = D3D11_BLEND_INV_SRC_ALPHA;
    INIT(_particleState.Create(&dsDesc, &blendDesc, &rssDesc));
  }

  INIT_RESOURCE(_samplerState, GRAPHICS.CreateSamplerState(CD3D11_SAMPLER_DESC(CD3D11_DEFAULT())));
  INIT(_cbPerFrame.Create());

  _particles.resize(_settings.num_particles);
  for (int i = 0; i < _settings.num_particles; ++i)
  {
    _particles[i].pos = Vector3(randf(-100.f, 100.f), randf(-100.f, 100.f), randf(-100.f, 100.f));
  }

  _cbPerFrame.world = Matrix::Identity();
  Matrix view = Matrix::CreateLookAt(Vector3(0, 0, -500), Vector3(0, 0, 0), Vector3(0, 1, 0));
  Matrix proj = Matrix::CreatePerspectiveFieldOfView(XMConvertToRadians(45), 16/10.f, 0.1f, 1000.f);
  Matrix viewProj = view * proj;
  _cbPerFrame.viewProj = viewProj.Transpose();
  _cbPerFrame.tint = _settings.tint;
  _cbPerFrame.inner = _settings.inner_color;
  _cbPerFrame.outer = _settings.outer_color;

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool ParticleTunnel::Update(const UpdateState& state)
{
  PosTex* vtx = _ctx->MapWriteDiscard<PosTex>(_particleGpuObjects._vb);
  if (!vtx)
    return false;

  for (int i = 0; i < _settings.num_particles; ++i)
  {
    float s = i / 100.f;

    // 0--1
    // 2--3
    PosTex v0 ={ Vector3(-s, +s, s), Vector2(0, 0) };
    PosTex v1 ={ Vector3(+s, +s, s), Vector2(1, 0) };
    PosTex v2 ={ Vector3(-s, -s, s), Vector2(0, 1) };
    PosTex v3 ={ Vector3(+s, -s, s), Vector2(1, 1) };

    v0.pos += _particles[i].pos;
    v1.pos += _particles[i].pos;
    v2.pos += _particles[i].pos;
    v3.pos += _particles[i].pos;

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
  return true;
}

//------------------------------------------------------------------------------
bool ParticleTunnel::Render()
{
  static Color black(0,0,0,0);
  //ScopedRenderTarget rt(DXGI_FORMAT_R16G16B16A16_FLOAT, BufferFlags());
  //_ctx->SetRenderTarget(rt.h, &black);
  _ctx->SetSwapChain(GRAPHICS.DefaultSwapChain(), Color(0.1f, 0.1f, 0.1f, 0));

  // Render the background
  _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::PixelShader, 0);
  _ctx->SetGpuObjects(_backgroundGpuObjects);
  _ctx->SetGpuState(_backgroundState);
  _ctx->Draw(3, 0);

  // Render particles

  // do funky stuff!

  _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::VertexShader, 0);
  _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::PixelShader, 0);
  _ctx->SetGpuObjects(_particleGpuObjects);
  _ctx->SetGpuState(_particleState);

  _ctx->SetSamplerState(_samplerState, 0, ShaderType::PixelShader);
  _ctx->SetShaderResource(_particleTexture, ShaderType::PixelShader);

  _ctx->Draw(6 * _settings.num_particles, 0);

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
