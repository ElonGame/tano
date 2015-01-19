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
  INIT(_gpuObjects.LoadShadersFromFile("shaders/particle_tunnel", "VsMain", "PsMain", vertexFlags));
  INIT(_gpuObjects.CreateDynamicVb(128 * 1024, sizeof(PosTex)));

  INIT_RESOURCE(_samplerState, GRAPHICS.CreateSamplerState(CD3D11_SAMPLER_DESC(CD3D11_DEFAULT())));
  INIT(_cbPerFrame.Create());

  _cbPerFrame.world = Matrix::Identity();
  Matrix view = Matrix::CreateLookAt(Vector3(0, 0, -500), Vector3(0, 0, 0), Vector3(0, 1, 0));
  Matrix proj = Matrix::CreatePerspectiveFieldOfView(XMConvertToRadians(45), 16/10.f, 0.1f, 1000.f);
  Matrix viewProj = view * proj;
  _cbPerFrame.viewProj = viewProj.Transpose();
  _cbPerFrame.tint = _settings.tint;

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool ParticleTunnel::Update(const UpdateState& state)
{

  //GRAPHICS.GetTempRenderTarget()

  PosTex* vtx = _ctx->MapWriteDiscard<PosTex>(_gpuObjects._vb);
  if (!vtx)
    return false;

  float s = 50;
  // 0--1
  // 2--3
  PosTex v0 ={ Vector3(-s, +s, 0), Vector2(0, 0) };
  PosTex v1 ={ Vector3(+s, +s, 0), Vector2(1, 0) };
  PosTex v2 ={ Vector3(-s, -s, 0), Vector2(0, 1) };
  PosTex v3 ={ Vector3(+s, -s, 0), Vector2(1, 1) };

  // 0, 1, 2
  // 2, 1, 3
  vtx[0] = v0;
  vtx[1] = v1;
  vtx[2] = v2;

  vtx[3] = v2;
  vtx[4] = v1;
  vtx[5] = v3;

  _ctx->Unmap(_gpuObjects._vb);
  return true;
}

//------------------------------------------------------------------------------
void ParticleTunnel::RenderParameterSet()
{
  if (ImGui::ColorEdit4("Tint", &_settings.tint.x))
    _cbPerFrame.tint = _settings.tint;
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
bool ParticleTunnel::Render()
{
  _ctx->SetSwapChain(GRAPHICS.DefaultSwapChain(), Color(0.1f, 0.1f, 0.1f, 0));
  _ctx->BeginFrame();

  _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::VertexShader, 0);
  _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::PixelShader, 0);
  _ctx->SetGpuObjects(_gpuObjects);

  _ctx->SetSamplerState(_samplerState, 0, ShaderType::PixelShader);
  _ctx->SetShaderResource(_particleTexture, ShaderType::PixelShader);

  _ctx->Draw(6,0);

  _ctx->EndFrame();

  return true;
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
