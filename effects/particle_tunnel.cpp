#include "particle_tunnel.hpp"
#include "../tano.hpp"
#include "../graphics.hpp"
#include "../deferred_context.hpp"
#include "../demo_engine.hpp"
#include "../resource_manager.hpp"
#include "../init_sequence.hpp"
#include "../generated/demo.parse.hpp"
#include "../generated/input_buffer.hpp"

using namespace tano;
using namespace bristol;

//------------------------------------------------------------------------------
ParticleTunnel::ParticleTunnel(const string &name, u32 id)
  : Effect(name, id)
{
}

//------------------------------------------------------------------------------
ParticleTunnel::~ParticleTunnel()
{
}

//------------------------------------------------------------------------------
bool ParticleTunnel::Init(const char* configFile)
{
  BEGIN_INIT_SEQUENCE();

  vector<char> buf;
  INIT(RESOURCE_MANAGER.LoadFile(configFile, &buf));
  INIT(ParseParticleTunnelSettings(InputBuffer(buf), &_settings));

//  _config = config.particle_config();

  FileWatcher& watcher = DEMO_ENGINE.GetFileWatcher();

  bool res = true;
  watcher.AddFileWatch(_settings.texture, nullptr, true, &res, [this](const string& filename, void* ctx) {
    _particleTexture = RESOURCE_MANAGER.LoadTexture(filename.c_str());
    return _particleTexture.IsValid();
  });

  u32 vertexFlags = VertexFlags::VF_POS | VertexFlags::VF_COLOR;
  INIT(_particleTexture.IsValid())
  INIT(_gpuObjects.LoadShadersFromFile("shaders/particle_tunnel", "VsMain", "PsMain", vertexFlags));
  INIT(_gpuObjects.CreateDynamicVb(128 * 1024, sizeof(PosCol)));

  INIT_RESOURCE(_samplerState, GRAPHICS.CreateSamplerState(CD3D11_SAMPLER_DESC(CD3D11_DEFAULT())));
  INIT(_cbPerFrame.Create());

  _cbPerFrame.world = Matrix::Identity();
  Matrix view = Matrix::CreateLookAt(Vector3(0, 0, -500), Vector3(0, 0, 0), Vector3(0, 1, 0));
  Matrix proj = Matrix::CreatePerspectiveFieldOfView(XMConvertToRadians(45), 16/10.f, 0.1f, 1000.f);
  Matrix viewProj = view * proj;
  _cbPerFrame.viewProj = viewProj.Transpose();

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool ParticleTunnel::Update(const UpdateState& state)
{
  PosCol* vtx = _ctx->MapWriteDiscard<PosCol>(_gpuObjects._vb);
  if (!vtx)
    return false;

  vtx[1].pos = Vector3(-100, -100, 0);
  vtx[0].pos = Vector3(+200, -100, 0);
  vtx[2].pos = Vector3(0, 100, 0);

  _ctx->Unmap(_gpuObjects._vb);
  return true;
}

//------------------------------------------------------------------------------
bool ParticleTunnel::Render()
{
  _ctx->SetSwapChain(GRAPHICS.DefaultSwapChain(), Color(0.1f, 0.1f, 0.1f, 0));
  _ctx->BeginFrame();

  _ctx->SetCBuffer(_cbPerFrame, ShaderType::VertexShader, 0);
  _ctx->SetGpuObjects(_gpuObjects);

  _ctx->Draw(3,0);

  _ctx->EndFrame();

  return true;
}

//------------------------------------------------------------------------------
bool ParticleTunnel::Close()
{
  return true;
}

//------------------------------------------------------------------------------
bool ParticleTunnel::SaveSettings()
{
  if (FILE* f = fopen(_configName.c_str() ,"wt"))
  {
    //fprintf(f, "%s", _config.DebugString().c_str());
    fclose(f);
  }
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
