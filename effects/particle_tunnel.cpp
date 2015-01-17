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

namespace
{
  struct CBufferPerFrame
  {
    Vector4 col;
  };
}
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
bool ParticleTunnel::Show()
{
  return true;
}

//------------------------------------------------------------------------------
bool ParticleTunnel::Hide()
{
  return true;
}

//------------------------------------------------------------------------------
bool ParticleTunnel::Init(const char* configFile)
{
  BEGIN_INIT_SEQUENCE();

  vector<char> buf;
  INIT(RESOURCE_MANAGER.LoadFile(configFile, &buf));
  INIT(ParseParticleTunnelSettings(InputBuffer(buf), &_settings));

//  _config = config.particle_config();

  INIT_RESOURCE(_texture, RESOURCE_MANAGER.LoadTexture("gfx/Abstract_BG_Texture2.jpg"));
  INIT(GRAPHICS.LoadShadersFromFile("shaders/fullscreen", &_vs, &_ps, nullptr, 0));

  CD3D11_SAMPLER_DESC sampler = CD3D11_SAMPLER_DESC(CD3D11_DEFAULT());
  INIT_RESOURCE(_samplerState, GRAPHICS.CreateSamplerState(sampler));
  INIT_RESOURCE(_cbuffer, GRAPHICS.CreateBuffer(D3D11_BIND_CONSTANT_BUFFER, sizeof(CBufferPerFrame), true));

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool ParticleTunnel::Update(const UpdateState& state)
{
  return true;
}

struct CommandBase
{
  u8* data = nullptr;
  CommandBase* next = nullptr;
};

struct SetSwapChain : CommandBase
{
  SetSwapChain(ObjectHandle h, const Color& color) : handle(h), clearColor(color) {}
  ObjectHandle handle;
  Color clearColor;
};

struct SetConstantBuffer : CommandBase
{
  ObjectHandle handle;
  ShaderType shader;
};

struct RenderQueue
{
  static const u32 INITIAL_MEMORY_SIZE = 128 * 1024 * 1024;

  RenderQueue() : _memory(INITIAL_MEMORY_SIZE) {}

  void FrameStart();

  template <typename T, typename... Args>
  T* AllocateCommand(u32 dataSize, Args&&... args)
  {
    T* cmd = new (AllocChunk(sizeof(T)))T(forward<Args>(args)...);
    _memoryOfs += sizeof(T);

    if (dataSize)
      cmd->data = AllocChunk(dataSize);

    return cmd;
  }

  template <typename T, typename... Args>
  T* AppendCommand(CommandBase* parent, u32 dataSize, Args&&... args)
  {
    T* cmd = new (AllocChunk(sizeof(T)))T(forward<Args>(args)...);
    _memoryOfs += sizeof(T);

    if (parent)
      parent->next = cmd;

    if (dataSize)
      cmd->data = AllocChunk(dataSize);

    return cmd;
  }

  u8* AllocChunk(u32 size)
  {
    u8* tmp = &_memory[_memoryOfs];
    _memoryOfs += size;
    return tmp;
  }

  vector<u8> _memory;
  u32 _memoryOfs = 0;
};

void RenderQueue::FrameStart()
{
  // reset the render queue
  _memoryOfs = 0;
}


RenderQueue renderQueue;

//------------------------------------------------------------------------------
bool ParticleTunnel::Render()
{
  SetSwapChain* setSwapChain = renderQueue.AllocateCommand<SetSwapChain>(0, GRAPHICS.DefaultSwapChain(), Color(0,0,0,0));
  SetConstantBuffer* setConstantBuffer = renderQueue.AppendCommand<SetConstantBuffer>(setSwapChain, 0);

  float black[] ={ 0, 0, 0, 0 };
  _ctx->SetSwapChain(GRAPHICS.DefaultSwapChain(), black);
  _ctx->BeginFrame();

  CBufferPerFrame cb;
  //::boba::common::FromProtocol(_config.bb_col4f(), &cb.col);
  _ctx->SetCBuffer(_cbuffer, &cb, sizeof(cb), ShaderType::PixelShader, 0);

  _ctx->SetVS(_vs);
  _ctx->SetPS(_ps);
  _ctx->SetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
  _ctx->SetShaderResource(_texture, ShaderType::PixelShader);
  _ctx->SetSamplerState(_samplerState, 0, ShaderType::PixelShader);
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
