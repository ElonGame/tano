#pragma once

#include "../effect.hpp"
#include "../gpu_objects.hpp"
#include "../generated/demo.types.hpp"

namespace tano
{
  class DeferredContext;

  class ParticleTunnel : public Effect
  {
  public:

    ParticleTunnel(const string &name, u32 id);
    ~ParticleTunnel();
    virtual bool Init(const char* configFile) override;
    virtual bool Update(const UpdateState& state) override;
    virtual bool Render() override;
    virtual bool Close() override;

    static const char* Name();
    static Effect* Create(const char* name, u32 id);
    static void Register();

  private:

    void RenderParameterSet();
    void SaveParameterSet();

    struct Particle
    {
      Vector3 pos;
    };

    vector<Particle> _particles;

    struct CBufferPerFrame
    {
      Matrix world;
      Matrix viewProj;
      Color tint;
      Color inner;
      Color outer;
    };

    string _configName;
    ObjectHandle _particleTexture;
    GpuState _particleState;
    GpuState _backgroundState;

    GpuObjects _backgroundGpuObjects;
    GpuObjects _particleGpuObjects;

    ObjectHandle _texture;
    ObjectHandle _samplerState;
    ConstantBuffer<CBufferPerFrame> _cbPerFrame;

    ParticleTunnelSettings _settings;
  };
}
