#pragma once

#include "../effect.hpp"
#include "../gpu_objects.hpp"
#include "../generated/demo.types.hpp"
#include "../text_writer.hpp"

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

    // SOA style
    struct Particles
    {
      void Create(int numParticles);
      void Destroy();
      void Update(float dt);
      void CreateParticle(int idx, float s);

      // TODO: test speed diff if these are double buffered, to avoid LHS
      float* x = nullptr;
      float* y = nullptr;
      float* z = nullptr;
      float* vx = nullptr;
      float* vy = nullptr;
      float* vz = nullptr;

      float* scale = nullptr;

      struct Lifetime
      {
        int total;
        int left;
      };
      Lifetime* lifetime = nullptr;

      int* deadParticles = nullptr;

      int numParticles = 0;
    };

    Particles _particles;

    struct CBufferPerFrame
    {
      Matrix world;
      Matrix viewProj;
      Color tint;
      Color inner;
      Color outer;
    };

    string _configName;

    GpuState _backgroundState;
    GpuObjects _backgroundGpuObjects;

    ObjectHandle _particleTexture;
    GpuState _particleState;
    GpuObjects _particleGpuObjects;
    ObjectHandle _particleSamplerState;

    GpuState _textState;
    GpuObjects _textGpuObjects;

    GpuState _compositeState;
    GpuObjects _compositeGpuObjects;

    ConstantBuffer<CBufferPerFrame> _cbPerFrame;
    ParticleTunnelSettings _settings;

    TextWriter _textWriter;
    vector<Vector3> _neuroticaTris;
  };
}
