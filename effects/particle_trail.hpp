#pragma once
#include "../base_effect.hpp"
#include "../camera.hpp"
#include "../generated/demo.types.hpp"
#include "../gpu_objects.hpp"
#include "../shaders/out/trail.particle_gsparticle.cbuffers.hpp"
#include "../shaders/out/trail.composite_pscomposite.cbuffers.hpp"

namespace tano
{
  struct ParticleFade
  {
    V3 pos;
  };
  struct Taily
  {
    enum
    {
      MAX_TAIL_LENGTH = 16 * 1024
    };

    void AddPos(const V3& pos);
    V3* CopyOut(V3* buf);

    V3 cur = {0.1f, 0, 0};
    V3 tail[MAX_TAIL_LENGTH];
    int tailLength = 0;
    int writePos = 0;
  };

  class ParticleTrail : public BaseEffect
  {
  public:
    ParticleTrail(const string& name, const string& config, u32 id);
    ~ParticleTrail();
    virtual bool OnConfigChanged(const vector<char>& buf) override;
    virtual bool Init() override;
    virtual bool Update(const UpdateState& state) override;
    virtual bool FixedUpdate(const FixedUpdateState& state) override;
    virtual bool Render() override;
    virtual bool Close() override;
    virtual const char* GetName() { return Name(); }

    static const char* Name();
    static BaseEffect* Create(const char* name, const char* config, u32 id);
    static void Register();

  private:
#if WITH_IMGUI
    void RenderParameterSet();
    void SaveParameterSet(bool inc);
#endif

    void Reset();
    void UpdateCameraMatrix(const UpdateState& state);

    Taily _taily;

    ObjectHandle _particleTexture;
    GpuBundle _particleBundle;
    ConstantBufferBundle<void, void, cb::TrailParticleG> _cbParticle;

    GpuBundle _compositeBundle;
    ConstantBufferBundle<void, cb::TrailCompositeP> _cbComposite;

    ParticleTrailSettings _settings;
    FreeflyCamera _camera;
  };
}