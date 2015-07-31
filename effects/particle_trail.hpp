#pragma once
#include "../base_effect.hpp"
#include "../camera.hpp"
#include "../generated/demo.types.hpp"
#include "../gpu_objects.hpp"
#include "../shaders/out/trail.particle_gsparticle.cbuffers.hpp"
#include "../shaders/out/trail.composite_pscomposite.cbuffers.hpp"

namespace tano
{
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

    static const char* Name();
    static BaseEffect* Create(const char* name, const char* config, u32 id);
    static void Register();

  private:

#if WITH_IMGUI
    void RenderParameterSet();
    void SaveParameterSet();
#endif

    void Reset();
    void UpdateCameraMatrix(const UpdateState& state);

    ObjectHandle _particleTexture;
    GpuBundle _particleBundle;
    ConstantBufferBundle<void, void, cb::TrailParticleG> _cbParticle;

    GpuBundle _compositeBundle;
    ConstantBufferBundle<void, cb::TrailCompositeP> _cbComposite;

    ParticleTrailSettings _settings;
    FreeFlyCamera _camera;
  };

}