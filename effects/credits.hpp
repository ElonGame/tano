#pragma once

#include "../base_effect.hpp"
#include "../gpu_objects.hpp"
#include "../generated/demo.types.hpp"
#include "../mesh_loader.hpp"
#include "../animation_helpers.hpp"
#include "../camera.hpp"
#include "../scene.hpp"
#include "../tano_math.hpp"
#include "../shaders/out/credits.particle_gsparticle.cbuffers.hpp"
#include "../shaders/out/credits.composite_pscomposite.cbuffers.hpp"
#include "../shaders/out/credits.background_psbackground.cbuffers.hpp"
#include "../shaders/out/credits.gaussian_psblur35.cbuffers.hpp"

namespace tano
{
  class Credits : public BaseEffect
  {
  public:

    Credits(const string &name, const string& config, u32 id);
    ~Credits();
    virtual bool OnConfigChanged(const vector<char>& buf) override;
    virtual bool Init() override;
    virtual bool Update(const UpdateState& state) override;
    virtual bool FixedUpdate(const FixedUpdateState& state) override;
    virtual bool Render() override;
    virtual bool Close() override;
    virtual const char* GetName() { return Name(); }

    virtual bool InitAnimatedParameters() override;

    static const char* Name();
    static BaseEffect* Create(const char* name, const char* config, u32 id);
    static void Register();

  private:

    void Reset();
#if WITH_IMGUI
    void RenderParameterSet();
    void SaveParameterSet(bool inc);
#endif

    void UpdateCameraMatrix(const UpdateState& state);
    void UpdateParticles(const FixedUpdateState& state);
    bool InitParticles();
    void ResetParticles();
    void GroupConstraints();

    void ResetParticleSpline();
    void UpdateParticleSpline(float dt);
    void InitParticleSpline(const vector<int>& indices);

    struct ClothParticle
    {
      vec3 pos;
      vec3 lastPos;
      vec3 acc;
    };

    struct Constraint
    {
      Constraint() {}
      Constraint(u32 idx0, u32 idx1, float length) : idx0(idx0), idx1(idx1), restLength(length) {}
      union {
        ClothParticle* p0;
        u32 idx0;
      };
      union {
        ClothParticle* p1;
        u32 idx1;
      };

      float restLength;
    };

    struct ParticleState
    {
      float speed;
      float pos;
      float height;
      float angle;
      float angleInc;
      float fade;
      float fadeInc;
    };

    vector<ParticleState> _particleState;

    vector<vec4> _particles;

    vector<ClothParticle> _clothParticles;
    vector<Constraint> _constraints;

    ConstantBufferBundle<void, void, cb::CreditsParticleF> _cbParticle;

    GpuBundle _backgroundBundle;
    ConstantBufferBundle<void, cb::CreditsBackgroundF> _cbBackground;

    GpuBundle _compositeBundle;
    ConstantBufferBundle<void, cb::CreditsCompositeP> _cbComposite;

    GpuBundle _blurBundle;
    ConstantBufferBundle<void, cb::CreditsGaussianF> _cbBlur;

    u32 _numTris = 0;
    u32 _numClothParticles = 0;
    int _clothDimX = 0;
    int _clothDimY = 0;

    ObjectHandle _creditsTexture;
    ObjectHandle _particleTexture;
    GpuBundle _particleBundle;
    int _numParticles = 0;

    GpuState _clothState;
    GpuObjects _clothGpuObjects;
    CreditsSettings _settings;

    FreeflyCamera _textCamera;
    ObjectHandle _csBlur;

    RollingAverage<double> _avgUpdate;
  };
}
