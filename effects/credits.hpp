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

    struct CBufferPerFrame
    {
      Matrix world;
      Matrix view;
      Matrix proj;
      Matrix viewProj;
      Vector4 dim;
      Vector3 cameraPos;
      float pad1;
      Vector3 cameraLookAt;
      float pad2;
      Vector3 cameraUp;
      float pad3;
    };
    ConstantBuffer<CBufferPerFrame> _cbPerFrame;

    struct ClothParticle
    {
      V3 pos;
      V3 lastPos;
      V3 acc;
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

    vector<V4> _particles;

    vector<ClothParticle> _clothParticles;
    vector<Constraint> _constraints;

    ConstantBufferBundle<void, void, cb::CreditsParticleF> _cbParticle;

    u32 _numTris = 0;
    u32 _numParticles = 0;
    int _clothDimX = 0;
    int _clothDimY = 0;

    ObjectHandle _particleTexture;
    GpuBundle _particleBundle;

    GpuState _clothState;
    GpuObjects _clothGpuObjects;
    CreditsSettings _settings;
    FreeflyCamera _camera;

    RollingAverage<double> _avgUpdate;
  };
}
