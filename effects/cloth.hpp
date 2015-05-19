#pragma once

#include "../effect.hpp"
#include "../gpu_objects.hpp"
#include "../generated/demo.types.hpp"
#include "../mesh_loader.hpp"
#include "../animation_helpers.hpp"
#include "../camera.hpp"
#include "../scene.hpp"

namespace tano
{
  class Cloth : public Effect
  {
  public:

    Cloth(const string &name, u32 id);
    ~Cloth();
    virtual bool Init(const char* configFile) override;
    virtual bool Update(const UpdateState& state) override;
    virtual bool Render() override;
    virtual bool Close() override;
    virtual bool InitAnimatedParameters() override;

    static const char* Name();
    static Effect* Create(const char* name, u32 id);
    static void Register();

  private:

    void Reset();
#if WITH_IMGUI
    void RenderParameterSet();
    void SaveParameterSet();
#endif

    void UpdateCameraMatrix();
    void UpdateParticles(const UpdateState& state);
    bool InitParticles();
    void ResetParticles();

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

    struct Particle
    {
      Vector3 pos;
      Vector3 lastPos;
      Vector3 acc;
    };

    struct Constraint
    {
      Particle* p0;
      Particle* p1;
      float restLength;
    };

    vector<Particle> _particles;
    vector<Constraint> _constraints;
    u32 _numTris = 0;
    u32 _numParticles = 0;
    u32 _clothDimX = 0;
    u32 _clothDimY = 0;

    GpuState _clothState;
    GpuObjects _clothGpuObjects;
    ClothSettings _settings;
    string _configName;
    Camera _camera;
  };
}
