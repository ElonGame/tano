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

  struct V3
  {
    V3() {}
    V3(float x, float y, float z) : x(x), y(y), z(z) {}
    V3(const Vector3& v) : x(v.x), y(v.y), z(v.z) {}
    float x, y, z;
  };

  inline float Distance(const V3& a, const V3& b)
  {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float dz = a.z - b.z;

    return sqrtf(dx*dx+dy*dy+dz*dz);
  }

  inline V3 operator*(float s, const V3& v)
  {
    return V3(s*v.x, s*v.y, s*v.z);
  }

  inline V3 operator-(const V3& a, const V3& b)
  {
    return V3(a.x - b.x, a.y - b.y, a.z - b.z);
  }

  inline V3 operator+(const V3& a, const V3& b)
  {
    return V3(a.x + b.x, a.y + b.y, a.z + b.z);
  }

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
    void UpdateDistTable();

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
      V3 pos;
      V3 lastPos;
      V3 acc;
    };

    struct Constraint
    {
      Particle* p0;
      Particle* p1;
      float restLength;
    };

    struct ConstraintByParticle
    {
      Particle* p1;
      float restLength;
    };

    vector<Particle> _particles;
    vector<Constraint> _constraints;
    unordered_map<Particle*, vector<ConstraintByParticle>> _constraintsByParticle;
    u32 _numTris = 0;
    u32 _numParticles = 0;
    u32 _clothDimX = 0;
    u32 _clothDimY = 0;

    GpuState _clothState;
    GpuObjects _clothGpuObjects;
    ClothSettings _settings;
    string _configName;
    Camera _camera;

    RollingAverage<double> _avgUpdate;
    float* _distTable = nullptr;
  };
}
