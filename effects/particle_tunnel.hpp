#pragma once

#include "../effect.hpp"
#include "../gpu_objects.hpp"
#include "../generated/demo.types.hpp"
#include "../text_writer.hpp"
#include "../post_process.hpp"

namespace tano
{
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

    void Reset();
#if WITH_IMGUI
    void RenderParameterSet();
    void SaveParameterSet();
#endif

    void UpdateCameraMatrix();
    void ApplyBlur(ObjectHandle inputBuffer, ObjectHandle outputBuffer);

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

    struct TextParticles
    {
      TextParticles(const ParticleTunnelSettings& settings) : settings(settings) {}
      void Create(const vector<Vector3>& verts, float targetTime);
      void Destroy();
      void Update(const UpdateState& state);

      float* curX = nullptr;
      float* curY = nullptr;
      float* curZ = nullptr;
      float* startX = nullptr;
      float* startY = nullptr;
      float* startZ = nullptr;
      float* endX = nullptr;
      float* endY = nullptr;
      float* endZ = nullptr;

      int numParticles = 0;
      const ParticleTunnelSettings& settings;
      vector<int> selectedTris;
    };

    Particles _particles;

    struct CBufferPerFrame
    {
      Matrix world;
      Matrix view;
      Matrix proj;
      Matrix viewProj;
      Color tint;
      Color inner;
      Color outer;
      Vector4 dim;
      Vector4 viewDir;
      Vector4 camPos;
      Vector4 dofSettings;
    };
    ConstantBuffer<CBufferPerFrame> _cbPerFrame;

    string _configName;

    GpuState _backgroundState;
    GpuObjects _backgroundGpuObjects;

    ObjectHandle _particleTexture;
    GpuState _particleState;
    GpuObjects _particleGpuObjects;

    GpuState _textState;
    GpuObjects _textGpuObjects;

    ObjectHandle _lineTexture;
    GpuState _linesState;
    GpuObjects _linesGpuObjects;
    u32 _numLinesPoints = 0;

    GpuState _compositeState;
    GpuObjects _compositeGpuObjects;

    ParticleTunnelSettings _settings;

    struct CBufferBlur
    {
      Vector2 inputSize;
      float radius = 10;
    };
    ConstantBuffer<CBufferBlur> _cbBlur;
    ObjectHandle _csBlurX;
    ObjectHandle _csBlurTranspose;
    ObjectHandle _csCopyTranspose;

    TextWriter _textWriter;
    vector<Vector3> _neuroticaTris;
    TextParticles _textParticles;
  };
}
