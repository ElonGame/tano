#pragma once

#include "../base_effect.hpp"
#include "../gpu_objects.hpp"
#include "../generated/demo.types.hpp"
#include "../text_writer.hpp"
#include "../animation_helpers.hpp"
#include "../append_buffer.hpp"
#include "../tano_math.hpp"
#include "../mesh_utils.hpp"
#include "../camera.hpp"

namespace tano
{
  namespace scheduler
  {
    struct TaskData;
  }

  class ParticleTunnel : public BaseEffect
  {
  public:

    ParticleTunnel(const string &name, u32 id);
    ~ParticleTunnel();
    virtual bool OnConfigChanged(const vector<char>& buf) override;
    virtual bool Init(const char* configFile) override;
    virtual bool Update(const UpdateState& state) override;
    virtual bool FixedUpdate(const FixedUpdateState& state) override;
    virtual bool Render() override;
    virtual bool Close() override;

    static const char* Name();
    static BaseEffect* Create(const char* name, u32 id);
    static void Register();

  private:

    void Reset();
#if WITH_IMGUI
    void RenderParameterSet();
    void SaveParameterSet();
#endif

    void UpdateCameraMatrix(const UpdateState& state);
    void GenRandomPoints(float kernelSize);

    struct ParticleType
    {
      V4 pos;
    };

    struct ParticleEmitter
    {
      void Create(const V3& center, int numParticles);
      void Destroy();
      void Update(float dt);
      void CreateParticle(int idx, float s);
      void CopyToBuffer(ParticleType* vtx);

      XMVECTOR* pos = nullptr;
      XMVECTOR* vel = nullptr;

      int* _deadParticles = nullptr;

      int _numParticles = 0;
      V3 _center = { 0, 0, 0 };
    };

    struct EmitterKernelData
    {
      ParticleEmitter* emitter;
      float dt;
      ParticleType* vtx;
    };

    struct MemCpyKernelData
    {
      void *dst;
      const void* src;
      int size;
    };
    static void MemCpy(const scheduler::TaskData& data);

    static void UpdateEmitter(const scheduler::TaskData& data);
    static void CopyOutEmitter(const scheduler::TaskData& data);

    SimpleAppendBuffer<ParticleEmitter, 24> _particleEmitters;

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
      Vector4 time;
    };
    ConstantBuffer<CBufferPerFrame> _cbPerFrame;

    struct CBufferBasic
    {
      Matrix world;
      Matrix view;
      Matrix proj;
      Matrix viewProj;
      Vector4 cameraPos;
      Vector4 dim;
      Vector4 params = Vector4(5.0, 0.25f, 250.f, 1);
    };
    ConstantBuffer<CBufferBasic> _cbBasic;

    struct CBufferPerObject
    {
      Matrix world;
    };
    ConstantBuffer<CBufferPerObject> _cbPerObject;

    struct CBufferFracture
    {
      Matrix world;
      Matrix view;
      Matrix proj;
      Matrix viewProj;
      Vector4 cameraPos;
    };
    ConstantBuffer<CBufferFracture> _cbFracture;

    string _configName;

    GpuBundle _backgroundBundle;

    ObjectHandle _particleTexture;
    GpuBundle _particleBundle;

    ObjectHandle _lineTexture;
    GpuBundle _lineBundle;
    u32 _numLinesPoints = 0;

    GpuBundle _compositeBundle;

    ParticleTunnelSettings _settings;

    TextWriter _textWriter;
    struct TextData
    {
      vector<V3> outline;
      vector<V3> cap;
      vector<V3> verts;
      vector<V3> transformedVerts;
      vector<int> indices;
      int* neighbours;
    };
    TextData _textData[3];
    TextData* _curText = nullptr;

    float _particlesStart, _particlesEnd;

    struct FracturePiece
    {
      //scene::Mesh* mesh;
      V3 dir;
      V3 rot;
    };

    vector<FracturePiece> _pieces;

    SimpleAppendBuffer<V3, 1024> _randomPoints;
    GpuBundle _plexusLineBundle;

    bool _drawText = false;
    float _lineFade = 1.0f;
    float _curTime = 0;

    scene::Scene _scene;
    GpuBundle _fractureBundle;
    FreeFlyCamera _fixedCamera;
  };
}
