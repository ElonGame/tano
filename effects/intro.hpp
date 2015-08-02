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
#include "../shaders/out/intro.background_psbackground.cbuffers.hpp"
#include "../shaders/out/intro.composite_pscomposite.cbuffers.hpp"
#include "../shaders/out/intro.fracture_vsfracture.cbuffers.hpp"
#include "../shaders/out/intro.particle_gsparticle.cbuffers.hpp"
#include "../shaders/out/plexus_gslines.cbuffers.hpp"
#include "../shaders/out/plexus_pslines.cbuffers.hpp"

namespace tano
{
  namespace scheduler
  {
    struct TaskData;
  }

  class Intro : public BaseEffect
  {
  public:

    Intro(const string &name, const string& config, u32 id);
    ~Intro();
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

    void Reset();
#if WITH_IMGUI
    void RenderParameterSet();
    void SaveParameterSet(bool inc);
#endif

    void UpdateCameraMatrix(const UpdateState& state);
    void GenRandomPoints(float kernelSize);

    struct ParticleEmitter
    {
      void Create(const V3& center, int numParticles);
      void Destroy();
      void Update(float dt);
      void CreateParticle(int idx, float s);
      void CopyToBuffer(V4* vtx);

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
      V4* vtx;
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

    GpuBundle _backgroundBundle;
    ConstantBufferBundle<void, cb::IntroBackgroundF> _cbBackground;

    GpuBundle _compositeBundle;
    ConstantBufferBundle<void, cb::IntroCompositeF> _cbComposite;
    ConstantBufferBundle<
      cb::IntroFractureF, void, void,
      cb::IntroFractureO, void, void> _cbFracture;
    ConstantBufferBundle<void, void, cb::IntroParticleF> _cbParticle;

    ConstantBufferBundle<void, cb::PlexusPS, cb::PlexusGS> _cbPlexus;

    string _configName;
    
    ObjectHandle _particleTexture;
    GpuBundle _particleBundle;

    IntroSettings _settings;

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
    FreeflyCamera _fixedCamera;
  };
}
