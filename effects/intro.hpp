#pragma once

#include "../base_effect.hpp"
#include "../gpu_objects.hpp"
#include "../generated/demo.types.hpp"
#include "../text_writer.hpp"
#include "../animation_helpers.hpp"
#include "../append_buffer.hpp"
#include "../tano_math.hpp"
#include "../camera.hpp"
#include "../shaders/out/intro.background_psbackground.cbuffers.hpp"
#include "../shaders/out/intro.composite_pscomposite.cbuffers.hpp"
#include "../shaders/out/intro.fracture_vsfracture.cbuffers.hpp"
#include "../shaders/out/intro.particle_gsparticle.cbuffers.hpp"
#include "../shaders/out/plexus_gslines.cbuffers.hpp"
#include "../shaders/out/plexus_pslines.cbuffers.hpp"
#include "../particle_emitters.hpp"

namespace tano
{
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

    void UpdateParticleEmitters(float dt);
    void CopyOutParticleEmitters();


    SimpleAppendBuffer<RadialParticleEmitter, 24> _particleEmitters;

    GpuBundle _backgroundBundle;
    ConstantBufferBundle<void, cb::IntroBackgroundF> _cbBackground;

    GpuBundle _compositeBundle;
    ConstantBufferBundle<void, cb::IntroCompositeF> _cbComposite;
    ConstantBufferBundle<void, void, cb::IntroParticleF> _cbParticle;

    ConstantBufferBundle<void, cb::PlexusPS, cb::PlexusGS> _cbPlexus;

    string _configName;
    
    ObjectHandle _csParticleBlur;

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
      vector<u32> edges;
      vector<V3> keyframes;
      int* neighbours;
      enum { STATE_INACTIVE, STATE_ACTIVE, STATE_DONE };
      int state = STATE_INACTIVE;
      float fade = 1;
      ObjectHandle vb;
    };

    void CreateKeyframes(TextData* textData);
    void UpdateText(const UpdateState& state, TextData* textData, const char* prefix);

    TextData _textData[3];

    float _particlesStart, _particlesEnd;

    int _numSpawnedParticles = 0;
    SimpleAppendBuffer<V3, 1024> _randomPoints;
    GpuBundle _plexusLineBundle;

    bool _drawText = true;
    float _curTime = 0;

    FreeflyCamera _textCamera;
    FreeflyCamera _camera;
  };
}
