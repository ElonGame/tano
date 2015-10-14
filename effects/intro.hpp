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
#include "../shaders/out/intro.textpoly_vsmesh.cbuffers.hpp"
#include "../shaders/out/intro.textpoly_psmesh.cbuffers.hpp"
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

    ConstantBufferBundle<cb::IntroTextpolyV, cb::IntroTextpolyP> _cbTextPoly;
    GpuBundle _textPolyBundle;

    string _configName;
    
    ObjectHandle _csParticleBlur;

    ObjectHandle _particleTexture;
    GpuBundle _particleBundle;

    IntroSettings _settings;

    float _particlesStart, _particlesEnd;

    int _numSpawnedParticles = 0;

    float _curTime = 0;

    struct IntroTexture
    {
      ObjectHandle h;
      D3DX11_IMAGE_INFO info;
    };
    IntroTexture _introTexture[3];

    FreeflyCamera _textCamera;
  };
}
