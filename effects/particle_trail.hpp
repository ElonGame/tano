#pragma once
#include "../base_effect.hpp"
#include "../camera.hpp"
#include "../generated/demo.types.hpp"
#include "../gpu_objects.hpp"
#include "../shaders/out/trail.particle_gsparticle.cbuffers.hpp"
#include "../shaders/out/trail.composite_pscomposite.cbuffers.hpp"
#include "../shaders/out/trail.lines_gslines.cbuffers.hpp"
#include "../shaders/out/trail.lines_pslines.cbuffers.hpp"
#include "../shaders/out/trail.background_psbackground.cbuffers.hpp"

namespace tano
{
  struct Taily
  {
    enum
    {
      MAX_TAIL_LENGTH = 16 * 1024
    };

    void AddPos(const V3& pos);
    V3* CopyOut(V3* buf);

    V3 cur = { 0.1f, 0, 0 };
    V3 tail[MAX_TAIL_LENGTH];
    int tailLength = 0;
    int writePos = 0;
  };

  struct Pathy
  {
    enum
    {
      TOTAL_POINTS = 32 * 1024,
      POINTS_PER_CHILD = 1024,
      MAX_NUM_LINES = 32
    };
    void Create();
    V3* CopyOut(V3* buf);

    int numVerts = 0;
    V3 verts[TOTAL_POINTS];

    struct Instance
    {
      Vector3 cur;
      float scale;
      float angleX, angleY, angleZ;
    };

    float len = 5;

    float angleXMean = 0.f;
    float angleXVariance = 0.5f;

    float angleYMean = 0.f;
    float angleYVariance = 0.5f;

    float angleZMean = 0.f;
    float angleZVariance = 0.5f;

    float childProb = 0.85f;
    float childScale = 0.75f;

    int numLines = 0;

    deque<Instance> children;
  };

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
    virtual const char* GetName() { return Name(); }

    static const char* Name();
    static BaseEffect* Create(const char* name, const char* config, u32 id);
    static void Register();

  private:
#if WITH_IMGUI
    void RenderParameterSet();
    void SaveParameterSet(bool inc);
#endif

    void Reset();
    void UpdateCameraMatrix(const UpdateState& state);

    Pathy _pathy;
    Taily _taily;

    ObjectHandle _particleTexture;
    GpuBundle _particleBundle;
    ConstantBufferBundle<void, void, cb::TrailParticleG> _cbParticle;

    GpuBundle _backgroundBundle;
    ConstantBufferBundle<void, cb::TrailBackgroundF> _cbBackground;

    GpuBundle _compositeBundle;
    ConstantBufferBundle<void, cb::TrailCompositeP> _cbComposite;

    GpuBundle _lineBundle;
    ConstantBufferBundle<void, cb::TrailLinesPS, cb::TrailLinesGS> _cbPlexus;

    ParticleTrailSettings _settings;
    FreeflyCamera _camera;
  };
}