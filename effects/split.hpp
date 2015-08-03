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

    vector<V3> verts;

    struct Instance
    {
      Vector3 cur;
      float scale;
      float angleX, angleY, angleZ;
      vector<V3> verts;
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

    int maxChildren = 512;

    struct Line
    {
      int startOfs;
      int size;
    };

    vector<Line> lines;
  };

  class Split : public BaseEffect
  {
  public:
    Split(const string& name, const string& config, u32 id);
    ~Split();
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

    ObjectHandle _particleTexture;
    GpuBundle _particleBundle;
    ConstantBufferBundle<void, void, cb::TrailParticleG> _cbParticle;

    GpuBundle _backgroundBundle;
    ConstantBufferBundle<void, cb::TrailBackgroundF> _cbBackground;

    GpuBundle _compositeBundle;
    ConstantBufferBundle<void, cb::TrailCompositeP> _cbComposite;

    GpuBundle _lineBundle;
    ConstantBufferBundle<void, cb::TrailLinesPS, cb::TrailLinesGS> _cbPlexus;

    SplitSettings _settings;
    FreeflyCamera _camera;
  };
}