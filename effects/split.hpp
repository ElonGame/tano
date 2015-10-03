#pragma once
#include "../base_effect.hpp"
#include "../camera.hpp"
#include "../generated/demo.types.hpp"
#include "../gpu_objects.hpp"
#include "../shaders/out/split.compose_pscomposite.cbuffers.hpp"
#include "../shaders/out/trail.background_psbackground.cbuffers.hpp"
#include "../shaders/out/split.mesh_vsmesh.cbuffers.hpp"
#include "../shaders/out/split.mesh_psmeshtrans.cbuffers.hpp"
#include "../shaders/out/split.sky_pssky.cbuffers.hpp"
#include "../shaders/out/split.particle_gsparticle.cbuffers.hpp"
#include "../mesh_loader.hpp"

namespace tano
{
  struct PN
  {
    vec3 pos;
    vec3 normal;
  };

  struct Pathy
  {
    enum
    {
      TOTAL_POINTS = 32 * 1024,
      POINTS_PER_CHILD = 1024,
      MAX_NUM_LINES = 32
    };
    void Create(const MeshLoader& meshLoader);

    void CreateTubesIncremental(float t);

    struct Particle
    {
      float pos;
      float speed;
      float spawnTime;
      float fade;
    };

    struct Segment
    {
      Segment(const vec3& cur, float scale, float speed, float angleX, float angleY, float angleZ)
          : cur(cur), scale(scale), speed(speed), angleX(angleX), angleY(angleY), angleZ(angleZ)
      {
      }
      vec3 cur;
      float speed;
      float scale;
      float angleX, angleY, angleZ;
      int lastNumTicks = 0;
      bool isStarted = false;
      vector<vec3> verts;
      vector<PN> completeRings;
      vector<PN> inprogressRing;
      CardinalSpline spline;

      float lastSpawn = 0;
      vector<Particle> particles;

      // last reference frame
      vec3 frameD, frameN, frameT;
    };

    float len = 5;

    float angleXMean = 0.f;
    float angleXVariance = 1.f;

    float angleYMean = 0.f;
    float angleYVariance = 1.f;

    float angleZMean = 0.f;
    float angleZVariance = 1.f;

    // TODO(magnus): fix the popping here
    float childProb = 1.0f;
    float childScale = 0.50f;

    int maxChildren = 512;

    struct Line
    {
      int startOfs;
      int size;
    };

    struct SegmentStart
    {
      float time;
      Segment* segment;
    };
    deque<SegmentStart> segmentStart;
    vector<Segment*> segments;

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

    void UpdateParticles(const UpdateState& state);

    ObjectHandle _meshBackFace;
    ObjectHandle _meshFrontFace;

    Pathy _pathy;

    GpuBundle _backgroundBundle;
    ConstantBufferBundle<void, cb::TrailBackgroundF> _cbBackground;

    GpuBundle _compositeBundle;
    ConstantBufferBundle<void, cb::SplitComposeP> _cbComposite;

    GpuState _meshBlockerState;
    ObjectHandle _meshBlockerPs;

    GpuBundle _meshBundle;
    ConstantBufferBundle<
      cb::SplitMeshV, cb::SplitMeshP, void,
      cb::SplitMeshO, void, void> _cbMesh;

    ConstantBufferBundle<void, cb::SplitSkyF> _cbSky;
    GpuBundle _skyBundle;

    ConstantBufferBundle<void, void, cb::SplitParticleG> _cbParticle;
    ObjectHandle _particleTexture;
    GpuBundle _particleBundle;

    SplitSettings _settings;
    MeshLoader _meshLoader;

    Camera _camera;
    Camera* _curCamera = &_camera;
  };
}