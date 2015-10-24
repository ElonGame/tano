#pragma once

#include "../base_effect.hpp"
#include "../gpu_objects.hpp"
#include "../generated/demo.types.hpp"
#include "../mesh_loader.hpp"
#include "../animation_helpers.hpp"
#include "../camera.hpp"
#include "../dyn_particles.hpp"
#include "../tano_math.hpp"
#include "../random.hpp"
#include "../shaders/out/landscape.lensflare_pslensflare.cbuffers.hpp"
#include "../shaders/out/landscape.sky_pssky.cbuffers.hpp"
#include "../shaders/out/landscape.composite_pscomposite.cbuffers.hpp"
#include "../shaders/out/landscape.landscape_gslandscape.cbuffers.hpp"
#include "../shaders/out/landscape.landscape_vslandscape.cbuffers.hpp"
#include "../shaders/out/landscape.landscape_pslandscape.cbuffers.hpp"
#include "../shaders/out/landscape.particle_gsparticle.cbuffers.hpp"
#include "../shaders/out/landscape.particle_psparticle.cbuffers.hpp"
#include "../random.hpp"

namespace tano
{
  namespace scheduler
  {
    struct TaskData;
  }

  struct BehaviorPathFollow : public ParticleKinematics
  {
    BehaviorPathFollow(const CardinalSpline& spline);
    virtual void Update(const UpdateParams& params) override;

    vector<float> _splineOffset;
    const CardinalSpline& _spline;
  };

  struct BehaviorLandscapeFollow : public ParticleKinematics
  {
    virtual void Update(const ParticleKinematics::UpdateParams& params) override;
  };

  struct BehaviorSpacing : public ParticleKinematics
  {
    virtual void Update(const ParticleKinematics::UpdateParams& params) override;

  };

  class Landscape : public BaseEffect
  {
  public:
    enum
    {
      NUM_CHUNK_QUADS = 32,
      NUM_CHUNK_VERTS = NUM_CHUNK_QUADS + 1,
      HALF_CHUNK_SIZE = NUM_CHUNK_VERTS / 2,

      UPPER_NUM_CHUNK_QUADS = NUM_CHUNK_QUADS / 2,
      UPPER_NUM_CHUNK_VERTS = UPPER_NUM_CHUNK_QUADS + 1,
    };

    Landscape(const string& name, const string& config, u32 id);
    ~Landscape();
    virtual bool OnConfigChanged(const vector<char>& buf) override;
    virtual bool Init() override;
    virtual bool Update(const UpdateState& state) override;
    virtual bool FixedUpdate(const FixedUpdateState& state) override;
    virtual bool Render() override;
    virtual bool Close() override;
    virtual bool InitAnimatedParameters() override;
    virtual const char* GetName() { return Name(); }

    static const char* Name();
    static BaseEffect* Create(const char* name, const char* config, u32 id);
    static void Register();

    // private:

    void Reset();
#if WITH_IMGUI
    void RenderParameterSet();
    void SaveParameterSet(bool inc);
#endif

    void UpdateCameraMatrix(const UpdateState& state);
    void RasterizeLandscape();

    void InitBoids();
    void UpdateBoids(const FixedUpdateState& state);
    void RenderBoids(const ObjectHandle* renderTargets, ObjectHandle dsHandle);

    struct Flock;

    struct Chunk
    {
      Chunk() : id(nextId++) {}
      float x, y;
      vec3 center;
      float dist = 0;
      int lastAccessed = 0;
      enum
      {
        UPPER_INDICES = UPPER_NUM_CHUNK_QUADS * UPPER_NUM_CHUNK_QUADS * 6,
        LOWER_INDICES = NUM_CHUNK_QUADS * NUM_CHUNK_QUADS * 6,
        UPPER_VERTS = UPPER_NUM_CHUNK_VERTS * UPPER_NUM_CHUNK_VERTS,
        LOWER_VERTS = NUM_CHUNK_VERTS * NUM_CHUNK_VERTS,
      };
      vec3 noiseValues[NUM_CHUNK_VERTS * NUM_CHUNK_VERTS];
      vec3 upperData[UPPER_NUM_CHUNK_VERTS * UPPER_NUM_CHUNK_VERTS];
      vec3 lowerData[NUM_CHUNK_VERTS * NUM_CHUNK_VERTS];
      int id;
      static int nextId;
    };

    struct ChunkKernelData
    {
      Chunk* chunk;
      float x, z;
    };

    struct CopyKernelData
    {
      const Chunk* chunk;
      vec3* lowerBuf;
      vec3* upperBuf;
      vec3* particleBuf;
    };

    static void FillChunk(const scheduler::TaskData& data);
    static void CopyOutTask(const scheduler::TaskData& data);
    static void UpdateFlock(const scheduler::TaskData& data);

    struct FlockKernelData
    {
      Flock* flock;
      vec3 target;
      float waypointRadius;
      float deltaTime;
    };

    struct ChunkCache
    {
      Chunk* FindChunk(float x, float y, int timestamp);
      Chunk* GetFreeChunk(float x, float y, int timestamp);
      static const int CACHE_SIZE = 2048;
      Chunk _cache[CACHE_SIZE];
      int _used = 0;
      unordered_map<pair<float, float>, Chunk*> _chunkLookup;
    };

    ChunkCache _chunkCache;
    int _curTick = 0;

    struct Flock
    {
      Flock(const BoidSettings& settings, const CardinalSpline& spline);
      ~Flock();
      DynParticles boids;
      BehaviorSeek* seek = nullptr;
      BehaviorPathFollow* follow = nullptr;
    };

    struct FlockCamera : public Camera
    {
      virtual void Update(float deltaTime) override;
      Flock* flock;
    };

    SimpleAppendBuffer<Flock*, 128> _flocks;

    ConstantBufferBundle<void, cb::LandscapeLensflareP> _cbLensFlare;
    ConstantBufferBundle<void, cb::LandscapeCompositeF> _cbComposite;

    ConstantBufferBundle<void, cb::LandscapeSkyP> _cbSky;
    ConstantBufferBundle<cb::LandscapeLandscapeV, cb::LandscapeLandscapeP, cb::LandscapeLandscapeG>
        _cbLandscape;
    ConstantBufferBundle<void, cb::LandscapeParticleP, cb::LandscapeParticleG> _cbParticle;

    GpuBundle _landscapeLowerBundle;
    GpuBundle _landscapeUpperBundle;

    LandscapeSettings _settings;
    MeshLoader _meshLoader;

    GpuBundle _skyBundle;
    GpuBundle _compositeBundle;
    GpuBundle _lensFlareBundle;

    ObjectHandle _particleTexture;
    ObjectHandle _boidsTexture;
    GpuBundle _particleBundle;

    enum DrawFlags
    {
      DrawUpper = 0x1,
      DrawLower = 0x2,
      DrawParticles = 0x4,
    };
    u32 _drawFlags = 0x7;

    u32 _numUpperIndices = 0;
    u32 _numLowerIndices = 0;
    u32 _numParticles = 0;
    u32 _numChunks = 0;

    GpuBundle _boidsBundle;
    bool _renderLandscape = true;
    bool _renderBoids = true;

    BehaviorSeparataion* _behaviorSeparataion = nullptr;
    BehaviorCohesion* _behaviorCohesion = nullptr;
    BehaviorLandscapeFollow* _behaviorLandscapeFollow = nullptr;
    BehaviorSpacing* _behaviorSpacing = nullptr;

    FlockCamera _flockCamera;
    Camera* _curCamera = &_flockCamera;
    //Camera* _curCamera = &_freeflyCamera;
    int _followFlock = 0;

    CardinalSpline _spline;
    int _curFlockIdx = -1;
    float _flockFade = 1;

    RandomUniform _random;
    RandomInt _randomInt;
  };
}
