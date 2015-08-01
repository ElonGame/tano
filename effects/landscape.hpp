#pragma once

#include "../base_effect.hpp"
#include "../gpu_objects.hpp"
#include "../generated/demo.types.hpp"
#include "../mesh_loader.hpp"
#include "../animation_helpers.hpp"
#include "../camera.hpp"
#include "../dyn_particles.hpp"
#include "../tano_math.hpp"
#include "../shaders/out/landscape.lensflare_pslensflare.cbuffers.hpp"
#include "../shaders/out/landscape.sky_pssky.cbuffers.hpp"
#include "../shaders/out/landscape.composite_pscomposite.cbuffers.hpp"
#include "../shaders/out/landscape.landscape_gslandscape.cbuffers.hpp"
#include "../shaders/out/landscape.landscape_vslandscape.cbuffers.hpp"
#include "../shaders/out/landscape.particle_gsparticle.cbuffers.hpp"
#include "../shaders/out/landscape.particle_psparticle.cbuffers.hpp"

namespace tano
{
  namespace scheduler
  {
    struct TaskData;
  }

  struct BehaviorLandscapeFollow : public ParticleKinematics
  {
    BehaviorLandscapeFollow(float maxForce, float maxSpeed) : ParticleKinematics(maxForce, maxSpeed) {}
    virtual void Update(
      DynParticles::Bodies* bodies, int start, int end, float weight, const FixedUpdateState& state) override;
  };

  class Landscape : public BaseEffect
  {
  public:

    enum { 
      CHUNK_SIZE = 32,
      HALF_CHUNK_SIZE = CHUNK_SIZE / 2,
    };

    Landscape(const string &name, const string& config, u32 id);
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

  //private:

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
      Vector3 center;
      float dist = 0;
      int lastAccessed = 0;
      enum {
        UPPER_INDICES = HALF_CHUNK_SIZE * HALF_CHUNK_SIZE * 2 * 3,
        LOWER_INDICES = CHUNK_SIZE * CHUNK_SIZE * 2 * 3, 
        UPPER_VERTS = HALF_CHUNK_SIZE * HALF_CHUNK_SIZE * 4,
        LOWER_VERTS = CHUNK_SIZE * CHUNK_SIZE * 4,
        // pos/normal, 3 components
        UPPER_DATA_SIZE = UPPER_VERTS * 2 * 3,
        LOWER_DATA_SIZE = LOWER_VERTS * 2 * 3,
      };
      V3 noiseValues[(CHUNK_SIZE+1)*(CHUNK_SIZE+1)];
      float upperData[UPPER_DATA_SIZE];
      float lowerData[LOWER_DATA_SIZE];
      int id;
      static int nextId;
    };

    static void FillChunk(const scheduler::TaskData& data);
    static void UpdateFlock(const scheduler::TaskData& data);

    struct FlockKernelData
    {
      Flock* flock;
      float waypointRadius;
      FixedUpdateState updateState;
    };

    struct ChunkKernelData
    {
      Chunk* chunk;
      float x, z;
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
      Flock(const BoidSettings& settings);
      ~Flock();
      DynParticles boids;
      BehaviorSeek* seek = nullptr;
    };

    SimpleAppendBuffer<Flock*, 128> _flocks;

    ConstantBufferBundle<void, cb::LandscapeLensflareP> _cbLensFlare;
    ConstantBufferBundle<void, cb::LandscapeCompositeF> _cbComposite;
    ConstantBufferBundle<void, cb::LandscapeSkyF> _cbSky;
    ConstantBufferBundle<cb::LandscapeLandscapeV, void, cb::LandscapeLandscapeG> _cbLandscape;
    ConstantBufferBundle<void, cb::LandscapeParticleP, cb::LandscapeParticleG> _cbParticle;

    GpuBundle _landscapeBundle;
    GpuState _landscapeState;
    GpuObjects _landscapeGpuObjects;
    GpuState _landscapeLowerState;

    LandscapeSettings _settings;
    MeshLoader _meshLoader;

    GpuBundle _skyBundle;
    GpuBundle _compositeBundle;
    GpuBundle _lensFlareBundle;

    ObjectHandle _particleTexture;
    GpuBundle _particleBundle;

    enum DrawFlags {
      DrawUpper       = 0x1,
      DrawLower       = 0x2,
      DrawParticles   = 0x4,
    };
    u32 _drawFlags    = 0x7;

    u32 _numUpperIndices = 0;
    u32 _numLowerIndices = 0;
    u32 _numParticles = 0;

    GpuBundle _boidsBundle;
    bool _renderLandscape = true;
    bool _renderBoids = true;
    bool _useFreeFlyCamera = false;

    BehaviorSeparataion* _behaviorSeparataion = nullptr;
    BehaviorCohesion* _behaviorCohesion = nullptr;
    BehaviorAlignment* _behaviorAlignment = nullptr;
    BehaviorLandscapeFollow* _landscapeFollow = nullptr;

    FreeFlyCamera _freeflyCamera;
    FollowCam _followCamera;
    Camera* _curCamera = &_followCamera;
    int _followFlock = 0;
  };
}
