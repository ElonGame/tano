#pragma once

#include "../effect.hpp"
#include "../gpu_objects.hpp"
#include "../generated/demo.types.hpp"
#include "../mesh_loader.hpp"
#include "../animation_helpers.hpp"
#include "../camera.hpp"
#include "../dyn_particles.hpp"
#include "../tano_math.hpp"

namespace tano
{
  namespace scheduler
  {
    struct TaskData;
  }

  struct BehaviorLandscapeFollow : public ParticleKinematics
  {
    BehaviorLandscapeFollow(float maxForce, float maxSpeed) : ParticleKinematics(maxForce, maxSpeed) {}
    void Update(DynParticles::Body* bodies, int numBodies, float weight, const UpdateState& state);
  };

  struct LandscapeOverlay
  {
    LandscapeOverlay();
    void Create(int x, int y, float amp, int size);
    void Update();
    void BlurLine(float* x, float scale, int m, float alpha, float* y);
    enum { SIZE = 50 };
    float scratch[SIZE];
    float data[2][SIZE*SIZE];
    int curBuf = 0;
    float x = 0;
    float z = 0;
  };

  class Landscape : public Effect
  {
  public:

    enum { 
      CHUNK_SIZE = 32,
      HALF_CHUNK_SIZE = CHUNK_SIZE / 2,
    };

    Landscape(const string &name, u32 id);
    ~Landscape();
    virtual bool Init(const char* configFile) override;
    virtual bool Update(const UpdateState& state) override;
    virtual bool Render() override;
    virtual bool Close() override;
    virtual bool InitAnimatedParameters() override;

    static const char* Name();
    static Effect* Create(const char* name, u32 id);
    static void Register();

  private:

    void Reset();
#if WITH_IMGUI
    void RenderParameterSet();
    void SaveParameterSet();
#endif

    void UpdateCameraMatrix(const UpdateState& state);
    void RasterizeLandscape();

    void InitBoids();
    void UpdateBoids(const UpdateState& state);

    struct Boid;
    Vector3 LandscapeFollow(const Boid& boid);

    struct Flock;

    struct Chunk
    {
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
    };

    static void FillChunk(const scheduler::TaskData& data);

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
      map<pair<float, float>, Chunk*> _chunkLookup;
    };

    ChunkCache _chunkCache;
    int _curTick = 0;

    struct Flock
    {
      Flock(int numBoids);
      DynParticles boids;
      Vector3 nextWaypoint;
      float wanderAngle = 0;
    };

    vector<Flock*> _flocks;

    struct CBufferPerFrame
    {
      Matrix world;
      Matrix view;
      Matrix proj;
      Matrix viewProj;
      Vector4 time;
      Vector4 dim;
      Vector3 cameraPos;
      float pad1;
      Vector3 cameraLookAt;
      float pad2;
      Vector3 cameraUp;
    };

    ConstantBuffer<CBufferPerFrame> _cbPerFrame;

    GpuState _landscapeState;
    GpuState _landscapeLowerState;
    GpuObjects _landscapeGpuObjects;

    LandscapeSettings _settings;
    string _configName;
    MeshLoader _meshLoader;

    GpuObjects _skyGpuObjects;

    GpuState _compositeState;
    GpuObjects _compositeGpuObjects;

    ObjectHandle _particleTexture;
    GpuState _particleState;
    GpuObjects _particleGpuObjects;

    AnimatedInt _blinkFace;

    enum DrawFlags {
      DrawUpper = 1,
      DrawLower = 2,
      DrawParticles = 4,
    };
    u32 _drawFlags = 7;

    u32 _numUpperIndices = 0;
    u32 _numLowerIndices = 0;
    u32 _numParticles = 0;

    GpuObjects _boidsMesh;
    bool _renderLandscape = true;
    bool _renderBoids = false;
    bool _useFreeFlyCamera = true;

    BehaviorSeek* _behaviorSeek = nullptr;
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
