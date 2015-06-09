#pragma once

#include "../effect.hpp"
#include "../gpu_objects.hpp"
#include "../generated/demo.types.hpp"
#include "../mesh_loader.hpp"
#include "../animation_helpers.hpp"
#include "../camera.hpp"
#include "../dyn_particles.hpp"
#include "../perlin2d.hpp"

namespace tano
{
  namespace scheduler
  {
    struct TaskData;
  }

  struct BehaviorLandscapeFollow : public ParticleKinematics
  {
    BehaviorLandscapeFollow(float maxForce, float maxSpeed, const Perlin2D& perlin) 
      : ParticleKinematics(maxForce, maxSpeed), _perlin(perlin) {}
    void Update(DynParticles::Body* bodies, int numBodies, float weight, const UpdateState& state);
    const Perlin2D& _perlin;
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

    enum { CHUNK_SIZE = 32 };

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
    void RasterizeLandscape(float* buf);

    void InitBoids();
    void UpdateBoids(const UpdateState& state);

    struct Boid;
    Vector3 LandscapeFollow(const Boid& boid);

    struct Flock;

    struct Chunk
    {
      float x, y;
      int lastAccessed = 0;
      static const int DATA_SIZE = CHUNK_SIZE*CHUNK_SIZE*2*18;
      float noiseValues[(CHUNK_SIZE+1)*(CHUNK_SIZE+1)];
      float data[DATA_SIZE];
    };

    static void FillChunk(Chunk* chunk, float x, float z, const Perlin2D& perlin);

    struct ChunkKernelData
    {
      Chunk* chunk;
      const Perlin2D& perlin;
      float x, z;
    };
    static void ChunkKernel(const scheduler::TaskData& data);

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
    GpuObjects _landscapeGpuObjects;

    LandscapeSettings _settings;
    string _configName;
    MeshLoader _meshLoader;

    GpuObjects _edgeGpuObjects;
    GpuObjects _skyGpuObjects;

    GpuState _compositeState;
    GpuObjects _compositeGpuObjects;

    AnimatedInt _blinkFace;

    u32 _numVerts;

    GpuObjects _boidsMesh;
    bool _renderLandscape = true;
    bool _renderBoids = true;
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

    vector<LandscapeOverlay> _overlays;

    Perlin2D _perlin;
  };
}
