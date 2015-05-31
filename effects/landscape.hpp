#pragma once

#include "../effect.hpp"
#include "../gpu_objects.hpp"
#include "../generated/demo.types.hpp"
#include "../mesh_loader.hpp"
#include "../animation_helpers.hpp"
#include "../camera.hpp"
#include "../dyn_particles.hpp"

namespace tano
{
  struct BehaviorLandscapeFollow : public ParticleKinematics
  {
    BehaviorLandscapeFollow(float maxForce, float maxSpeed) : ParticleKinematics(maxForce, maxSpeed) {}
    void Update(DynParticles::Body* bodies, int numBodies, float weight, const UpdateState& state);
  };

  class Landscape : public Effect
  {
  public:

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
