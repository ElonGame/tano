#pragma once

#include "tano_math.hpp"

namespace tano
{
  struct FixedUpdateState;
  struct ParticleKinematics;

  //------------------------------------------------------------------------------
  struct DynParticles
  {
    ~DynParticles();
    void Init(int numBodies, float maxSpeed, float maxForce);
    void Reset();
    void AddKinematics(ParticleKinematics* kinematics, float weight);
    void UpdateWeight(ParticleKinematics* kinematics, float weight);
    void Update(float deltaTime, bool alwaysUpdate);

#if WITH_IMGUI
    void DrawForcePlot();
#endif

    enum { MAX_NUM_FORCES = 5 };
    struct Bodies
    {
      Bodies() { memset(forces, 0, sizeof(forces)); }
      int numBodies = 0;
      vec3* pos = nullptr;
      vec3* vel = nullptr;
      vec3* acc = nullptr;
      vec3* forces[MAX_NUM_FORCES];
    };

    struct Kinematics
    {
      float weight;
      ParticleKinematics* kinematics;
    };

    struct Kinematic
    {
      ParticleKinematics* kinematic;
      float weight;
    };

    struct Bucket
    {
      u16 count = 0;
      u16* data = nullptr;
    };

    Bucket* _buckets = nullptr;
    vector<Bucket*> _validBuckets;

    vector<Kinematic> _kinematics;
    Bodies _bodies;
    vec3 _center = {0, 0, 0};
    float _maxSpeed = 10.f;
    float _maxForce = 10.f;
    int _tickCount = 0;
  };

  //------------------------------------------------------------------------------
  struct ParticleKinematics
  {
    struct UpdateParams
    {
      DynParticles::Bodies* bodies;
      int start, end;
      float weight;
      float deltaTime;
      const DynParticles* p;
    };

    virtual void Update(const UpdateParams& params) = 0;
    int forceIdx = 0;
  };

  //------------------------------------------------------------------------------
  struct BehaviorSeek : public ParticleKinematics
  {
    virtual void Update(const UpdateParams& params) override;
    vec3 target = vec3::Zero;
  };

  //------------------------------------------------------------------------------
  struct BehaviorSeparataion : public ParticleKinematics
  {
    BehaviorSeparataion(float separationDistance) 
      : separationDistance(separationDistance) {}
    virtual void Update(const UpdateParams& params) override;
    float separationDistance = 10;
  };

  //------------------------------------------------------------------------------
  struct BehaviorCohesion : public ParticleKinematics
  {
    BehaviorCohesion(float cohesionDistance) 
    : cohesionDistance(cohesionDistance) {}
    virtual void Update(const UpdateParams& params) override;
    float cohesionDistance = 10;
  };

  // NB: BehaviorAlignment has been removed
}
