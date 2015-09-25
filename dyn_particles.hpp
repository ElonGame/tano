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
    void Init(int numBodies);
    void Reset();
    void AddKinematics(ParticleKinematics* kinematics, float weight);
    void UpdateWeight(ParticleKinematics* kinematics, float weight);
    void Update(float deltaTime, bool alwaysUpdate);

    struct Bodies
    {
      int numBodies = 0;
      vec3* pos = nullptr;
      vec3* vel = nullptr;
      vec3* acc = nullptr;
      vec3* force = nullptr;
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

    vector<Kinematic> _kinematics;
    Bodies _bodies;
    vec3 _center = {0, 0, 0};
    float _maxSpeed = 10.f;
    int _tickCount = 0;

    struct Bucket
    {
      u16 count = 0;
      u16* data = nullptr;
    };

    Bucket* _buckets = nullptr;
    vector<Bucket*> _validBuckets;
  };

  //------------------------------------------------------------------------------
  struct ParticleKinematics
  {
    ParticleKinematics(float maxForce, float maxSpeed) : maxForce(maxForce), maxSpeed(maxSpeed) {}

    struct UpdateParams
    {
      DynParticles::Bodies* bodies;
      int start, end;
      float weight;
      float deltaTime;
      const DynParticles* p;
    };

    virtual void Update(const UpdateParams& params) = 0;
    float maxForce = 10.f;
    float maxSpeed = 10.f;
  };

  //------------------------------------------------------------------------------
  struct BehaviorSeek : public ParticleKinematics
  {
    BehaviorSeek(float maxForce, float maxSpeed) : ParticleKinematics(maxForce, maxSpeed) {}
    virtual void Update(const UpdateParams& params) override;
    vec3 target = vec3::Zero;
  };

  //------------------------------------------------------------------------------
  struct BehaviorSeparataion : public ParticleKinematics
  {
    BehaviorSeparataion(float maxForce, float maxSpeed, float separationDistance) 
      : ParticleKinematics(maxForce, maxSpeed), separationDistance(separationDistance) {}
    virtual void Update(const UpdateParams& params) override;
    float separationDistance = 10;
  };

  //------------------------------------------------------------------------------
  struct BehaviorCohesion : public ParticleKinematics
  {
    BehaviorCohesion(float maxForce, float maxSpeed, float cohesionDistance) 
    : ParticleKinematics(maxForce, maxSpeed), cohesionDistance(cohesionDistance) {}
    virtual void Update(const UpdateParams& params) override;
    float cohesionDistance = 10;
  };

  //------------------------------------------------------------------------------
  struct BehaviorAlignment : public ParticleKinematics
  {
    BehaviorAlignment(float maxForce, float maxSpeed, float cohesionDistance) 
    : ParticleKinematics(maxForce, maxSpeed), cohesionDistance(cohesionDistance) {}
    virtual void Update(const UpdateParams& params) override;
    float cohesionDistance = 10;
  };
}
