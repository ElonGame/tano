#pragma once

#include "tano_math.hpp"

namespace tano
{
  struct UpdateState;
  struct ParticleKinematics;

  //------------------------------------------------------------------------------
  struct DynParticles
  {
    enum DistMeasureType
    {
      DistCohesion,
      DistSeperation,
      DistCount
    };

    ~DynParticles();
    void Init(int numBodies);
    void Reset();
    void AddKinematics(ParticleKinematics* kinematics, float weight);
    void UpdateWeight(ParticleKinematics* kinematics, float weight);
    void Update(const UpdateState& updateState, bool alwaysUpdate);
    void UpdateDistMatrix(int start, int end);
    void SetDistCutOff(DistMeasureType type, float cutoff);

    struct DistMatrix
    {
      float dist;
      float invDist;
    };

    struct DistMeasureEntry
    {
      int idx;
      float dist;
      float invDist;
    };

    struct DistMeasure
    {
      float cutoff = 0;
      DistMeasureEntry* values = nullptr;
    };

    struct Bodies
    {
      int numBodies = 0;
      XMVECTOR* pos = nullptr;
      XMVECTOR* vel = nullptr;
      XMVECTOR* acc = nullptr;
      XMVECTOR* force = nullptr;

      DistMeasure distMeasures[DistCount];
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
    XMVECTOR _center = XMVectorZero();
    float _maxSpeed = 10.f;
    int _tickCount = 0;
  };

  //------------------------------------------------------------------------------
  struct ParticleKinematics
  {
    ParticleKinematics(float maxForce, float maxSpeed) : maxForce(maxForce), maxSpeed(maxSpeed) {}

    virtual void Update(
      DynParticles::Bodies* bodies,
      int start, int end,
      float weight, 
      const UpdateState& state) = 0;
    float maxForce = 10.f;
    float maxSpeed = 10.f;
  };

  //------------------------------------------------------------------------------
  struct BehaviorSeek : public ParticleKinematics
  {
    BehaviorSeek(float maxForce, float maxSpeed) : ParticleKinematics(maxForce, maxSpeed) {}
    virtual void Update(
      DynParticles::Bodies* bodies, 
      int start, int end,
      float weight, 
      const UpdateState& state) override;
    XMVECTOR target = XMVectorZero();
  };

  //------------------------------------------------------------------------------
  struct BehaviorSeparataion : public ParticleKinematics
  {
    BehaviorSeparataion(float maxForce, float maxSpeed, float separationDistance) 
      : ParticleKinematics(maxForce, maxSpeed), separationDistance(separationDistance) {}
    virtual void Update(
      DynParticles::Bodies* bodies, 
      int start, int end,
      float weight, 
      const UpdateState& state) override;
    float separationDistance = 10;
  };

  //------------------------------------------------------------------------------
  struct BehaviorCohesion : public ParticleKinematics
  {
    BehaviorCohesion(float maxForce, float maxSpeed, float cohesionDistance) 
    : ParticleKinematics(maxForce, maxSpeed), cohesionDistance(cohesionDistance) {}
    virtual void Update(
      DynParticles::Bodies* bodies, 
      int start, int end,
      float weight, 
      const UpdateState& state) override;
    float cohesionDistance = 10;
  };

  //------------------------------------------------------------------------------
  struct BehaviorAlignment : public ParticleKinematics
  {
    BehaviorAlignment(float maxForce, float maxSpeed, float cohesionDistance) 
    : ParticleKinematics(maxForce, maxSpeed), cohesionDistance(cohesionDistance) {}
    virtual void Update(
      DynParticles::Bodies* bodies, 
      int start, int end,
      float weight, 
      const UpdateState& state) override;
    float cohesionDistance = 10;
  };
}
