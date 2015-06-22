#pragma once

#include "tano_math.hpp"

namespace tano
{
  struct UpdateState;
  struct ParticleKinematics;

  //------------------------------------------------------------------------------
  struct DynParticles
  {
    ~DynParticles();
    void Init(int numBodies);
    void Reset();
    void AddKinematics(ParticleKinematics* kinematics, float weight);
    void UpdateWeight(ParticleKinematics* kinematics, float weight);
    void Update(const UpdateState& updateState);

    struct DistMatrix
    {
      float dist;
    };

    struct Bodies
    {
      int numBodies = 0;
      V3* pos = nullptr;
      V3* vel = nullptr;
      V3* acc = nullptr;
      V3* force = nullptr;
      DistMatrix* distMatrix = nullptr;
      //V3 pos = { 0, 0, 0 };
      //V3 vel = { 0, 0, 0 };
      //V3 acc = { 0, 0, 0 };
      //V3 force = { 0, 0, 0 };
      //float mass = 1.f;
    };

    struct Kinematics
    {
      float weight;
      ParticleKinematics* kinematics;
    };

    //Body* begin() { return _bodies ? &_bodies[0] : nullptr; }
    //Body* end() { return _bodies ? &_bodies[_numBodies] : nullptr; }

    //const Body* begin() const { return _bodies ? &_bodies[0] : nullptr; }
    //const Body* end() const { return _bodies ? &_bodies[_numBodies] : nullptr; }

    struct Kinematic
    {
      ParticleKinematics* kinematic;
      float weight;
    };

    vector<Kinematic> _kinematics;
    Bodies _bodies;
    //Body* _bodies = nullptr;
    //V3* _bodyPos = nullptr;
    //int _numBodies = 0;
    V3 _center = { 0, 0, 0 };
    float _maxSpeed = 10.f;
  };

  //------------------------------------------------------------------------------
  struct ParticleKinematics
  {
    ParticleKinematics(float maxForce, float maxSpeed) : maxForce(maxForce), maxSpeed(maxSpeed) {}

    virtual void Update(DynParticles::Bodies* bodies, float weight, const UpdateState& state) = 0;
    float maxForce = 10.f;
    float maxSpeed = 10.f;
  };

  //------------------------------------------------------------------------------
  struct BehaviorSeek : public ParticleKinematics
  {
    BehaviorSeek(float maxForce, float maxSpeed) : ParticleKinematics(maxForce, maxSpeed) {}
    virtual void Update(DynParticles::Bodies* bodies, float weight, const UpdateState& state) override;
    V3 target = { 0, 0, 0 };
  };

  //------------------------------------------------------------------------------
  struct BehaviorSeparataion : public ParticleKinematics
  {
    BehaviorSeparataion(float maxForce, float maxSpeed, float separationDistance) 
      : ParticleKinematics(maxForce, maxSpeed), separationDistance(separationDistance) {}
    virtual void Update(DynParticles::Bodies* bodies, float weight, const UpdateState& state) override;
    float separationDistance = 10;
  };

  //------------------------------------------------------------------------------
  struct BehaviorCohesion : public ParticleKinematics
  {
    BehaviorCohesion(float maxForce, float maxSpeed, float cohesionDistance) 
    : ParticleKinematics(maxForce, maxSpeed), cohesionDistance(cohesionDistance) {}
    virtual void Update(DynParticles::Bodies* bodies, float weight, const UpdateState& state) override;
    float cohesionDistance = 10;
  };

  //------------------------------------------------------------------------------
  struct BehaviorAlignment : public ParticleKinematics
  {
    BehaviorAlignment(float maxForce, float maxSpeed, float cohesionDistance) 
    : ParticleKinematics(maxForce, maxSpeed), cohesionDistance(cohesionDistance) {}
    virtual void Update(DynParticles::Bodies* bodies, float weight, const UpdateState& state) override;
    float cohesionDistance = 10;
  };
}
