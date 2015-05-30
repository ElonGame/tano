#pragma once

namespace tano
{
  struct UpdateState;
  struct ParticleKinematics;

  struct DynParticles
  {
    ~DynParticles();
    void Init(int numBodies);
    void AddKinematics(ParticleKinematics* kinematics);
    void Update(const UpdateState& updateState);

    struct Body
    {
      Vector3 pos = { 0, 0, 0 };
      Vector3 vel = { 0, 0, 0 };
      Vector3 acc = { 0, 0, 0 };
      Vector3 force = { 0, 0, 0 };
      float mass = 1.f;
    };

    struct Kinematics
    {
      float weight;
      ParticleKinematics* kinematics;
    };

    vector<ParticleKinematics*> _kinematics;
    Body* _bodies = nullptr;
    int _numBodies = 0;
  };

  struct ParticleKinematics
  {
    virtual void Update(DynParticles::Body* bodies, int numBodies, const UpdateState& state) {}
    float maxForce = 10.f;
    float maxSpeed = 10.f;
  };

}