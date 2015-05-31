#include "dyn_particles.hpp"
#include "update_state.hpp"
#include "tano_math.hpp"

using namespace tano;
using namespace bristol;

//------------------------------------------------------------------------------
DynParticles::~DynParticles()
{
  SAFE_ADELETE(_bodies);
}

//------------------------------------------------------------------------------
void DynParticles::Init(int numBodies)
{
  SAFE_ADELETE(_bodies);
  _numBodies = numBodies;
  _bodies = new Body[_numBodies];
}

//------------------------------------------------------------------------------
void DynParticles::Update(const UpdateState& updateState)
{
  if (!_numBodies)
    return;

  Vector3 center(0,0,0);
  for (int i = 0; i < _numBodies; ++i)
  {
    _bodies[i].force = { 0, 0, 0 };
    center += _bodies[i].pos;
  }

  _center = center /  (float)_numBodies;

  for (Kinematic& k : _kinematics)
  {
    k.kinematic->Update(_bodies, _numBodies, k.weight, updateState);
  }

  float dt = 1.0f / updateState.frequency;
  for (int i = 0; i < _numBodies; ++i)
  {
    Body& b = _bodies[i];
    b.acc = b.force;
    b.vel = ClampVector(b.vel + dt * b.acc, _maxSpeed);
    b.pos += dt * b.vel;
  }
}

//------------------------------------------------------------------------------
void DynParticles::AddKinematics(ParticleKinematics* kinematics, float weight)
{
  _kinematics.push_back({kinematics, weight});
}

//------------------------------------------------------------------------------
void DynParticles::UpdateWeight(ParticleKinematics* kinematics, float weight)
{
  for (Kinematic& k : _kinematics)
  {
    if (k.kinematic == kinematics)
    {
      k.weight = weight;
      return;
    }
  }
}

//------------------------------------------------------------------------------
void BehaviorSeek::Update(DynParticles::Body* bodies, int numBodies, float weight, const UpdateState& state)
{
  for (int i = 0; i < numBodies; ++i)
  {
    DynParticles::Body* b = &bodies[i];

    Vector3 desiredVel = Normalize(target - b->pos) * maxSpeed;
    b->force += weight * ClampVector(desiredVel - b->vel, maxForce);
  }
}

//------------------------------------------------------------------------------
void BehaviorSeparataion::Update(DynParticles::Body* bodies, int numBodies, float weight, const UpdateState& state)
{
  for (int i = 0; i < numBodies; ++i)
  {
    DynParticles::Body* b = &bodies[i];

    // return a force away from any close boids
    Vector3 avg(0, 0, 0);

    float cnt = 0.f;
    for (int j = 0; j < numBodies; ++j)
    {
      if (i == j)
        continue;

      DynParticles::Body* inner = &bodies[j];

      float dist = Vector3::Distance(inner->pos, b->pos);
      if (dist > separationDistance)
        continue;

      avg += 1.f / dist * Normalize(b->pos - inner->pos);
      cnt += 1.f;
    }

    if (cnt == 0.f)
      continue;

    avg /= cnt;

    // Reynolds uses: steering = desired - current
    Vector3 desired = Normalize(avg) * maxSpeed;
    b->force += weight * ClampVector(desired - b->vel, maxForce);
  }
}

//------------------------------------------------------------------------------
void BehaviorCohesion::Update(DynParticles::Body* bodies, int numBodies, float weight, const UpdateState& state)
{
  for (int i = 0; i < numBodies; ++i)
  {
    DynParticles::Body* b = &bodies[i];

    // Return a force towards the average boid position
    Vector3 avg(0, 0, 0);

    float cnt = 0.f;

    for (int j = 0; j < numBodies; ++j)
    {
      if (i == j)
        continue;

      DynParticles::Body* inner = &bodies[j];

      float dist = Vector3::Distance(inner->pos, b->pos);
      if (dist > cohesionDistance)
        continue;

      avg += inner->pos;
      cnt += 1.f;
    }

    if (cnt == 0.f)
      continue;

    avg /= cnt;

    Vector3 desiredVel = Normalize(avg - b->pos) * maxSpeed;
    b->force += weight * ClampVector(desiredVel - b->vel, maxForce);
  }
}

//------------------------------------------------------------------------------
void BehaviorAlignment::Update(DynParticles::Body* bodies, int numBodies, float weight, const UpdateState& state)
{
  for (int i = 0; i < numBodies; ++i)
  {
    DynParticles::Body* b = &bodies[i];

    // return a force to align the boids velocity with the average velocity
    Vector3 avg(0, 0, 0);

    float cnt = 0.f;
    for (int j = 0; j < numBodies; ++j)
    {
      if (i == j)
        continue;

      DynParticles::Body* inner = &bodies[j];

      float dist = Vector3::Distance(inner->pos, b->pos);
      if (dist > cohesionDistance)
        continue;

      avg += inner->vel;
      cnt += 1.f;
    }

    if (cnt == 0.f)
      continue;

    avg /= cnt;
    Vector3 desired = Normalize(avg) * maxSpeed;
    b->force += weight * ClampVector(desired - b->vel, maxForce);
  }
}
