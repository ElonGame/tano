#include "dyn_particles.hpp"
#include "update_state.hpp"
#include "tano_math.hpp"
#include "arena_allocator.hpp"

using namespace tano;
using namespace bristol;

//------------------------------------------------------------------------------
DynParticles::~DynParticles()
{
  SAFE_ADELETE(_bodies);
  SAFE_ADELETE(_bodyPos);
}

//------------------------------------------------------------------------------
void DynParticles::Init(int numBodies)
{
  SAFE_ADELETE(_bodies);
  SAFE_ADELETE(_bodyPos);
  _numBodies = numBodies;
  _bodies = new Body[_numBodies];
  _bodyPos = new V3[_numBodies];
}

//------------------------------------------------------------------------------
void DynParticles::Update(const UpdateState& updateState)
{
  if (!_numBodies)
    return;

  int numBodies = _numBodies;
  V3 center(0,0,0);
  for (int i = 0; i < numBodies; ++i)
  {
    _bodies[i].force = { 0, 0, 0 };
    //center += _bodies[i].pos;
    center += _bodyPos[i];
  }

  _center = 1.0f / numBodies * center;

  // precompute particle distance calculations
  float* distMatrix = (float*)ARENA.Alloc(numBodies*numBodies*sizeof(float));
  for (int i = 0; i < numBodies; ++i)
  {
    for (int j = i; j < numBodies; ++j)
    {
      if (i != j)
      {
        //float tmp = Distance(_bodies[i].pos, _bodies[j].pos);
        float tmp = Distance(_bodyPos[i], _bodyPos[j]);
        distMatrix[i*numBodies + j] = tmp;
        distMatrix[j*numBodies + i] = tmp;
      }
      else
      {
        distMatrix[i*numBodies + j] = 0;
      }
    }
  }


  for (Kinematic& k : _kinematics)
  {
    k.kinematic->Update(_bodies, _bodyPos, _numBodies, k.weight, updateState, distMatrix);
  }

  float dt = 1.0f / updateState.frequency;
  for (int i = 0; i < numBodies; ++i)
  {
    Body& b = _bodies[i];
    b.acc = b.force;
    b.vel = ClampVector(b.vel + dt * b.acc, _maxSpeed);
    _bodyPos[i] += dt * b.vel;
//    b.pos += dt * b.vel;
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
void BehaviorSeek::Update(
    DynParticles::Body* bodies,
    const V3* bodyPos,
    int numBodies,
    float weight,
    const UpdateState& state,
    const float* distMatrix)
{
  for (int i = 0; i < numBodies; ++i)
  {
    DynParticles::Body* b = &bodies[i];

    //V3 desiredVel = Normalize(target - b->pos) * maxSpeed;
    V3 desiredVel = Normalize(target - bodyPos[i]) * maxSpeed;
    b->force += weight * ClampVector(desiredVel - b->vel, maxForce);
  }
}

//------------------------------------------------------------------------------
void BehaviorSeparataion::Update(
    DynParticles::Body* bodies,
    const V3* bodyPos,
    int numBodies,
    float weight,
    const UpdateState& state,
    const float* distMatrix)
{
  for (int i = 0; i < numBodies; ++i)
  {
    DynParticles::Body* b = &bodies[i];

    // return a force away from any close boids
    V3 avg(0, 0, 0);

    float cnt = 0.f;
    for (int j = 0; j < numBodies; ++j)
    {
      if (i == j)
        continue;

      float dist = distMatrix[i*numBodies+j];
      if (dist > separationDistance)
        continue;

      float invDist = 1.0f / dist;
      avg += (bodyPos[i] - bodyPos[j]) * (invDist * invDist);
      cnt += 1.f;
    }

    if (cnt == 0.f)
      continue;

    avg /= cnt;

    // Reynolds uses: steering = desired - current
    V3 desired = Normalize(avg) * maxSpeed;
    b->force += weight * ClampVector(desired - b->vel, maxForce);
  }
}

//------------------------------------------------------------------------------
void BehaviorCohesion::Update(
    DynParticles::Body* bodies,
    const V3* bodyPos,
    int numBodies,
    float weight,
    const UpdateState& state,
    const float* distMatrix)
{
  for (int i = 0; i < numBodies; ++i)
  {
    DynParticles::Body* b = &bodies[i];

    // Return a force towards the average boid position
    V3 avg(0, 0, 0);

    float cnt = 0.f;

    for (int j = 0; j < numBodies; ++j)
    {
      if (i == j)
        continue;

      float dist = distMatrix[i*numBodies + j];
      if (dist > cohesionDistance)
        continue;

      DynParticles::Body* inner = &bodies[j];

//      avg += inner->pos;
      avg += bodyPos[j];
      cnt += 1.f;
    }

    if (cnt == 0.f)
      continue;

    avg /= cnt;

//    V3 desiredVel = Normalize(avg - b->pos) * maxSpeed;
    V3 desiredVel = Normalize(avg - bodyPos[i]) * maxSpeed;
    b->force += weight * ClampVector(desiredVel - b->vel, maxForce);
  }
}

//------------------------------------------------------------------------------
void BehaviorAlignment::Update(
    DynParticles::Body* bodies,
    const V3* bodyPos,
    int numBodies,
    float weight,
    const UpdateState& state,
    const float* distMatrix)
{
  for (int i = 0; i < numBodies; ++i)
  {
    DynParticles::Body* b = &bodies[i];

    // return a force to align the boids velocity with the average velocity
    V3 avg(0, 0, 0);

    float cnt = 0.f;
    for (int j = 0; j < numBodies; ++j)
    {
      if (i == j)
        continue;

      float dist = distMatrix[i*numBodies + j];
      if (dist > cohesionDistance)
        continue;

      DynParticles::Body* inner = &bodies[j];

      avg += inner->vel;
      cnt += 1.f;
    }

    if (cnt == 0.f)
      continue;

    avg /= cnt;
    V3 desired = Normalize(avg) * maxSpeed;
    b->force += weight * ClampVector(desired - b->vel, maxForce);
  }
}
