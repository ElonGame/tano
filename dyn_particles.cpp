#include "dyn_particles.hpp"
#include "update_state.hpp"
#include "tano_math.hpp"
#include "arena_allocator.hpp"

using namespace tano;
using namespace bristol;

//------------------------------------------------------------------------------
DynParticles::~DynParticles()
{
  Reset();
}

//------------------------------------------------------------------------------
void DynParticles::Init(int numBodies)
{
  Reset();
  _bodies.numBodies = numBodies;
  _bodies.pos = new V3[numBodies];
  _bodies.vel = new V3[numBodies];
  _bodies.acc = new V3[numBodies];
  _bodies.force = new V3[numBodies];
  _bodies.distMatrix = new DistMatrix[numBodies*numBodies];

  V3 zero(0,0,0);
  for (int i = 0; i < numBodies; ++i)
  {
    _bodies.pos[i] = _bodies.vel[i] = _bodies.acc[i] = _bodies.force[i] = zero;
  }
}

//------------------------------------------------------------------------------
void DynParticles::Reset()
{
  SAFE_ADELETE(_bodies.pos);
  SAFE_ADELETE(_bodies.vel);
  SAFE_ADELETE(_bodies.acc);
  SAFE_ADELETE(_bodies.force);
  SAFE_ADELETE(_bodies.distMatrix);
}

//------------------------------------------------------------------------------
void DynParticles::Update(const UpdateState& updateState)
{
  if (!_bodies.numBodies)
    return;

  int numBodies = _bodies.numBodies;
  V3* pos = _bodies.pos;
  V3* acc = _bodies.acc;
  V3* vel = _bodies.vel;
  V3* force = _bodies.force;
  DynParticles::DistMatrix* distMatrix = _bodies.distMatrix;

  _tickCount++;
  int step = _tickCount % 4;
  bool updateForces = step == 0 || step == 2;
  bool updateDistances = step == 1;

  V3 center(0,0,0);
  for (int i = 0; i < numBodies; ++i)
  {
    if (updateForces)
    {
      force[i] = { 0, 0, 0 };
    }
    center += pos[i];
  }

  _center = 1.0f / numBodies * center;

  if (updateDistances)
  {
    // precompute particle distance calculations
    for (int i = 0; i < numBodies; ++i)
    {
      for (int j = i; j < numBodies; ++j)
      {
        if (i != j)
        {
          float dist = Distance(pos[i], pos[j]);
          float invDist = 1.0f / dist;
          distMatrix[i*numBodies + j].dist = dist;
          distMatrix[j*numBodies + i].dist = dist;
          distMatrix[i*numBodies + j].invDist = invDist;
          distMatrix[j*numBodies + i].invDist = invDist;
        }
        else
        {
          distMatrix[i*numBodies + j].dist = 0;
          distMatrix[i*numBodies + j].invDist = 0;
        }
      }
    }
  }

  if (updateForces)
  {
    for (Kinematic& k : _kinematics)
    {
      k.kinematic->Update(&_bodies, k.weight, updateState);
    }
  }

  float dt = 1.0f / updateState.frequency;
  for (int i = 0; i < numBodies; ++i)
  {
    _bodies.acc[i] = _bodies.force[i];
    _bodies.vel[i] = ClampVector(_bodies.vel[i] + dt * _bodies.acc[i], _maxSpeed);
    _bodies.pos[i] += dt * _bodies.vel[i];
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
void BehaviorSeek::Update(DynParticles::Bodies* bodies, float weight, const UpdateState& state)
{
  V3* pos = bodies->pos;
  V3* vel = bodies->vel;
  V3* force = bodies->force;
  int numBodies = bodies->numBodies;

  for (int i = 0; i < numBodies; ++i)
  {
    V3 desiredVel = Normalize(target - pos[i]) * maxSpeed;
    force[i] += weight * ClampVector(desiredVel - vel[i], maxForce);
  }
}

//------------------------------------------------------------------------------
void BehaviorSeparataion::Update(DynParticles::Bodies* bodies, float weight, const UpdateState& state)
{
  V3* pos = bodies->pos;
  V3* acc = bodies->acc;
  V3* vel = bodies->vel;
  V3* force = bodies->force;
  DynParticles::DistMatrix* dm = bodies->distMatrix;
  int numBodies = bodies->numBodies;

  for (int i = 0; i < numBodies; ++i)
  {
    // return a force away from any close boids
    V3 avg(0, 0, 0);

    int numRejected = 0;
    for (int j = 0; j < numBodies; ++j)
    {
      float dist = dm[i*numBodies + j].dist;

      if (dist > separationDistance || i == j)
      {
        ++numRejected;
        continue;
      }

      // Note, what's really happening is that we're first normalizing the repelling
      // vector, then scaling by 1/r
      float invDist = dm[i*numBodies+j].invDist;
      avg += (pos[i] - pos[j]) * (invDist * invDist);
    }

    if (numRejected == numBodies)
      continue;

    avg /= (float)(numBodies - numRejected);

    // Reynolds uses: steering = desired - current (current + steering = desired)
    V3 desired = Normalize(avg) * maxSpeed;
    force[i] += weight * ClampVector(desired - vel[i], maxForce);
  }
}

//------------------------------------------------------------------------------
void BehaviorCohesion::Update(DynParticles::Bodies* bodies, float weight, const UpdateState& state)
{
  V3* pos = bodies->pos;
  V3* acc = bodies->acc;
  V3* vel = bodies->vel;
  V3* force = bodies->force;
  DynParticles::DistMatrix* dm = bodies->distMatrix;
  int numBodies = bodies->numBodies;

  for (int i = 0; i < numBodies; ++i)
  {
    // Return a force towards the average boid position
    V3 avg(0, 0, 0);

    int numRejected = 0;
    for (int j = 0; j < numBodies; ++j)
    {
      float dist = dm[i*numBodies + j].dist;

      if (dist > cohesionDistance || i == j)
      {
        ++numRejected;
        continue;
      }

      avg += pos[j];
    }

    if (numBodies == numRejected)
      continue;

    avg /= (float)(numBodies - numRejected);

    V3 desiredVel = Normalize(avg - pos[i]) * maxSpeed;
    force[i] += weight * ClampVector(desiredVel - vel[i], maxForce);
  }
}

//------------------------------------------------------------------------------
void BehaviorAlignment::Update(DynParticles::Bodies* bodies, float weight, const UpdateState& state)
{
  V3* pos = bodies->pos;
  V3* acc = bodies->acc;
  V3* vel = bodies->vel;
  V3* force = bodies->force;
  DynParticles::DistMatrix* dm = bodies->distMatrix;
  int numBodies = bodies->numBodies;

  for (int i = 0; i < numBodies; ++i)
  {
    // return a force to align the boids velocity with the average velocity
    V3 avg(0, 0, 0);

    int numRejected = 0;
    for (int j = 0; j < numBodies; ++j)
    {

      float dist = dm[i*numBodies + j].dist;
      if (dist > cohesionDistance || i == j)
      {
        numRejected++;
        continue;
      }

      avg += vel[j];
    }

    if (numRejected == numBodies)
      continue;

    avg /= (float)(numBodies - numRejected);
    V3 desired = Normalize(avg) * maxSpeed;
    force[i] += weight * ClampVector(desired - vel[i], maxForce);
  }
}
