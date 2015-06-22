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
  _bodies.distMatrix = new DistMatrix[numBodies];
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

  V3 center(0,0,0);
  for (int i = 0; i < numBodies; ++i)
  {
    force[i] = { 0, 0, 0 };
    center += pos[i];
  }

  _center = 1.0f / numBodies * center;

  // precompute particle distance calculations
  for (int i = 0; i < numBodies; ++i)
  {
    for (int j = i; j < numBodies; ++j)
    {
      if (i != j)
      {
        float tmp = Distance(pos[i], pos[j]);
        distMatrix[i*numBodies + j].dist = tmp;
        distMatrix[j*numBodies + i].dist = tmp;
      }
      else
      {
        distMatrix[i*numBodies + j].dist = 0;
      }
    }
  }


  for (Kinematic& k : _kinematics)
  {
    k.kinematic->Update(&_bodies, k.weight, updateState);
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
  V3* acc = bodies->acc;
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

    float cnt = 0.f;
    for (int j = 0; j < numBodies; ++j)
    {
      if (i == j)
        continue;

      float dist = dm[i*numBodies+j].dist;
      if (dist > separationDistance)
        continue;

      float invDist = 1.0f / dist;
      avg += (pos[i] - pos[j]) * (invDist * invDist);
      cnt += 1.f;
    }

    if (cnt == 0.f)
      continue;

    avg /= cnt;

    // Reynolds uses: steering = desired - current
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

    float cnt = 0.f;

    for (int j = 0; j < numBodies; ++j)
    {
      if (i == j)
        continue;

      float dist = dm[i*numBodies + j].dist;
      if (dist > cohesionDistance)
        continue;

      //DynParticles::Body* inner = &bodies[j];

//      avg += inner->pos;
      avg += pos[j];
      cnt += 1.f;
    }

    if (cnt == 0.f)
      continue;

    avg /= cnt;

//    V3 desiredVel = Normalize(avg - b->pos) * maxSpeed;
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
    //DynParticles::Body* b = &bodies[i];

    // return a force to align the boids velocity with the average velocity
    V3 avg(0, 0, 0);

    float cnt = 0.f;
    for (int j = 0; j < numBodies; ++j)
    {
      if (i == j)
        continue;

      float dist = dm[i*numBodies + j].dist;
      if (dist > cohesionDistance)
        continue;

      //DynParticles::Body* inner = &bodies[j];

      avg += vel[j];
      cnt += 1.f;
    }

    if (cnt == 0.f)
      continue;

    avg /= cnt;
    V3 desired = Normalize(avg) * maxSpeed;
    force[i] += weight * ClampVector(desired - vel[i], maxForce);
  }
}
