/*
  Update time

  Baseline (mothership): 39 ms
  First pass, converted to XMVECTOR: 35 ms
*/
#include "dyn_particles.hpp"
#include "update_state.hpp"
#include "tano_math.hpp"
#include "arena_allocator.hpp"

using namespace tano;
using namespace bristol;
using namespace DirectX;

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
  _bodies.pos = (XMVECTOR*)g_GlobalMemory.Alloc(numBodies * sizeof(XMVECTOR));
  _bodies.vel = (XMVECTOR*)g_GlobalMemory.Alloc(numBodies * sizeof(XMVECTOR));
  _bodies.acc = (XMVECTOR*)g_GlobalMemory.Alloc(numBodies * sizeof(XMVECTOR));
  _bodies.force = (XMVECTOR*)g_GlobalMemory.Alloc(numBodies * sizeof(XMVECTOR));
  _bodies.distMatrix = new DistMatrix[numBodies*numBodies];

  V3 zero(0,0,0);
  for (int i = 0; i < numBodies; ++i)
  {

    _bodies.pos[i] = _bodies.vel[i] = _bodies.acc[i] = _bodies.force[i] = XMVectorZero();
  }
}

//------------------------------------------------------------------------------
void DynParticles::Reset()
{
  SAFE_ADELETE(_bodies.distMatrix);
}

//------------------------------------------------------------------------------
void DynParticles::Update(const UpdateState& updateState)
{
  if (!_bodies.numBodies)
    return;

  int numBodies = _bodies.numBodies;
  XMVECTOR* pos = _bodies.pos;
  XMVECTOR* acc = _bodies.acc;
  XMVECTOR* vel = _bodies.vel;
  XMVECTOR* force = _bodies.force;
  DynParticles::DistMatrix* distMatrix = _bodies.distMatrix;

  _tickCount++;
  int step = _tickCount % 4;
  bool updateForces = step == 0 || step == 2;
  bool updateDistances = step == 1;

  updateForces = true;
  updateDistances = true;

  XMVECTOR center = XMVectorZero();
  //V3 center(0,0,0);
  for (int i = 0; i < numBodies; ++i)
  {
    if (updateForces)
    {
      force[i] = { 0, 0, 0 };
    }
    center += pos[i];
  }

  _center = XMVectorScale(center, 1.0f / numBodies);
//  _center = 1.0f / numBodies * center;

  if (updateDistances)
  {
    // precompute particle distance calculations
    for (int i = 0; i < numBodies; ++i)
    {
      for (int j = i; j < numBodies; ++j)
      {
        if (i != j)
        {
          XMVECTOR tmp = XMVector3LengthEst(XMVectorSubtract(pos[i], pos[j]));
          float dist;
          XMStoreFloat(&dist, tmp);
          //float dist = Distance(pos[i], pos[j]);
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
    _bodies.vel[i] = XMVector3ClampLength(
        XMVectorAdd(_bodies.vel[i], XMVectorScale(_bodies.acc[i], dt)), 
        0, 
        _maxSpeed);
    //_bodies.vel[i] = ClampVector(XMVectorAdd(_bodies.vel[i], XMVectorScale(_bodies.acc[i], dt)), _maxSpeed);
    _bodies.pos[i] = XMVectorAdd(_bodies.pos[i], XMVectorScale(_bodies.vel[i], dt));
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
  XMVECTOR* pos = bodies->pos;
  XMVECTOR* vel = bodies->vel;
  XMVECTOR* force = bodies->force;
  int numBodies = bodies->numBodies;

  for (int i = 0; i < numBodies; ++i)
  {
    XMVECTOR desiredVel = XMVectorScale(XMVector3Normalize(XMVectorSubtract(target, pos[i])), maxSpeed);
    force[i] = XMVectorAdd(
      force[i], 
      XMVectorScale(
        XMVector3ClampLength(XMVectorSubtract(desiredVel, vel[i]), 0, maxForce), 
        weight));
  }
}

//------------------------------------------------------------------------------
void BehaviorSeparataion::Update(DynParticles::Bodies* bodies, float weight, const UpdateState& state)
{
  XMVECTOR* pos = bodies->pos;
  XMVECTOR* acc = bodies->acc;
  XMVECTOR* vel = bodies->vel;
  XMVECTOR* force = bodies->force;
  DynParticles::DistMatrix* dm = bodies->distMatrix;
  int numBodies = bodies->numBodies;

  for (int i = 0; i < numBodies; ++i)
  {
    // return a force away from any close boids
    XMVECTOR avg = XMVectorZero();

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
      avg = XMVectorAdd(avg, XMVectorScale(XMVectorSubtract(pos[i], pos[j]), invDist * invDist));
    }

    if (numRejected == numBodies)
      continue;

    avg = XMVectorDivide(avg, XMVectorReplicate((float)(numBodies - numRejected)));

    // Reynolds uses: steering = desired - current (current + steering = desired)
    XMVECTOR desired = XMVectorScale(XMVector3Normalize(avg), maxSpeed);
    force[i] = XMVectorAdd(
      force[i],
      XMVectorScale(
        XMVector3ClampLength(XMVectorSubtract(desired, vel[i]), 0, maxForce),
        weight));

  }
}

//------------------------------------------------------------------------------
void BehaviorCohesion::Update(DynParticles::Bodies* bodies, float weight, const UpdateState& state)
{
  XMVECTOR* pos = bodies->pos;
  XMVECTOR* acc = bodies->acc;
  XMVECTOR* vel = bodies->vel;
  XMVECTOR* force = bodies->force;
  DynParticles::DistMatrix* dm = bodies->distMatrix;
  int numBodies = bodies->numBodies;

  for (int i = 0; i < numBodies; ++i)
  {
    // Return a force towards the average boid position
    XMVECTOR avg = XMVectorZero();

    int numRejected = 0;
    for (int j = 0; j < numBodies; ++j)
    {
      float dist = dm[i*numBodies + j].dist;

      if (dist > cohesionDistance || i == j)
      {
        ++numRejected;
        continue;
      }
      avg = XMVectorAdd(avg, pos[j]);
    }

    if (numBodies == numRejected)
      continue;

    avg = XMVectorDivide(avg, XMVectorReplicate((float)(numBodies - numRejected)));

    XMVECTOR desiredVel = XMVectorScale(XMVector3Normalize(XMVectorSubtract(avg, pos[i])), maxSpeed);
    force[i] = XMVectorAdd(
      force[i], 
      XMVectorScale(XMVector3ClampLength(XMVectorSubtract(desiredVel, vel[i]), 0, maxForce), weight));
  }
}

//------------------------------------------------------------------------------
void BehaviorAlignment::Update(DynParticles::Bodies* bodies, float weight, const UpdateState& state)
{
  XMVECTOR* pos = bodies->pos;
  XMVECTOR* acc = bodies->acc;
  XMVECTOR* vel = bodies->vel;
  XMVECTOR* force = bodies->force;
  DynParticles::DistMatrix* dm = bodies->distMatrix;
  int numBodies = bodies->numBodies;

  for (int i = 0; i < numBodies; ++i)
  {
    // return a force to align the boids velocity with the average velocity
    XMVECTOR avg = XMVectorZero();

    int numRejected = 0;
    for (int j = 0; j < numBodies; ++j)
    {

      float dist = dm[i*numBodies + j].dist;
      if (dist > cohesionDistance || i == j)
      {
        numRejected++;
        continue;
      }
      avg = XMVectorAdd(avg, vel[j]);
    }

    if (numRejected == numBodies)
      continue;

    avg = XMVectorDivide(avg, XMVectorReplicate((float)(numBodies - numRejected)));
    XMVECTOR desired = XMVectorScale(XMVector3Normalize(avg), maxSpeed);
    force[i] = XMVectorAdd(
      force[i],
      XMVectorScale(XMVector3ClampLength(XMVectorSubtract(desired, vel[i]), 0, maxForce), weight));
  }
}
