/*
  Update time

  Baseline (mothership): 39 ms
  First pass, converted to XMVECTOR: 35 ms
  Use smarter distmatrix thing: 23 ms
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
  _bodies.pos = g_GlobalMemory.Alloc<XMVECTOR>(numBodies);
  _bodies.vel = g_GlobalMemory.Alloc<XMVECTOR>(numBodies);
  _bodies.acc = g_GlobalMemory.Alloc<XMVECTOR>(numBodies);
  _bodies.force = g_GlobalMemory.Alloc<XMVECTOR>(numBodies);

  V3 zero(0,0,0);
  for (int i = 0; i < numBodies; ++i)
  {
    _bodies.pos[i] = _bodies.vel[i] = _bodies.acc[i] = _bodies.force[i] = XMVectorZero();
  }

  for (int i = 0; i < DistCount; ++i)
  {
    _bodies.distMeasures[i].values = new DistMeasureEntry[numBodies*numBodies];
  }
}

//------------------------------------------------------------------------------
void DynParticles::Reset()
{
  for (int i = 0; i < DistCount; ++i)
  {
    SAFE_ADELETE(_bodies.distMeasures[i].values);
  }
}

//------------------------------------------------------------------------------
void DynParticles::SetDistCutOff(DistMeasureType type, float cutoff)
{
  _bodies.distMeasures[type].cutoff = cutoff;
}

//------------------------------------------------------------------------------
void DynParticles::Update(const UpdateState& updateState, bool alwaysUpdate)
{
  if (!_bodies.numBodies)
    return;

  int numBodies = _bodies.numBodies;
  XMVECTOR* pos = _bodies.pos;
  XMVECTOR* acc = _bodies.acc;
  XMVECTOR* vel = _bodies.vel;
  XMVECTOR* force = _bodies.force;

  _tickCount++;
  int step = _tickCount % 8;
  bool updateForces = alwaysUpdate || step == 0 || step == 2 || step == 4 || step == 6;
  bool updateDistances = alwaysUpdate || step == 1;

  XMVECTOR center = XMVectorZero();
  for (int i = 0; i < numBodies; ++i)
  {
    if (updateForces)
    {
      force[i] = { 0, 0, 0 };
    }
    center += pos[i];
  }

  _center = XMVectorScale(center, 1.0f / numBodies);

  if (updateDistances)
  {
    UpdateDistMatrix();
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
    _bodies.vel[i] = XMVector3ClampLengthMax(
        XMVectorAdd(_bodies.vel[i], XMVectorScale(_bodies.acc[i], dt)), 
        _maxSpeed);
    _bodies.pos[i] = XMVectorAdd(_bodies.pos[i], XMVectorScale(_bodies.vel[i], dt));
  }
}

//------------------------------------------------------------------------------
void DynParticles::UpdateDistMatrix()
{
  int numBodies = _bodies.numBodies;
  XMVECTOR* pos = _bodies.pos;

  for (int i = 0; i < DistCount; ++i)
  {
    for (int j = 0; j < numBodies; ++j)
    {
      _bodies.distMeasures[i].values[j*numBodies].idx = -1;
    }
  }

  for (int k = 0; k < DistCount; ++k)
  {
    // precompute particle distance calculations
    for (int i = 0; i < numBodies; ++i)
    {
      DistMeasureEntry* e = &_bodies.distMeasures[k].values[i*numBodies];
      float cutOff = _bodies.distMeasures[k].cutoff;
      for (int j = 0; j < numBodies; ++j)
      {
        if (i != j)
        {
          XMVECTOR tmp = XMVector3LengthEst(XMVectorSubtract(pos[i], pos[j]));
          float dist;
          XMStoreFloat(&dist, tmp);
          if (dist < cutOff)
          {
            float invDist = 1.0f / dist;
            e->idx = j;
            e->dist = dist;
            e->invDist = invDist;
            e++;
          }
        }
      }
      // use -1 as a sentinal
      e->idx = -1;
    }
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
        XMVector3ClampLengthMax(XMVectorSubtract(desiredVel, vel[i]), maxForce), 
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
  int numBodies = bodies->numBodies;

  for (int i = 0; i < numBodies; ++i)
  {
    // return a force away from any close boids
    XMVECTOR avg = XMVectorZero();

    DynParticles::DistMeasureEntry* values = &bodies->distMeasures[DynParticles::DistSeperation].values[i*numBodies];
    DynParticles::DistMeasureEntry* start = values;
    while (values->idx != -1)
    {
      float invDist = values->invDist;
      int idx = values->idx;
      avg = XMVectorAdd(avg, XMVectorScale(XMVectorSubtract(pos[i], pos[idx]), invDist * invDist));
      values++;
    }
    int numCounted = (int)(values - start);

    if (numCounted == 0)
      continue;

    avg = XMVectorDivide(avg, XMVectorReplicate((float)numCounted));

    // Reynolds uses: steering = desired - current (current + steering = desired)
    XMVECTOR desired = XMVectorScale(XMVector3Normalize(avg), maxSpeed);
    force[i] = XMVectorAdd(
      force[i],
      XMVectorScale(
        XMVector3ClampLengthMax(XMVectorSubtract(desired, vel[i]), maxForce),
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
  int numBodies = bodies->numBodies;

  for (int i = 0; i < numBodies; ++i)
  {
    // Return a force towards the average boid position
    XMVECTOR avg = XMVectorZero();

    DynParticles::DistMeasureEntry* values = &bodies->distMeasures[DynParticles::DistCohesion].values[i*numBodies];
    DynParticles::DistMeasureEntry* start = values;
    while (values->idx != -1)
    {
      float invDist = values->invDist;
      int idx = values->idx;
      avg = XMVectorAdd(avg, pos[idx]);
      values++;
    }
    int numCounted = (int)(values - start);

    if (numCounted == 0)
      continue;

    avg = XMVectorDivide(avg, XMVectorReplicate((float)numCounted));

    XMVECTOR desiredVel = XMVectorScale(XMVector3Normalize(XMVectorSubtract(avg, pos[i])), maxSpeed);
    force[i] = XMVectorAdd(
      force[i], 
      XMVectorScale(XMVector3ClampLengthMax(XMVectorSubtract(desiredVel, vel[i]), maxForce), weight));
  }
}

//------------------------------------------------------------------------------
void BehaviorAlignment::Update(DynParticles::Bodies* bodies, float weight, const UpdateState& state)
{
  XMVECTOR* pos = bodies->pos;
  XMVECTOR* acc = bodies->acc;
  XMVECTOR* vel = bodies->vel;
  XMVECTOR* force = bodies->force;
  int numBodies = bodies->numBodies;

  for (int i = 0; i < numBodies; ++i)
  {
    // return a force to align the boids velocity with the average velocity
    XMVECTOR avg = XMVectorZero();

    DynParticles::DistMeasureEntry* values = &bodies->distMeasures[DynParticles::DistCohesion].values[i*numBodies];
    DynParticles::DistMeasureEntry* start = values;
    while (values->idx != -1)
    {
      float invDist = values->invDist;
      int idx = values->idx;
      avg = XMVectorAdd(avg, vel[idx]);
      values++;
    }
    int numCounted = (int)(values - start);

    if (numCounted == 0)
      continue;

    avg = XMVectorDivide(avg, XMVectorReplicate((float)numCounted));
    XMVECTOR desired = XMVectorScale(XMVector3Normalize(avg), maxSpeed);
    force[i] = XMVectorAdd(
      force[i],
      XMVectorScale(XMVector3ClampLengthMax(XMVectorSubtract(desired, vel[i]), maxForce), weight));
  }
}
