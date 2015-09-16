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
  _bodies.pos = new V3[numBodies];
  _bodies.vel = new V3[numBodies];
  _bodies.acc = new V3[numBodies];
  _bodies.force = new V3[numBodies];

  V3 zero(0,0,0);
  for (int i = 0; i < numBodies; ++i)
  {
    _bodies.pos[i] = _bodies.vel[i] = _bodies.acc[i] = _bodies.force[i] = V3::Zero;
  }
}

//------------------------------------------------------------------------------
void DynParticles::Reset()
{
  SAFE_ADELETE(_bodies.pos);
  SAFE_ADELETE(_bodies.vel);
  SAFE_ADELETE(_bodies.acc);
  SAFE_ADELETE(_bodies.force);
}

//------------------------------------------------------------------------------
void DynParticles::SetDistCutOff(DistMeasureType type, float cutoff)
{
  //_bodies.distMeasures[type].cutoff = cutoff;
}

//------------------------------------------------------------------------------
void DynParticles::Update(const FixedUpdateState& updateState, bool alwaysUpdate)
{
  if (!_bodies.numBodies)
    return;

  int numBodies = _bodies.numBodies;
  V3* pos = _bodies.pos;
  V3* acc = _bodies.acc;
  V3* vel = _bodies.vel;
  V3* force = _bodies.force;

  V3 center = V3::Zero;
  for (int i = 0; i < numBodies; ++i)
  {
    center += pos[i];
  }

  _center /= (float)numBodies;//  =  XMVectorScale(center, 1.0f / numBodies);

  // update distances over N frames
  {
    int DISTANCE_FRAMES = 100;
    float f = (_tickCount % DISTANCE_FRAMES) / (float)DISTANCE_FRAMES;
    int bodiesPerFrame = max(1, numBodies / DISTANCE_FRAMES);
    int start = (int)(f * numBodies);
    int end = min(numBodies, start + bodiesPerFrame);
    UpdateDistMatrix(start, end);
  }

  {
    int UPDATE_FRAMES = 1;
    float f = (_tickCount % UPDATE_FRAMES) / (float)UPDATE_FRAMES;
    int bodiesPerFrame = max(1, numBodies / UPDATE_FRAMES);
    int start = (int)(f * numBodies);
    int end = min(numBodies, start + bodiesPerFrame);

    for (int i = start; i < end; ++i)
    {
      force[i] = { 0, 0, 0 };
    }

    for (Kinematic& k : _kinematics)
    {
      k.kinematic->Update(&_bodies, start, end, k.weight, updateState);
    }
  }

  float dt = updateState.delta;
  for (int i = 0; i < numBodies; ++i)
  {
    _bodies.acc[i] = _bodies.force[i];
    _bodies.vel[i] = ClampVector(_bodies.vel[i] + dt * _bodies.acc[i], _maxSpeed);
    //  XMVector3ClampLengthMax(
    //    XMVectorAdd(_bodies.vel[i], XMVectorScale(_bodies.acc[i], dt)), 
    //    _maxSpeed);
    //_bodies.pos[i] = XMVectorAdd(_bodies.pos[i], XMVectorScale(_bodies.vel[i], dt));
    _bodies.pos[i] += dt * _bodies.vel[i];
  }

  _tickCount++;

}

//------------------------------------------------------------------------------
void DynParticles::UpdateDistMatrix(int start, int end)
{
  return;
  //int numBodies = _bodies.numBodies;
  //XMVECTOR* pos = _bodies.pos;

  //for (int i = 0; i < DistCount; ++i)
  //{
  //  for (int j = start; j < end; ++j)
  //  {
  //    _bodies.distMeasures[i].values[j*numBodies].idx = -1;
  //  }
  //}

  //for (int k = 0; k < DistCount; ++k)
  //{
  //  // precompute particle distance calculations
  //  for (int i = start; i < end; ++i)
  //  {
  //    DistMeasureEntry* e = &_bodies.distMeasures[k].values[i*numBodies];
  //    float cutOff = _bodies.distMeasures[k].cutoff;
  //    cutOff *= cutOff;
  //    for (int j = 0; j < numBodies; ++j)
  //    {
  //      if (i != j)
  //      {
  //        XMVECTOR tmp = XMVector3LengthSq(XMVectorSubtract(pos[i], pos[j]));
  //        float dist;
  //        XMStoreFloat(&dist, tmp);
  //        if (dist < cutOff)
  //        {
  //          dist = sqrtf(dist);
  //          float invDist = 1.0f / dist;
  //          e->idx = j;
  //          e->dist = dist;
  //          e->invDist = invDist;
  //          e++;
  //        }
  //      }
  //    }
  //    // use -1 as a sentinal
  //    e->idx = -1;
  //  }
  //}

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
  DynParticles::Bodies* bodies, int start, int end, float weight, const FixedUpdateState& state)
{
  V3* pos = bodies->pos;
  V3* vel = bodies->vel;
  V3* force = bodies->force;
  int numBodies = bodies->numBodies;

  for (int i = start; i < end; ++i)
  {
    V3 desiredVel = maxSpeed * Normalize(target - pos[i]);
    force[i] += weight * ClampVector(desiredVel - vel[i], maxForce);
    //XMVECTOR desiredVel = XMVectorScale(XMVector3Normalize(XMVectorSubtract(target, pos[i])), maxSpeed);
    //force[i] = XMVectorAdd(
    //  force[i], 
    //  XMVectorScale(
    //    XMVector3ClampLengthMax(XMVectorSubtract(desiredVel, vel[i]), maxForce), 
    //    weight));
  }
}

//------------------------------------------------------------------------------
void BehaviorSeparataion::Update(
  DynParticles::Bodies* bodies, int start, int end, float weight, const FixedUpdateState& state)
{
  return;
  V3* pos = bodies->pos;
  V3* acc = bodies->acc;
  V3* vel = bodies->vel;
  V3* force = bodies->force;
  int numBodies = bodies->numBodies;

  for (int i = start; i < end; ++i)
  {
    // return a force away from any close boids
    //V3 avg = XMVectorZero();
    V3 avg = V3::Zero;
    V3 curPos = pos[i];

    for (int j = 0; j < numBodies; ++j)
    {
      // vector away from j
      V3 away = Normalize(curPos - pos[j]);
      avg += away;
      //float dist = LengthSquared(away);
      //avg = XMVectorAdd(avg, XMVectorScale(XMVectorSubtract(pos[i], pos[j]), invDist * invDist));
      //avg += 1.0f / dist * delta;
      //avg = XMVectorAdd(avg, XMVectorScale(XMVectorSubtract(pos[i], pos[j]), invDist * invDist));
    }

    //DynParticles::DistMeasureEntry* values = &bodies->distMeasures[DynParticles::DistSeperation].values[i*numBodies];
    //DynParticles::DistMeasureEntry* start = values;
    //while (values->idx != -1)
    //{
    //  float invDist = values->invDist;
    //  int idx = values->idx;
    //  avg = XMVectorAdd(avg, XMVectorScale(XMVectorSubtract(pos[i], pos[idx]), invDist * invDist));
    //  values++;
    //}
    //int numCounted = (int)(values - start);

    //if (numCounted == 0)
    //  continue;

    //avg = XMVectorDivide(avg, XMVectorReplicate((float)numCounted));

    avg *= 1.0f / numBodies;

    // Reynolds uses: steering = desired - current (current + steering = desired)

    V3 desiredVel = maxSpeed * Normalize(avg);
    force[i] += weight * ClampVector(desiredVel - vel[i], maxForce);

    //V3 desired = XMVectorScale(XMVector3Normalize(avg), maxSpeed);
    //force[i] = XMVectorAdd(
    //  force[i],
    //  XMVectorScale(
    //    XMVector3ClampLengthMax(XMVectorSubtract(desired, vel[i]), maxForce),
    //    weight));

  }
}

//------------------------------------------------------------------------------
void BehaviorCohesion::Update(
  DynParticles::Bodies* bodies, int start, int end, float weight, const FixedUpdateState& state)
{
  return;

  V3* pos = bodies->pos;
  V3* acc = bodies->acc;
  V3* vel = bodies->vel;
  V3* force = bodies->force;
  int numBodies = bodies->numBodies;

  for (int i = start; i < end; ++i)
  {
    // Return a force towards the average boid position
    V3 avg = V3::Zero;
    V3 curPos = pos[i];
    for (int j = 0; j < numBodies; j++)
    {
      V3 towards = Normalize(pos[j] - curPos);
      avg += towards;
    }

    avg *= 1.0f / numBodies;

    //DynParticles::DistMeasureEntry* values = &bodies->distMeasures[DynParticles::DistCohesion].values[i*numBodies];
    //DynParticles::DistMeasureEntry* start = values;
    //while (values->idx != -1)
    //{
    //  float invDist = values->invDist;
    //  int idx = values->idx;
    //  avg = XMVectorAdd(avg, pos[idx]);
    //  values++;
    //}
    //int numCounted = (int)(values - start);

    //if (numCounted == 0)
    //  continue;

    //avg = XMVectorDivide(avg, XMVectorReplicate((float)numCounted));

    V3 desiredVel = maxSpeed * Normalize(avg);
    force[i] += weight * ClampVector(desiredVel - vel[i], maxForce);

    //XMVECTOR desiredVel = XMVectorScale(XMVector3Normalize(XMVectorSubtract(avg, pos[i])), maxSpeed);
    //force[i] = XMVectorAdd(
    //  force[i], 
    //  XMVectorScale(
    //    XMVector3ClampLengthMax(XMVectorSubtract(desiredVel, vel[i]), maxForce),
    //    weight));
  }
}

//------------------------------------------------------------------------------
void BehaviorAlignment::Update(
  DynParticles::Bodies* bodies, int start, int end, float weight, const FixedUpdateState& state)
{
  return;

  V3* pos = bodies->pos;
  V3* acc = bodies->acc;
  V3* vel = bodies->vel;
  V3* force = bodies->force;
  int numBodies = bodies->numBodies;

  for (int i = start; i < end; ++i)
  {
    // return a force to align the boids velocity with the average velocity
    V3 avg = V3::Zero;

    for (int j = 0; j < numBodies; ++j)
    {
      avg += vel[j];
    }

    avg *= 1.0f / numBodies;

    //DynParticles::DistMeasureEntry* values = &bodies->distMeasures[DynParticles::DistCohesion].values[i*numBodies];
    //DynParticles::DistMeasureEntry* start = values;
    //while (values->idx != -1)
    //{
    //  float invDist = values->invDist;
    //  int idx = values->idx;
    //  avg = XMVectorAdd(avg, vel[idx]);
    //  values++;
    //}
    //int numCounted = (int)(values - start);

    //if (numCounted == 0)
    //  continue;

    V3 desiredVel = maxSpeed * Normalize(avg);
    force[i] += weight * ClampVector(desiredVel - vel[i], maxForce);

    //avg = XMVectorDivide(avg, XMVectorReplicate((float)numCounted));
    //XMVECTOR desired = XMVectorScale(XMVector3Normalize(avg), maxSpeed);
    //force[i] = XMVectorAdd(
    //  force[i],
    //  XMVectorScale(
    //    XMVector3ClampLengthMax(XMVectorSubtract(desired, vel[i]), maxForce), 
    //    weight));
  }
}
