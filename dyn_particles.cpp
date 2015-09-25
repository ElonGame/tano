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

static const int NUM_BUCKETS = 16;
static const int TOTAL_NUM_BUCKETS = NUM_BUCKETS * NUM_BUCKETS;

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
  _bodies.pos = new vec3[numBodies];
  _bodies.vel = new vec3[numBodies];
  _bodies.acc = new vec3[numBodies];
  _bodies.force = new vec3[numBodies];

  _buckets = new Bucket[TOTAL_NUM_BUCKETS];
  for (int i = 0; i < TOTAL_NUM_BUCKETS; ++i)
    _buckets[i].data = new u16[numBodies];

  vec3 zero(0,0,0);
  for (int i = 0; i < numBodies; ++i)
  {
    _bodies.pos[i] = _bodies.vel[i] = _bodies.acc[i] = _bodies.force[i] = vec3::Zero;
  }
}

//------------------------------------------------------------------------------
void DynParticles::Reset()
{
  SAFE_ADELETE(_bodies.pos);
  SAFE_ADELETE(_bodies.vel);
  SAFE_ADELETE(_bodies.acc);
  SAFE_ADELETE(_bodies.force);

  if (_buckets)
  {
    for (int i = 0; i < TOTAL_NUM_BUCKETS; ++i)
      SAFE_ADELETE(_buckets[i].data);
    SAFE_ADELETE(_buckets);
  }
}

//------------------------------------------------------------------------------
void DynParticles::Update(float deltaTime, bool alwaysUpdate)
{
  if (!_bodies.numBodies)
    return;

  int numBodies = _bodies.numBodies;
  vec3* pos = _bodies.pos;
  vec3* acc = _bodies.acc;
  vec3* vel = _bodies.vel;
  vec3* force = _bodies.force;

  vec3 center = vec3::Zero;
  vec3 minPos = pos[0];
  vec3 maxPos = pos[0];
  for (int i = 0; i < numBodies; ++i)
  {
    center += pos[i];
    minPos = Min(minPos, pos[i]);
    maxPos = Max(maxPos, pos[i]);
  }

  _center = 1.f / numBodies * center;

  // sort the bodies into the correct bucket
  for (int i = 0; i < TOTAL_NUM_BUCKETS; ++i)
    _buckets[i].count = 0;

  float dx = maxPos.x - minPos.x;
  float dz = maxPos.z - minPos.z;

  _validBuckets.clear();
  for (int i = 0; i < numBodies; ++i)
  {
    vec3 p = pos[i];
    int xx = (int)((NUM_BUCKETS - 1) * (p.x - minPos.x) / dx);
    int zz = (int)((NUM_BUCKETS - 1) * (p.z - minPos.z) / dz);
    int idx = zz * NUM_BUCKETS + xx;

    if (_buckets[idx].count == 0)
    {
      _validBuckets.push_back(&_buckets[idx]);
    }
    _buckets[idx].data[_buckets[idx].count++] = i;
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
      ParticleKinematics::UpdateParams params{ &_bodies, start, end, k.weight, deltaTime, this };
      k.kinematic->Update(params);
    }
  }

  for (int i = 0; i < numBodies; ++i)
  {
    _bodies.acc[i] = _bodies.force[i];
    _bodies.vel[i] = ClampVector(_bodies.vel[i] + deltaTime * _bodies.acc[i], _maxSpeed);
    _bodies.pos[i] += deltaTime * _bodies.vel[i];
  }

  _tickCount++;

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
void BehaviorSeek::Update(const ParticleKinematics::UpdateParams& params)
{
  vec3* pos = params.bodies->pos;
  vec3* vel = params.bodies->vel;
  vec3* force = params.bodies->force;
  int numBodies = params.bodies->numBodies;

  for (int i = params.start; i < params.end; ++i)
  {
    vec3 desiredVel = maxSpeed * Normalize(target - pos[i]);
    force[i] += params.weight * ClampVector(desiredVel - vel[i], maxForce);
  }
}

//------------------------------------------------------------------------------
void BehaviorSeparataion::Update(const ParticleKinematics::UpdateParams& params)
{
  vec3* pos = params.bodies->pos;
  vec3* acc = params.bodies->acc;
  vec3* vel = params.bodies->vel;
  vec3* force = params.bodies->force;
  int numBodies = params.bodies->numBodies;

  for (DynParticles::Bucket* bucket : params.p->_validBuckets)
  {
    for (int iIdx = 0; iIdx < bucket->count; ++iIdx)
    {
      int i = bucket->data[iIdx];
      vec3 avg = vec3::Zero;
      vec3 curPos = pos[i];

      for (int jIdx = 0; jIdx < bucket->count; ++jIdx)
      {
        int j = bucket->data[jIdx];
        vec3 away = Normalize(curPos - pos[j]);
        avg += away;
      }
      avg *= 1.0f / numBodies;

      // Reynolds uses: steering = desired - current (current + steering = desired)
      vec3 desiredVel = maxSpeed * Normalize(avg);
      force[i] += params.weight * ClampVector(desiredVel - vel[i], maxForce);
    }
  }
}

//------------------------------------------------------------------------------
void BehaviorCohesion::Update(const ParticleKinematics::UpdateParams& params)
{
  vec3* pos = params.bodies->pos;
  vec3* acc = params.bodies->acc;
  vec3* vel = params.bodies->vel;
  vec3* force = params.bodies->force;
  int numBodies = params.bodies->numBodies;

  for (DynParticles::Bucket* bucket : params.p->_validBuckets)
  {
    for (int iIdx = 0; iIdx < bucket->count; ++iIdx)
    {
      int i = bucket->data[iIdx];

      // Return a force towards the average boid position
      vec3 avg = vec3::Zero;
      vec3 curPos = pos[i];

      for (int jIdx = 0; jIdx < bucket->count; ++jIdx)
      {
        int j = bucket->data[jIdx];
        vec3 towards = Normalize(pos[j] - curPos);
        avg += towards;
      }
      avg *= 1.0f / numBodies;

      // Reynolds uses: steering = desired - current (current + steering = desired)
      vec3 desiredVel = maxSpeed * Normalize(avg);
      force[i] += params.weight * ClampVector(desiredVel - vel[i], maxForce);
    }
  }
}

//------------------------------------------------------------------------------
void BehaviorAlignment::Update(const ParticleKinematics::UpdateParams& params)
{
  vec3* pos = params.bodies->pos;
  vec3* acc = params.bodies->acc;
  vec3* vel = params.bodies->vel;
  vec3* force = params.bodies->force;
  int numBodies = params.bodies->numBodies;

  for (DynParticles::Bucket* bucket : params.p->_validBuckets)
  {
    for (int iIdx = 0; iIdx < bucket->count; ++iIdx)
    {
      int i = bucket->data[iIdx];

      // return a force to align the boids velocity with the average velocity
      vec3 avg = vec3::Zero;

      for (int jIdx = 0; jIdx < bucket->count; ++jIdx)
      {
        int j = bucket->data[jIdx];
        avg += vel[j];
      }
      avg *= 1.0f / numBodies;

      // Reynolds uses: steering = desired - current (current + steering = desired)
      vec3 desiredVel = maxSpeed * Normalize(avg);
      force[i] += params.weight * ClampVector(desiredVel - vel[i], maxForce);
    }
  }
}
