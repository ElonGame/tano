#include "dyn_particles.hpp"
#include "update_state.hpp"
#include "tano_math.hpp"

using namespace tano;
using namespace bristol;

//------------------------------------------------------------------------------
DynParticles::~DynParticles()
{
  SAFE_ADELETE(_bodies);
  SeqDelete(&_kinematics);
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

  for (int i = 0; i < _numBodies; ++i)
  {
    _bodies[i].acc = { 0, 0, 0 };
    _bodies[i].force = { 0, 0, 0 };
  }

  for (ParticleKinematics* p : _kinematics)
  {
    p->Update(_bodies, _numBodies, updateState);
  }
}

//------------------------------------------------------------------------------
void DynParticles::AddKinematics(ParticleKinematics* kinematics)
{
  _kinematics.push_back(kinematics);
}

//------------------------------------------------------------------------------
struct BehaviorSeek : public ParticleKinematics
{
  virtual void Update(DynParticles::Body* bodies, int numBodies, const UpdateState& state)
  {
    for (int i = 0; i < numBodies; ++i)
    {
      DynParticles::Body* b = &bodies[i];

      Vector3 tmp = target - b->pos;
      tmp.Normalize();
      Vector3 desiredVel = tmp * maxSpeed;
      b->force += ClampVector(desiredVel - b->vel, maxForce);
    }
  }

  Vector3 target = { 0, 0, 0 };
};

//------------------------------------------------------------------------------
struct BehaviorSeparataion : public ParticleKinematics
{
  virtual void Update(DynParticles::Body* bodies, int numBodies, const UpdateState& state)
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

        Vector3 f = b->pos - inner->pos;
        f.Normalize();
        avg += 1.f / dist * f;
        cnt += 1.f;
      }

      if (cnt == 0.f)
        continue;

      avg /= cnt;

      // Reynolds uses: steering = desired - current
      avg.Normalize();
      Vector3 desired = avg * maxSpeed;
      b->force += ClampVector(desired - b->vel, maxForce);
    }
  }

  float separationDistance = 10;
};

struct BehaviorCohesion : public ParticleKinematics
{
  virtual void Update(DynParticles::Body* bodies, int numBodies, const UpdateState& state)
  {
    for (int i = 0; i < numBodies; ++i)
    {
      DynParticles::Body* b = &bodies[i];
    }
  }
};

struct BehaviorAlignment : public ParticleKinematics
{
  virtual void Update(DynParticles::Body* bodies, int numBodies, const UpdateState& state)
  {
    for (int i = 0; i < numBodies; ++i)
    {
      DynParticles::Body* b = &bodies[i];
    }
  }
};

#if 0
//------------------------------------------------------------------------------
Vector3 Landscape::BoidCohesion(const Boid& boid)
{
  // Return a force towards the average boid position
  Vector3 avg(0, 0, 0);

  float cnt = 0.f;
  for (const Boid& b : boid.flock->boids)
  {
    if (b.id == boid.id)
      continue;

    float dist = Vector3::Distance(boid.pos, b.pos);
    if (dist > _settings.boids.cohesion_distance)
      continue;

    avg += b.pos;
    cnt += 1.f;
  }

  if (cnt == 0.f)
    return avg;

  avg /= cnt;
  return Seek(boid, avg);
}

//------------------------------------------------------------------------------
Vector3 Landscape::BoidAlignment(const Boid& boid)
{
  // return a force to align the boids velocity with the average velocity
  Vector3 avg(0, 0, 0);

  float cnt = 0.f;
  for (const Boid& b : boid.flock->boids)
  {
    if (b.id == boid.id)
      continue;

    float dist = Vector3::Distance(boid.pos, b.pos);
    if (dist > _settings.boids.cohesion_distance)
      continue;

    avg += b.vel;
    cnt += 1.f;
  }

  if (cnt == 0.f)
    return avg;

  avg /= cnt;
  avg.Normalize();
  Vector3 desired = avg * _settings.boids.max_speed;
  Vector3 steering = desired - boid.vel;
  return steering;
}
#endif