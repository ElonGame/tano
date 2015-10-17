#include "particle_emitters.hpp"
#include "scheduler.hpp"
#include "blackboard.hpp"

using namespace tano;
using namespace tano::scheduler;
using namespace bristol;
using namespace DirectX;

//------------------------------------------------------------------------------
template <typename T>
void UpdateEmitter(const scheduler::TaskData& data)
{
  typedef typename T::EmitterKernelData EmitterKernelData;
  EmitterKernelData* emitterData = (EmitterKernelData*)data.kernelData.data;
  T* emitter = emitterData->emitter;
  emitter->Update(emitterData->dt);
}

//------------------------------------------------------------------------------
template <typename T>
void CopyOutEmitter(const scheduler::TaskData& data)
{
  typedef typename T::EmitterKernelData EmitterKernelData;
  EmitterKernelData* emitterData = (EmitterKernelData*)data.kernelData.data;
  T* emitter = emitterData->emitter;
  emitter->CopyToBuffer(emitterData->vtx);
}

//------------------------------------------------------------------------------
void ParticleEmitter::Create(const vec3& center, int numParticles)
{
  _center = center;
  _numParticles = numParticles;
  _pos = new XMVECTOR[_numParticles];
  _vel = new XMVECTOR[_numParticles];
  _deadParticles = new int[_numParticles];

  float s = 100;
  for (int i = 0; i < _numParticles; ++i)
  {
    CreateParticle(i, s);
  }

  for (int i = 0; i < 1000; ++i)
  {
    Update(0.01f);
  }
}

//------------------------------------------------------------------------------
void ParticleEmitter::CreateParticle(int idx, float s)
{
  // lifetime and lifetime decay is stored in the w-component
  XMFLOAT4 p(_center.x + randf(-s, s), _center.y + randf(-s, s), _center.z + randf(1500.f, 2000.f), 0.f);

  XMFLOAT4 v(randf(-s, s), randf(-s, s), -randf(10.f, 200.f), 0);

  _pos[idx] = XMLoadFloat4(&p);
  _vel[idx] = XMLoadFloat4(&v);
}

//------------------------------------------------------------------------------
void ParticleEmitter::Destroy()
{
  SAFE_ADELETE(_pos);
  SAFE_ADELETE(_vel);
  SAFE_ADELETE(_deadParticles);
}

//------------------------------------------------------------------------------
void ParticleEmitter::Update(float dt)
{
  int numDead = 0;
  int* dead = _deadParticles;

  XMVECTOR zClip = XMVectorReplicate(-1500);

  for (int i = 0, e = _numParticles; i < e; ++i)
  {
    XMVECTOR scaledVel = XMVectorScale(_vel[i], dt);
    _pos[i] = XMVectorAdd(_pos[i], scaledVel);

    XMVECTOR res = XMVectorLess(_pos[i], zClip);
    u32 rr[4];
    // rr[2] is 0xffffffff if pos[i] < zClip
    XMStoreInt4(rr, res);
    if (rr[2])
    {
      *dead++ = i;
      numDead++;
    }
  }

  float s = 100;
  for (int i = 0; i < numDead; ++i)
  {
    CreateParticle(_deadParticles[i], s);
  }
}

//------------------------------------------------------------------------------
void ParticleEmitter::CopyToBuffer(vec4* vtx)
{
  memcpy(vtx, _pos, _numParticles * sizeof(vec4));
}

//------------------------------------------------------------------------------
void ParticleEmitter::UpdateEmitter(const TaskData& data)
{
  ::UpdateEmitter<ParticleEmitter>(data);
}

//------------------------------------------------------------------------------
void ParticleEmitter::CopyOutEmitter(const TaskData& data)
{
  ::CopyOutEmitter<ParticleEmitter>(data);
}

//------------------------------------------------------------------------------
void RadialParticleEmitter::Create(const vec3& center, float radius, int numParticles)
{
  _center = center;
  _radius = radius;
  _numParticles = numParticles;
  _pos = new XMVECTOR[_numParticles];
  _angleVel = new float[_numParticles];
  _angle = new float[_numParticles];

  _deadParticles = new int[_numParticles];
  _spawnedParticles = 0;

  float s = 100;
  for (int i = 0; i < _numParticles; ++i)
  {
    CreateParticle(i, s);
  }

  for (int i = 0; i < 1000; ++i)
  {
    Update(0.05f);
  }
}

//------------------------------------------------------------------------------
void RadialParticleEmitter::CreateParticle(int idx, float s)
{
  // lifetime and lifetime decay is stored in the w-component
  XMFLOAT4 p(_center.x + randf(-s, s), _center.y + randf(-s, s), _center.z + randf(1500.f, 2000.f), 0.f);

  XMFLOAT4 v(randf(-s, s), randf(-s, s), -randf(10.f, 200.f), 0);

  float newAngle = randf(0.1f, 1.0f);
  XMFLOAT4 newPos(_radius * sinf(newAngle), 0, _radius * cosf(newAngle), 1);

  _pos[idx] = XMLoadFloat4(&newPos);
  _angle[idx] = newAngle;
  _angleVel[idx] = newAngle / 4;
}

//------------------------------------------------------------------------------
void RadialParticleEmitter::Destroy()
{
  SAFE_ADELETE(_pos);
  SAFE_ADELETE(_angle);
  SAFE_ADELETE(_angleVel);
  SAFE_ADELETE(_deadParticles);
}

//------------------------------------------------------------------------------
void RadialParticleEmitter::Update(float dt)
{
  int numDead = 0;
  int* dead = _deadParticles;

  int diff = _numParticles - _spawnedParticles;
  if (diff > 0)
  {
    float ratio = 1.0f;
    int toSpawn = (int)(ratio * diff);
    for (int i = 0; i < toSpawn; ++i)
    {
      CreateParticle(i+_spawnedParticles, 100);
    }
    _spawnedParticles += toSpawn;
  }

  float rad_mult = g_Blackboard->GetFloatVar("radial.radius_mult");
  float rad_scale = g_Blackboard->GetFloatVar("radial.radius_scale");

  for (int i = 0, e = _spawnedParticles; i < e; ++i)
  {
    float angle = _angle[i] + dt * _angleVel[i];
    angle = fmodf(angle, XM_2PI);

    float dist = fabsf(angle - XM_PI);
    float ff = XM_PIDIV4;
    float y = 0;
    if (dist < ff) {
      float t = SmoothStep(0, 1, dist/ff);
      //y = _radius * cosf(t);
    }
    _angle[i] = angle;
    
    float newAngle = _angle[i];
    float x = _radius * sinf(newAngle);
    float z = _radius * cosf(newAngle);
    XMFLOAT4 newPos(x, y, z, 1);



    _pos[i] = XMLoadFloat4(&newPos);
  }

  float s = 100;
  for (int i = 0; i < numDead; ++i)
  {
    CreateParticle(_deadParticles[i], s);
  }
}

//------------------------------------------------------------------------------
void RadialParticleEmitter::CopyToBuffer(vec4* vtx)
{
  memcpy(vtx, _pos, _spawnedParticles * sizeof(vec4));
}

//------------------------------------------------------------------------------
void RadialParticleEmitter::UpdateEmitter(const TaskData& data)
{
  ::UpdateEmitter<RadialParticleEmitter>(data);
}

//------------------------------------------------------------------------------
void RadialParticleEmitter::CopyOutEmitter(const TaskData& data)
{
  ::CopyOutEmitter<RadialParticleEmitter>(data);
}
