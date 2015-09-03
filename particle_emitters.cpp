#include "particle_emitters.hpp"
#include "scheduler.hpp"

using namespace tano;
using namespace tano::scheduler;
using namespace bristol;

//------------------------------------------------------------------------------
void ParticleEmitter::Create(const V3& center, int numParticles)
{
  _center = center;
  _numParticles = numParticles;
  pos = new XMVECTOR[_numParticles];
  vel = new XMVECTOR[_numParticles];
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

  pos[idx] = XMLoadFloat4(&p);
  vel[idx] = XMLoadFloat4(&v);
}

//------------------------------------------------------------------------------
void ParticleEmitter::Destroy()
{
  SAFE_ADELETE(pos);
  SAFE_ADELETE(vel);
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
    XMVECTOR scaledVel = XMVectorScale(vel[i], dt);
    pos[i] = XMVectorAdd(pos[i], scaledVel);

    XMVECTOR res = XMVectorLess(pos[i], zClip);
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
void ParticleEmitter::CopyToBuffer(V4* vtx)
{
  memcpy(vtx, pos, _numParticles * sizeof(V4));
}

//------------------------------------------------------------------------------
void ParticleEmitter::UpdateEmitter(const TaskData& data)
{
  EmitterKernelData* emitterData = (EmitterKernelData*)data.kernelData.data;
  ParticleEmitter* emitter = emitterData->emitter;
  emitter->Update(emitterData->dt);
}

//------------------------------------------------------------------------------
void ParticleEmitter::CopyOutEmitter(const TaskData& data)
{
  EmitterKernelData* emitterData = (EmitterKernelData*)data.kernelData.data;
  ParticleEmitter* emitter = emitterData->emitter;
  emitter->CopyToBuffer(emitterData->vtx);
}
