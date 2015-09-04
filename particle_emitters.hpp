#pragma once
#include "tano_math.hpp"

namespace tano
{
  namespace scheduler
  {
    struct TaskData;
  }

  //------------------------------------------------------------------------------
  struct ParticleEmitter
  {
    void Create(const V3& center, int numParticles);
    void Destroy();
    void Update(float dt);
    void CreateParticle(int idx, float s);
    void CopyToBuffer(V4* vtx);

    XMVECTOR* _pos = nullptr;
    XMVECTOR* _vel = nullptr;

    int* _deadParticles = nullptr;

    int _numParticles = 0;
    V3 _center = { 0, 0, 0 };

    struct EmitterKernelData
    {
      ParticleEmitter* emitter;
      float dt;
      V4* vtx;
    };

    static void UpdateEmitter(const scheduler::TaskData& data);
    static void CopyOutEmitter(const scheduler::TaskData& data);
  };

  //------------------------------------------------------------------------------
  struct RadialParticleEmitter
  {
    void Create(const V3& center, float radius, int numParticles);
    void Destroy();
    void Update(float dt);
    void CreateParticle(int idx, float s);
    void CopyToBuffer(V4* vtx);

    XMVECTOR* _pos = nullptr;
    float* _angle = nullptr;
    float* _angleVel = nullptr;

    int* _deadParticles = nullptr;

    int _spawnedParticles = 0;
    int _numParticles = 0;
    float _radius = 10;
    V3 _center = { 0, 0, 0 };

    struct EmitterKernelData
    {
      RadialParticleEmitter* emitter;
      float dt;
      V4* vtx;
    };

    static void UpdateEmitter(const scheduler::TaskData& data);
    static void CopyOutEmitter(const scheduler::TaskData& data);
  };

}