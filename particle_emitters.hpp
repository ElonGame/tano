#pragma once
#include "tano_math.hpp"
#include "random.hpp"

namespace tano
{
  namespace scheduler
  {
    struct TaskData;
  }

  //------------------------------------------------------------------------------
  struct ParticleEmitter
  {
    void Create(const vec3& center, int numParticles);
    void Destroy();
    void Update(float dt);
    void CreateParticle(int idx, float s);
    void CopyToBuffer(vec4* vtx);

    XMVECTOR* _pos = nullptr;
    XMVECTOR* _vel = nullptr;

    int* _deadParticles = nullptr;

    int _numParticles = 0;
    vec3 _center = { 0, 0, 0 };

    struct EmitterKernelData
    {
      ParticleEmitter* emitter;
      float dt;
      vec4* vtx;
    };

    static void UpdateEmitter(const scheduler::TaskData& data);
    static void CopyOutEmitter(const scheduler::TaskData& data);
  };

  //------------------------------------------------------------------------------
  struct RadialParticleEmitter
  {
    void Create(const vec3& center, float radius, int numParticles);
    void Destroy();
    void Update(float dt);
    void CreateParticle(int idx, float s);
    void CopyToBuffer(vec4* vtx);

    XMVECTOR* _pos = nullptr;
    float* _angle = nullptr;
    float* _angleVel = nullptr;

    int* _deadParticles = nullptr;

    int _spawnedParticles = 0;
    int _numParticles = 0;
    float _radius = 10;
    vec3 _center = { 0, 0, 0 };

    struct EmitterKernelData
    {
      RadialParticleEmitter* emitter;
      float dt;
      vec4* vtx;
    };

    static void UpdateEmitter(const scheduler::TaskData& data);
    static void CopyOutEmitter(const scheduler::TaskData& data);
  };
}