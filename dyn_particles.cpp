#include "dyn_particles.hpp"
#include "update_state.hpp"

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
