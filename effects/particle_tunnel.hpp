#pragma once

#include "../effect.hpp"
#include "../generated/demo.types.hpp"

namespace tano
{
  class DeferredContext;

  class ParticleTunnel : public Effect
  {
  public:

    ParticleTunnel(const string &name, u32 id);
    ~ParticleTunnel();
    virtual bool Show() override;
    virtual bool Hide() override;
    virtual bool Init(const char* configFile) override;
    virtual bool Update(const UpdateState& state) override;
    virtual bool Render() override;
    virtual bool Close() override;

    virtual bool SaveSettings() override;

    static const char* Name();
    static Effect* Create(const char* name, u32 id);
    static void Register();

  private:
    string _configName;
    ObjectHandle _texture;
    ObjectHandle _vs;
    ObjectHandle _ps;
    ObjectHandle _samplerState;
    ObjectHandle _cbuffer;

    ParticleTunnelSettings _settings;
  };
}
