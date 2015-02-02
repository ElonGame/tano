#pragma once

#include "../effect.hpp"
#include "../gpu_objects.hpp"
#include "../generated/demo.types.hpp"

namespace tano
{
  class RayMarcher : public Effect
  {
  public:

    RayMarcher(const string &name, u32 id);
    ~RayMarcher();
    virtual bool Init(const char* configFile) override;
    virtual bool Update(const UpdateState& state) override;
    virtual bool Render() override;
    virtual bool Close() override;

    static const char* Name();
    static Effect* Create(const char* name, u32 id);
    static void Register();

  private:

    void Reset();
    void RenderParameterSet();
    void SaveParameterSet();

    struct CBufferPerFrame
    {
      Vector2 dim;
      float time;
    };

    GpuState _raymarcherState;
    GpuObjects _raymarcherGpuObjects;

    ConstantBuffer<CBufferPerFrame> _cbPerFrame;
    RayMarcherSettings _settings;
    string _configName;
  };
}
