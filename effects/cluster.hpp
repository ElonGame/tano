#pragma once

#include "../base_effect.hpp"
#include "../gpu_objects.hpp"
#include "../generated/demo.types.hpp"
#include "../mesh_loader.hpp"
#include "../animation_helpers.hpp"

namespace tano
{
  class Cluster : public BaseEffect
  {
  public:

    Cluster(const string &name, u32 id);
    ~Cluster();
    virtual bool Init(const char* configFile) override;
    virtual bool Update(const UpdateState& state) override;
    virtual bool Render() override;
    virtual bool Close() override;
    virtual bool InitAnimatedParameters() override;

    static const char* Name();
    static BaseEffect* Create(const char* name, u32 id);
    static void Register();

  private:

    void Reset();
#if WITH_IMGUI
    void RenderParameterSet();
    void SaveParameterSet();
#endif

    void UpdateCameraMatrix(const UpdateState& state);

    struct CBufferPerFrame
    {
      Matrix world;
      Matrix view;
      Matrix proj;
      Matrix viewProj;
    };
    ConstantBuffer<CBufferPerFrame> _cbPerFrame;

    GpuState _clusterState;
    GpuObjects _clusterGpuObjects;

    ClusterSettings _settings;
    string _configName;
    MeshLoader _meshLoader;
  };
}
