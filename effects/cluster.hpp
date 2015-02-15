#pragma once

#include "../effect.hpp"
#include "../gpu_objects.hpp"
#include "../generated/demo.types.hpp"
#include "../mesh_loader.hpp"

namespace tano
{
  class Cluster : public Effect
  {
  public:

    Cluster(const string &name, u32 id);
    ~Cluster();
    virtual bool Init(const char* configFile) override;
    virtual bool Update(const UpdateState& state) override;
    virtual bool Render() override;
    virtual bool Close() override;

    static const char* Name();
    static Effect* Create(const char* name, u32 id);
    static void Register();

  private:

    void Reset();
#if WITH_IMGUI
    void RenderParameterSet();
    void SaveParameterSet();
#endif

    void UpdateCameraMatrix();

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
