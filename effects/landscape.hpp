#pragma once

#include "../effect.hpp"
#include "../gpu_objects.hpp"
#include "../generated/demo.types.hpp"
#include "../mesh_loader.hpp"
#include "../animation_helpers.hpp"
#include "../camera.hpp"

namespace tano
{
  class Landscape : public Effect
  {
  public:

    Landscape(const string &name, u32 id);
    ~Landscape();
    virtual bool Init(const char* configFile) override;
    virtual bool Update(const UpdateState& state) override;
    virtual bool Render() override;
    virtual bool Close() override;
    virtual bool InitAnimatedParameters() override;

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
      Vector4 dim;
      Vector3 cameraPos;
      float pad1;
      Vector3 cameraLookAt;
      float pad2;
      Vector3 cameraUp;
    };
    ConstantBuffer<CBufferPerFrame> _cbPerFrame;

    GpuState _landscapeState;
    GpuObjects _landscapeGpuObjects;

    LandscapeSettings _settings;
    string _configName;
    MeshLoader _meshLoader;

    GpuObjects _edgeGpuObjects;
    GpuObjects _skyGpuObjects;

    GpuState _compositeState;
    GpuObjects _compositeGpuObjects;

    AnimatedInt _blinkFace;

    Camera _camera;
    u32 _numVerts;

  };
}
