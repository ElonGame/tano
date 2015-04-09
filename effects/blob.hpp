#pragma once

#include "../effect.hpp"
#include "../gpu_objects.hpp"
#include "../generated/demo.types.hpp"
#include "../mesh_loader.hpp"
#include "../animation_helpers.hpp"
#include "../camera.hpp"

namespace tano
{
  class Blob : public Effect
  {
  public:

    Blob(const string &name, u32 id);
    ~Blob();
    virtual bool Init(const char* configFile) override;
    virtual bool Update(const UpdateState& state) override;
    virtual bool Render() override;
    virtual bool Close() override;
    virtual bool InitAnimatedParameters() override;

    static const char* Name();
    static Effect* Create(const char* name, u32 id);
    static void Register();

  private:

    void InitLines();
    ObjectHandle _lineTexture;
    GpuObjects _lineObjects;

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
      float pad3;
      Vector4 cook = Vector4(0.1f, 0.1f, 0, 0);
      //float roughness_value = 0.1f;
      //float ref_at_norm_incidence = 0.1f;
    };
    ConstantBuffer<CBufferPerFrame> _cbPerFrame;

    GpuState _blobState;
    GpuObjects _blobGpuObjects;
    BlobSettings _settings;
    string _configName;
    MeshLoader _meshLoader;

    Camera _camera;
  };
}
