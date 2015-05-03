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
    };
    ConstantBuffer<CBufferPerFrame> _cbPerFrame;

    void AddLine(const Vector3& p0, const Vector3& p1, float radius, float aspectRatio);

    struct LineElement
    {
      Vector3 p0;
      Vector3 p1;
      float radius;
      float aspectRatio;
      Vector4 weights;
    };

    static const u32 MAX_LINES = 16000;
    LineElement _lineElements[MAX_LINES];
    u32 _numLineTris = 0;
    u32 _vtxIndex = 0;

    ObjectHandle _lineTexture;
    GpuObjects _lineObjects;
    ObjectHandle _lineSampler;

    GpuState _blobState;
    GpuObjects _blobGpuObjects;
    BlobSettings _settings;
    string _configName;
    MeshLoader _meshLoader;

    Camera _camera;
  };
}
