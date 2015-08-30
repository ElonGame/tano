#pragma once
#include "../base_effect.hpp"
#include "../camera.hpp"
#include "../generated/demo.types.hpp"
#include "../shaders/out/blob.mesh_vsmesh.cbuffers.hpp"
#include "../shaders/out/blob.compose_pscomposite.cbuffers.hpp"
#include "../mesh_utils.hpp"

namespace tano
{
  class Blob : public BaseEffect
  {
  public:

    Blob(const string& name, const string& config, u32 id);
    ~Blob();
    virtual bool OnConfigChanged(const vector<char>& buf) override;
    virtual bool Init() override;
    virtual bool Update(const UpdateState& state) override;
    virtual bool FixedUpdate(const FixedUpdateState& state) override;
    virtual bool Render() override;
    virtual bool Close() override;
    virtual const char* GetName() { return Name(); }

    static const char* Name();
    static BaseEffect* Create(const char* name, const char* config, u32 id);
    static void Register();

  private:

#if WITH_IMGUI
    void RenderParameterSet();
    void SaveParameterSet(bool inc);
#endif

    void Reset();
    void UpdateCameraMatrix(const UpdateState& state);

    GpuBundle _meshBundle;
    ConstantBufferBundle<
      cb::BlobMeshF, void, void,
      cb::BlobMeshO, cb::BlobMeshO, void> _cbMesh;

    scene::Scene _scene;

    GpuBundle _compositeBundle;
    ConstantBufferBundle<void, cb::BlobComposeF> _cbComposite;

    BlobSettings _settings;
    FreeflyCamera _camera;
  };
}
