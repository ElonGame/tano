#pragma once
#include "../base_effect.hpp"
#include "../camera.hpp"
#include "../generated/demo.types.hpp"
#include "../tano_math.hpp"
#include "../gpu_objects.hpp"
#include "../shaders/out/tunnel.lines_gstunnellines.cbuffers.hpp"
#include "../shaders/out/tunnel.lines_pstunnellines.cbuffers.hpp"
#include "../shaders/out/tunnel.composite_pscomposite.cbuffers.hpp"
#include "../shaders/out/tunnel.mesh_vsmesh.cbuffers.hpp"
#include "../scene.hpp"

namespace tano
{
  class Tunnel : public BaseEffect
  {
  public:

    Tunnel(const string& name, const string& config, u32 id);
    ~Tunnel();
    virtual bool OnConfigChanged(const vector<char>& buf) override;
    virtual bool Init() override;
    virtual bool Update(const UpdateState& state) override;
    virtual bool FixedUpdate(const FixedUpdateState& state) override;
    virtual bool Render() override;
    virtual bool Close() override;

    static const char* Name();
    static BaseEffect* Create(const char* name, const char* config, u32 id);
    static void Register();

  private:

#if WITH_IMGUI
    void RenderParameterSet();
    void SaveParameterSet();
#endif

    void Reset();
    void UpdateCameraMatrix(const UpdateState& state);

    void PlexusUpdate(const UpdateState& state);
    void NormalUpdate(const UpdateState& state);

    GpuBundle _linesBundle;
    GpuBundle _compositeBundle;

    ConstantBufferBundle<void, cb::TunnelLinesPS, cb::TunnelLinesGS> _cbLines;
    ConstantBufferBundle<void, cb::TunnelCompositeF> _cbComposite;

    ConstantBufferBundle<
      cb::TunnelMeshF, void, void,
      cb::TunnelMeshO, void, void> _cbMesh;

    SimpleAppendBuffer<V3, 64 * 1024> _tunnelVerts;

    float _dist = 0;
    CardinalSpline2 _spline;
    CardinalSpline2 _cameraSpline;
    TunnelSettings _settings;
    //Camera _camera;
    FreeFlyCamera _camera;

    GpuBundle _meshBundle;
    scene::Scene _scene;

  };

}