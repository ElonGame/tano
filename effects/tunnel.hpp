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
#include "../shaders/out/tunnel.facecolor_vsface.cbuffers.hpp"
#include "../shaders/out/tunnel.facecolor_psface.cbuffers.hpp"
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

    void PlexusUpdate(const UpdateState& state);

    u32 _numTunnelFaceIndices = 0;
    u32 _numTunnelFaces = 0;
    GpuBundle _tunnelFaceBundle;
    //ConstantBufferBundle<cb::TunnelFaceV, cb::TunnelFaceV, cb::TunnelFaceG> _cbFace;
    ConstantBufferBundle<cb::TunnelFacecolorV, cb::TunnelFacecolorP> _cbFace;

    GpuBundle _linesBundle;
    ConstantBufferBundle<void, cb::TunnelLinesPS, cb::TunnelLinesGS> _cbLines;

    GpuBundle _compositeBundle;
    ConstantBufferBundle<void, cb::TunnelCompositeF> _cbComposite;

    SimpleAppendBuffer<vec3, 64 * 1024> _tunnelPlexusVerts;
    SimpleAppendBuffer<vec3, 64 * 1024> _tunnelFaceVerts;

    bool _useFreeFly = false;
    float _dist = 0;
    CardinalSpline _spline;
    CardinalSpline _cameraSpline;
    TunnelSettings _settings;

    ConstantBufferBundle<
      cb::TunnelMeshF, void, void,
      cb::TunnelMeshO, void, void> _cbMesh;
    GpuBundle _meshBundle;
    scene::Scene _scene;

    float _bonusTime = 0;

  };
}
