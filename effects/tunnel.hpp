#pragma once
#include "../base_effect.hpp"
#include "../camera.hpp"
#include "../generated/demo.types.hpp"
#include "../tano_math.hpp"
#include "../gpu_objects.hpp"
#include "../shaders/out/plexus_gslines.cbuffers.hpp"
#include "../shaders/out/plexus_pslines.cbuffers.hpp"

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

    GpuBundle _tunnelBundle;

    ConstantBufferBundle<void, cb::PlexusPS, cb::PlexusGS> _cbTunnel;

    SimpleAppendBuffer<V3, 64 * 1024> _tunnelVerts;

    float _dist = 0;
    CardinalSpline2 _spline;
    TunnelSettings _settings;
    Camera _camera;
  };

}