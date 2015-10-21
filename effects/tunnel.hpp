#pragma once
#include "../base_effect.hpp"
#include "../camera.hpp"
#include "../generated/demo.types.hpp"
#include "../tano_math.hpp"
#include "../gpu_objects.hpp"
#include "../shaders/out/tunnel.lines_gstunnellines.cbuffers.hpp"
#include "../shaders/out/tunnel.lines_pstunnellines.cbuffers.hpp"
#include "../shaders/out/tunnel.composite_pscomposite.cbuffers.hpp"
#include "../shaders/out/tunnel.facecolor_vsface.cbuffers.hpp"
#include "../shaders/out/tunnel.facecolor_psface.cbuffers.hpp"
#include "../shaders/out/tunnel.particle_gsparticle.cbuffers.hpp"
#include "../shaders/out/tunnel.particle_psparticle.cbuffers.hpp"

namespace tano
{
  struct Snake
  {
    Snake(int dimX,
      int dimY,
      float segmentWidth,
      float segmentHeight,
      const vec3& anchor,
      const vec3& dir);

    void Update(float dt);
    int CopyOutLines(vec3* out);

    struct Particle
    {
      vec3 pos;
      vec3 lastPos;
      vec3 acc;
    };

    struct Constraint
    {
      u16 i0;
      u16 i1;
      float restLength;
    };

    vec3 _gravity = vec3{ 0, 0, 0 };
    float _damping = 0.99f;
    int _clothDimX, _clothDimY;
    float _segmentWidth, _segmentHeight;
    vec3 _anchor;
    vec3 _dir;
    int _numParticles = 0;
    float _forceAngle;

    vector<Particle> _particles;
    vector<Constraint> _constraints;
  };

  //------------------------------------------------------------------------------
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
    ConstantBufferBundle<cb::TunnelFacecolorV, cb::TunnelFacecolorP> _cbFace;

    GpuBundle _snakeBundle;

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

    ObjectHandle _particleTexture;
    GpuBundle _particleBundle;
    ConstantBufferBundle<void, cb::TunnelParticleP, cb::TunnelParticleG> _cbParticle;

    vector<Snake> _snakes;
  };
}
