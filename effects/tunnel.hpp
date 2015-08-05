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
#include "../shaders/out/tunnel.greets_vsgreets.cbuffers.hpp"
#include "../shaders/out/tunnel.greets_psgreets.cbuffers.hpp"
#include "../scene.hpp"

namespace tano
{
  //------------------------------------------------------------------------------
  struct GreetsBlock
  {
    ~GreetsBlock();

    struct PathElem
    {
      PathElem(int x, int y) : x(x), y(y) {}
      int x, y;
    };

    struct Particle
    {
      int x, y;
      float speed;
      float cur;
      int dir;
    };

    struct GreetsData
    {
      GreetsData(int w, int h);
      ~GreetsData();
      void CalcPath(int w, int h, const char* buf);

      void Update(const UpdateState& state);

      bool IsValid(int x, int y);

      vector<Particle> particles;

      vector<vector<PathElem*>> paths;
      vector<PathElem*> startingPoints;

      vector<float> curParticleCount;
      vector<float> targetParticleCount;
      vector<u8> background;
      int width, height;
    };

    void Update(const UpdateState& state);
    bool Init();
    void Reset();

    vector<GreetsData*> _data;
    int curText = 0;
  };

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
    void SaveParameterSet();
#endif

    void Reset();
    void UpdateCameraMatrix(const UpdateState& state);

    void PlexusUpdate(const UpdateState& state);
    void NormalUpdate(const UpdateState& state);

    void UpdateGreets(const UpdateState& state);

    GpuBundle _linesBundle;
    GpuBundle _compositeBundle;

    ConstantBufferBundle<void, cb::TunnelLinesPS, cb::TunnelLinesGS> _cbLines;
    ConstantBufferBundle<void, cb::TunnelCompositeF> _cbComposite;

    SimpleAppendBuffer<V3, 64 * 1024> _tunnelVerts;

    ConstantBufferBundle<cb::TunnelGreetsV, cb::TunnelGreetsP> _cbGreets;

    float _dist = 0;
    CardinalSpline2 _spline;
    CardinalSpline2 _cameraSpline;
    TunnelSettings _settings;
    //Camera _camera;
    FreeflyCamera _camera;

    ConstantBufferBundle<
      cb::TunnelMeshF, void, void,
      cb::TunnelMeshO, void, void> _cbMesh;
    GpuBundle _meshBundle;
    scene::Scene _scene;

    int _numGreetsCubes = 0;
    GpuBundle _greetsBundle;
    GreetsBlock _greetsBlock;

  };

}