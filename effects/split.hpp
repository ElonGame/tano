#pragma once
#include "../base_effect.hpp"
#include "../camera.hpp"
#include "../generated/demo.types.hpp"
#include "../gpu_objects.hpp"
#include "../shaders/out/trail.composite_pscomposite.cbuffers.hpp"
#include "../shaders/out/trail.background_psbackground.cbuffers.hpp"
#include "../shaders/out/split.mesh_vsmesh.cbuffers.hpp"

namespace tano
{
  struct Pathy
  {
    enum
    {
      TOTAL_POINTS = 32 * 1024,
      POINTS_PER_CHILD = 1024,
      MAX_NUM_LINES = 32
    };
    void Create();
    V3* CopyOut(V3* buf);

    void CreateTubesIncremental(float t);

    vector<V3> verts;
    vector<Vector3> tubeVerts;

    struct Segment
    {
      Segment(const Vector3& cur, float scale, float angleX, float angleY, float angleZ)
          : cur(cur), scale(scale), angleX(angleX), angleY(angleY), angleZ(angleZ)
      {
      }
      Vector3 cur;
      float scale;
      float angleX, angleY, angleZ;
      int lastNumTicks = 0;
      vector<V3> verts;
      vector<Vector3> completeRings;
      vector<Vector3> inprogressRing;
      CardinalSpline2 spline;

      // last reference frame
      V3 frameD, frameN, frameT;
    };

    float len = 5;

    float angleXMean = 0.f;
    float angleXVariance = 1.f;

    float angleYMean = 0.f;
    float angleYVariance = 1.f;

    float angleZMean = 0.f;
    float angleZVariance = 1.f;

    float childProb = 0.75f;
    float childScale = 0.50f;

    int maxChildren = 512;

    struct Line
    {
      int startOfs;
      int size;
    };

    struct SegmentStart
    {
      float time;
      Segment* segment;
    };
    deque<SegmentStart> segmentStart;
    vector<Segment*> segments;

    vector<Line> lines;
  };

  class Split : public BaseEffect
  {
  public:
    Split(const string& name, const string& config, u32 id);
    ~Split();
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

    Pathy _pathy;

    GpuBundle _backgroundBundle;
    ConstantBufferBundle<void, cb::TrailBackgroundF> _cbBackground;

    GpuBundle _compositeBundle;
    ConstantBufferBundle<void, cb::TrailCompositeP> _cbComposite;

    ConstantBufferBundle<
      cb::SplitMeshF, void, void,
      cb::SplitMeshO, void, void> _cbMesh;

    GpuBundle _meshBundle;

    SplitSettings _settings;
    FreeflyCamera _camera;
  };
}