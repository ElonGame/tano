#pragma once

#include "../base_effect.hpp"
#include "../gpu_objects.hpp"
#include "../generated/demo.types.hpp"
#include "../camera.hpp"
#include "../shaders/out/plexus_gslines.cbuffers.hpp"
#include "../shaders/out/plexus_pslines.cbuffers.hpp"
#include "../shaders/out/plexus.greets_vsgreets.cbuffers.hpp"
#include "../shaders/out/plexus.greets_psgreets.cbuffers.hpp"
#include "../shaders/out/plexus.compose_pscomposite.cbuffers.hpp"
#include "../shaders/out/plexus.sky_pssky.cbuffers.hpp"
#include "../random.hpp"

namespace tano
{
  //------------------------------------------------------------------------------
  struct GreesBlock
  {
    bool Init();
    void Update(const UpdateState& state);
    void CopyOut(vec3* verts);

    struct GreetsData
    {
      vector<u8> block;
      void CopyToTarget(vector<float>* target);
    };

    void Transition(const GreetsData& from, const GreetsData& to);

    vector<GreetsData> greetsData;
    vector<u8> curBlocks;
    vector<float> targetSize;
    vector<float> blockSize;
    vector<float> blockAcc;
    vector<float> blockVel;
    int greetsIdx = 0;
    int prevGreetsIdx = -1;
    int width, height;
    int curText = 0;
    int numGreetsCubes = 0;
  };

  //------------------------------------------------------------------------------
  class Plexus : public BaseEffect
  {
  public:

    Plexus(const string& name, const string& config, u32 id);
    ~Plexus();
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

    void PointsTest(const UpdateState& state);
    void CalcPoints(bool recalcEdges);
    enum { MAX_POINTS = 16 * 1024 };

    SimpleAppendBuffer<vec3, MAX_POINTS> _points;
    SimpleAppendBuffer<vec3, MAX_POINTS> _tris;
    int* _neighbours = nullptr;

    void UpdateGreets(const UpdateState& state);

    GpuBundle _plexusLineBundle;
    ConstantBufferBundle<void, cb::PlexusPS, cb::PlexusGS> _cbPlexus;

    GpuBundle _greetsBundle;
    ConstantBufferBundle<cb::PlexusGreetsV, cb::PlexusGreetsP> _cbGreets;

    GpuBundle _compositeBundle;
    ConstantBufferBundle<void, cb::PlexusComposeP> _cbComposite;

    ConstantBufferBundle<void, cb::PlexusSkyF> _cbSky;
    GpuBundle _skyBundle;

    PlexusSettings _settings;

    Camera _plexusCamera;
    Camera _greetsCamera;

    ObjectHandle _perlinTexture;

    int _numGreetsCubes = 0;
    GreesBlock _greetsBlock;
    RandomUniform _randomFloat;
  };
}
