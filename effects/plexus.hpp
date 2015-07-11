#pragma once

#include "../base_effect.hpp"
#include "../gpu_objects.hpp"
#include "../generated/demo.types.hpp"
#include "../camera.hpp"
#include "../text_writer.hpp"

namespace tano
{
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

    void PointsTest(const UpdateState& state);

    void CalcText();
    void TextTest(const UpdateState& state);
    void CalcPoints(bool recalcEdges);
    int CalcLines(V3* vtx);
    enum { MAX_POINTS = 16 * 1024 };

    SimpleAppendBuffer<V3, MAX_POINTS> _points;
    SimpleAppendBuffer<V3, MAX_POINTS> _tris;
    int* _neighbours = nullptr;

    void UpdateNoise();

    bool _renderPoints = false;
    GpuBundle _pointBundle;
    GpuBundle _plexusLineBundle;

    struct CBufferBasic
    {
      Matrix world;
      Matrix view;
      Matrix proj;
      Matrix viewProj;
      Vector4 cameraPos;
      Vector4 dim;
    };
    ConstantBuffer<CBufferBasic> _cbBasic;

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
      Vector4 cook;
      Color diffuse;
    };
    ConstantBuffer<CBufferPerFrame> _cbPerFrame;

    TextWriter _textWriter;
    vector<V3> _textVerts;
    vector<int> _textIndices;

    PlexusSettings _settings;

    ObjectHandle _perlinTexture;

    FreeFlyCamera _camera;
  };
}
