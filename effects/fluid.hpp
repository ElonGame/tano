#pragma once
#include "../base_effect.hpp"
#include "../camera.hpp"
#include "../generated/demo.types.hpp"
#include "../gpu_objects.hpp"

namespace tano
{
  namespace scheduler
  {
    struct TaskData;
  }

  //------------------------------------------------------------------------------
  struct FluidSim
  {
    enum
    {
      FLUID_SIZE = 100,
      FLUID_SIZE_PADDED = FLUID_SIZE + 2,
      FLUID_SIZE_SQ = FLUID_SIZE * FLUID_SIZE,
      FLUID_SIZE_PADDED_SQ = FLUID_SIZE_PADDED * FLUID_SIZE_PADDED,
    };

    FluidSim();

    static int IX(int x, int y) { return x + FLUID_SIZE_PADDED * y; }

    void Update(const UpdateState& state);

    void AddForce(float dt, float* out, float* force);
    void Diffuse(int b, float dt, float diff, float* out, float* old);
    void Advect(int b, float dt, float* out, float* old, float* u, float *v);
    void Project(float* u, float* v, float* p, float* div);

    void DensityStep(float dt);
    void VelocityStep(float dt);

    void BoundaryConditions(int b, float* x);

    void Verify();

    struct FluidKernelChunk
    {
      int yStart, yEnd;
      float* out;
      float* old;
      float dt;
      float diff;
    };

    static void FluidKernelWorker(const scheduler::TaskData& td);

    float density0[FLUID_SIZE_PADDED_SQ];
    float density1[FLUID_SIZE_PADDED_SQ];
    float* dCur = density0;
    float* dOld = density1;

    float u0[FLUID_SIZE_PADDED_SQ];
    float u1[FLUID_SIZE_PADDED_SQ];
    float* uCur = u0;
    float* uOld = u1;

    float v0[FLUID_SIZE_PADDED_SQ];
    float v1[FLUID_SIZE_PADDED_SQ];
    float* vCur = v0;
    float* vOld = v1;
  };

  class Fluid : public BaseEffect
  {
  public:

    Fluid(const string& name, const string& config, u32 id);
    ~Fluid();
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

    void UpdateFluidTexture();

    void Reset();
    void UpdateCameraMatrix(const UpdateState& state);
    void UpdateBackgroundTexture(float dt);
    void InitBackgroundTexture();

    vector<float> _textureU;
    vector<float> _textureV;

    GpuBundle _backgroundBundle;
    ObjectHandle _backgroundTexture;

    FluidSim _sim;
    ObjectHandle _fluidTexture;

    FluidSettings _settings;
    FreeFlyCamera _camera;

  };
}