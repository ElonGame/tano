#pragma once
#include "../base_effect.hpp"
#include "../camera.hpp"
#include "../generated/demo.types.hpp"

namespace tano
{
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

    struct FluidSim
    {
      enum
      {
        FLUID_SIZE = 256,
        FLUID_SIZE_PADDED = FLUID_SIZE + 2,
        FLUID_SIZE_SQ = FLUID_SIZE * FLUID_SIZE,
        FLUID_SIZE_PADDED_SQ = FLUID_SIZE_PADDED * FLUID_SIZE_PADDED,
      };

      int IX(int x, int y) { return x + FLUID_SIZE_PADDED * y; }

      void Update(const FixedUpdateState& state);

      void AddForces(const FixedUpdateState& state);
      void Diffuse(const FixedUpdateState& state);
      void Advect(const FixedUpdateState& state);

      void DensityStep(const FixedUpdateState& state);
      void VelocityStep(const FixedUpdateState& state);

      void BoundaryConditions();

      float forces[FLUID_SIZE_PADDED_SQ];
      float density0[FLUID_SIZE_PADDED_SQ];
      float density1[FLUID_SIZE_PADDED_SQ];
      float* d = density0;
      float* d0 = density1;

      V2 velocity[FLUID_SIZE_PADDED_SQ];
      V2* v = velocity;
    };

    FluidSim _sim;
    ObjectHandle _fluidTexture;

    FluidSettings _settings;
    FreeFlyCamera _camera;
  };
}