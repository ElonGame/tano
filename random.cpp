#include "random.hpp"

#if WITH_IMGUI
#include "imgui/imgui_internal.h"
#endif

using namespace tano;
using namespace bristol;

static const int NUM_VALUES = 21347;

static float FLOAT_TABLE[NUM_VALUES];
static float GAUSS_1_01_TABLE[NUM_VALUES];
static float GAUSS_1_10_TABLE[NUM_VALUES];
static float GAUSS_1_50_TABLE[NUM_VALUES];
static float GAUSS_1_150_TABLE[NUM_VALUES];

static int INT_TABLE[NUM_VALUES];

static bool GenTables()
{
  for (int i = 0; i < NUM_VALUES; ++i)
  {
    FLOAT_TABLE[i] = randf(0.f, 1.f);
    GAUSS_1_01_TABLE[i] = GaussianRand(0.f, 0.10f);
    GAUSS_1_10_TABLE[i] = GaussianRand(0.f, 0.50f);
    GAUSS_1_50_TABLE[i] = GaussianRand(0.f, 1.50f);
    GAUSS_1_150_TABLE[i] = GaussianRand(0.f, 2.50f);
    INT_TABLE[i] = rand();
  }
  return true;
}

static bool tablesGenerated = GenTables();

//------------------------------------------------------------------------------
float RandomUniform::Next(float minValue, float maxValue)
{
  assert(maxValue >= minValue);
  float s = maxValue - minValue;
  float res = minValue + s * FLOAT_TABLE[_idx];
  _idx = (_idx + 1) % NUM_VALUES;
  return res;
}

//------------------------------------------------------------------------------
int RandomInt::Next()
{
  int res = INT_TABLE[_idx];
  _idx = (_idx + 1) % NUM_VALUES;
  return res;
}

//------------------------------------------------------------------------------
RandomGaussBase::RandomGaussBase(float* table) : _table(table)
{
}

//------------------------------------------------------------------------------
float RandomGaussBase::Next(float mean, float variance)
{
  float res = mean + variance * _table[_idx];
  _idx = (_idx + 1) % NUM_VALUES;
  return res;
}

//------------------------------------------------------------------------------
RandomGauss01::RandomGauss01() : RandomGaussBase(GAUSS_1_01_TABLE)
{
}

//------------------------------------------------------------------------------
RandomGauss10::RandomGauss10() : RandomGaussBase(GAUSS_1_10_TABLE)
{
}

//------------------------------------------------------------------------------
RandomGauss50::RandomGauss50() : RandomGaussBase(GAUSS_1_50_TABLE)
{
}

//------------------------------------------------------------------------------
RandomGauss150::RandomGauss150() : RandomGaussBase(GAUSS_1_150_TABLE)
{
}

#if WITH_IMGUI
//------------------------------------------------------------------------------
void tano::ShowRandomDistribution()
{
#define IM_ARRAYSIZE(_ARR) ((int)(sizeof(_ARR) / sizeof(*_ARR)))

  static bool opened = true;
  ImGui::Begin("Distribution");

  float* distTables[] = {
      GAUSS_1_01_TABLE, GAUSS_1_10_TABLE, GAUSS_1_50_TABLE, GAUSS_1_150_TABLE, FLOAT_TABLE};
  float scaleFactors[] = { 0.5f, 0.5f, 0.5f, 0.5f, 1.0f };
  float ofs[] = { 0.5f, 0.5f, 0.5f, 0.5f, 0 };
  static char* distributions[] = {
      "Gaussian 1/0.01", "Gaussian 1/0.10", "Gaussian 1/0.50", "Gaussian 1/1.50", "Uniform"};
  auto fnGetParam = [](void* data, int idx, const char** out_text)
  {
    *out_text = distributions[idx];
    return true;
  };

  static int curDist = 0;
  ImGui::Combo("Distribution", &curDist, fnGetParam, NULL, IM_ARRAYSIZE(distributions));

  int numBuckets = 2048;
  vector<float> values(numBuckets);
  for (int i = 0; i < NUM_VALUES; ++i)
  {
    float v = distTables[curDist][i];
    int idx = (int)(numBuckets * (scaleFactors[curDist] * v + ofs[curDist]));
    if (idx >= 0 && idx < numBuckets)
      values[idx] += 1;
  }

  ImGuiWindow* window = ImGui::GetCurrentWindow();

  ImVec2 size = ImGui::GetContentRegionAvail();
  ImGui::PlotHistogram("Distribution", values.data(), numBuckets, 0, NULL, FLT_MAX, FLT_MAX, size);

  ImGui::End();
}
#endif