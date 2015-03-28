#include "landscape.hpp"
#include "../tano.hpp"
#include "../graphics.hpp"
#include "../graphics_context.hpp"
#include "../demo_engine.hpp"
#include "../resource_manager.hpp"
#include "../init_sequence.hpp"
#include "../generated/demo.parse.hpp"
#include "../generated/input_buffer.hpp"
#include "../generated/output_buffer.hpp"
#include "../mesh_utils.hpp"
#include "../post_process.hpp"

using namespace tano;
using namespace bristol;

extern "C" float stb_perlin_noise3(float x, float y, float z, int x_wrap=0, int y_wrap=0, int z_wrap=0);


void Vector3ToFloat(float* buf, const Vector3& v)
{
  buf[0] = v.x;
  buf[1] = v.y;
  buf[2] = v.z;
}

struct NoiseValues
{
  float v0, v1, v2, v3;
  Vector3 n0, n1;
};

unordered_map<u32, NoiseValues> noiseCache;

void Rasterize(
  const Vector3& scale,
  int startZ, const vector<pair<int, int>>& spans, 
  float* verts, u32* numVerts)
{
  Vector3 v0, v1, v2, v3;
  Vector3 n0, n1;
  Vector3 size(10 * 1024, 10 * 1024, 10 * 1024);

  int triIdx = 0;
  for (int idx = 0; idx < (int)spans.size(); ++idx)
  {
    int i = startZ + idx;
    int a = spans[idx].first;
    int b = spans[idx].second;
    for (int j = a; j <= b; ++j)
    {
      // 1--2
      // |  |
      // 0--3

      float xx0 = (float)(j+0) * scale.x;
      float xx1 = (float)(j+1) * scale.x;
      float zz0 = (float)(i+0) * scale.z;
      float zz1 = (float)(i+1) * scale.z;

      v0.x = xx0; v0.z = zz0;
      v1.x = xx0; v1.z = zz1;
      v2.x = xx1; v2.z = zz1;
      v3.x = xx1; v3.z = zz0;

      v0.y = scale.y * stb_perlin_noise3(256 * xx0 / size.x, 0, 256 * zz0 / size.z);
      v1.y = scale.y * stb_perlin_noise3(256 * xx0 / size.x, 0, 256 * zz1 / size.z);
      v2.y = scale.y * stb_perlin_noise3(256 * xx1 / size.x, 0, 256 * zz1 / size.z);
      v3.y = scale.y * stb_perlin_noise3(256 * xx1 / size.x, 0, 256 * zz0 / size.z);

      Vector3 e1, e2;
      e1 = v2 - v1;
      e2 = v0 - v1;
      n0 = Cross(e1, e2);
      n0.Normalize();

      e1 = v0 - v3;
      e2 = v2 - v3;
      n1 = Cross(e1, e2);
      n1.Normalize();


      // 0, 1, 3
      Vector3ToFloat(&verts[triIdx*18+0], v0);
      Vector3ToFloat(&verts[triIdx*18+3], n0);
      Vector3ToFloat(&verts[triIdx*18+6], v1);
      Vector3ToFloat(&verts[triIdx*18+9], n0);
      Vector3ToFloat(&verts[triIdx*18+12], v3);
      Vector3ToFloat(&verts[triIdx*18+15], n0);
      ++triIdx;

      Vector3ToFloat(&verts[triIdx*18+0], v3);
      Vector3ToFloat(&verts[triIdx*18+3], n1);
      Vector3ToFloat(&verts[triIdx*18+6], v1);
      Vector3ToFloat(&verts[triIdx*18+9], n1);
      Vector3ToFloat(&verts[triIdx*18+12], v2);
      Vector3ToFloat(&verts[triIdx*18+15], n1);
      ++triIdx;
    }
  }

  *numVerts = triIdx * 3;
}

//------------------------------------------------------------------------------
Landscape::Landscape(const string &name, u32 id)
  : Effect(name, id)
  , _blinkFace("blinkface")
{
#if WITH_IMGUI
  PROPERTIES.Register(Name(),
    bind(&Landscape::RenderParameterSet, this),
    bind(&Landscape::SaveParameterSet, this));

  PROPERTIES.SetActive(Name());
#endif
}

//------------------------------------------------------------------------------
Landscape::~Landscape()
{
}

//------------------------------------------------------------------------------
bool Landscape::Init(const char* configFile)
{
  BEGIN_INIT_SEQUENCE();

  _configName = configFile;
  vector<char> buf;
  INIT_FATAL(RESOURCE_MANAGER.LoadFile(configFile, &buf));

  INIT(ParseLandscapeSettings(InputBuffer(buf), &_settings));
  _camera._pitch = _settings.camera.pitch;
  _camera._yaw = _settings.camera.yaw;
  _camera._roll = _settings.camera.roll;
  _camera._pos = _settings.camera.pos;

//   INIT_FATAL(RESOURCE_MANAGER.LoadFile("gfx/landscape1.png", &buf));
//   int x, y, n;
//   u8* data = stbi_load_from_memory((const u8*)buf.data(), (int)buf.size(), &x, &y, &n, 4);

  // create mesh from landscape
  u32 vertexFlags = VF_POS | VF_NORMAL;
  //INIT(CreateBuffersFromBitmapFaceted(data, x, y, Vector3(10, 30, 10), &vertexFlags, &_landscapeGpuObjects));
  u32 vertexSize = sizeof(PosNormal);
  INIT(_landscapeGpuObjects.CreateDynamicVb(1024*1024*6*vertexSize, vertexSize));

  INIT(_landscapeGpuObjects.LoadShadersFromFile("shaders/out/landscape", "VsLandscape", nullptr, "PsLandscape", vertexFlags));
  INIT(_landscapeState.Create());

  INIT(_edgeGpuObjects.LoadShadersFromFile("shaders/out/landscape", "VsQuad", nullptr, "PsEdgeDetect"));
  INIT(_skyGpuObjects.LoadShadersFromFile("shaders/out/landscape", "VsQuad", nullptr, "PsSky"));

  INIT(_compositeGpuObjects.LoadShadersFromFile("shaders/out/landscape", "VsQuad", nullptr, "PsComposite"));
  INIT(_compositeState.Create());

  int w, h;
  INIT(_cbPerFrame.Create());
  GRAPHICS.GetBackBufferSize(&w, &h);

//  _camera._pos = Vector3(0, 200, 500);

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool Landscape::Update(const UpdateState& state)
{
  rmt_ScopedCPUSample(Landscape_Update);

  UpdateCameraMatrix();
  return true;
}

//------------------------------------------------------------------------------
void Landscape::UpdateCameraMatrix()
{
  _camera.Update();
  Matrix view = _camera._view;
  Matrix proj = _camera._proj;
  Matrix viewProj = view * proj;

  // compute size of frustum
  float farW = _camera._farPlane * tan(_camera._fov);
  Vector3 v0(-farW, 0, _camera._farPlane);
  Vector3 v1(+farW, 0, _camera._farPlane);

  float nearW = _camera._nearPlane * tan(_camera._fov);
  Vector3 v2(-nearW, 0, _camera._nearPlane);
  Vector3 v3(+nearW, 0, _camera._nearPlane);

  v0 = Vector3::Transform(v0 + _camera._pos, _camera._mtx);
  v1 = Vector3::Transform(v1 + _camera._pos, _camera._mtx);
  v2 = Vector3::Transform(v2 + _camera._pos, _camera._mtx);
  v3 = Vector3::Transform(v3 + _camera._pos, _camera._mtx);

  _cbPerFrame.world = Matrix::Identity();
  _cbPerFrame.view = view.Transpose();
  _cbPerFrame.proj = proj.Transpose();
  _cbPerFrame.viewProj = viewProj.Transpose();
  _cbPerFrame.cameraPos = _camera._pos;
  _cbPerFrame.cameraLookAt = _camera._target;
  _cbPerFrame.cameraUp = _camera._up;
}
//------------------------------------------------------------------------------
void Landscape::RasterizeLandscape(float* buf)
{
  Vector3 corners[6];
  _camera.GetFrustumCenter(&corners[0]);

  Plane planes[6];
  ExtractPlanes(_camera._view * _camera._proj, true, planes);

  float ofs = 2 * _camera._farPlane;
  Vector3 c = _camera._pos;
  c.y = 0;
  Vector3 buf0[16] ={
    c + Vector3(-ofs, 0, +ofs),
    c + Vector3(+ofs, 0, +ofs),
    c + Vector3(+ofs, 0, -ofs),
    c + Vector3(-ofs, 0, -ofs)
  };
  Vector3 buf1[16];

  int numVerts = 4;
  for (int i = 0; i < 6; ++i)
  {
    numVerts = ClipPolygonAgainstPlane(numVerts, buf0, planes[i], buf1);
    if (numVerts == 0)
      break;
    memcpy(buf0, buf1, numVerts * sizeof(Vector3));
  }

  Vector3 minValues = buf0[0];
  Vector3 maxValues = buf0[0];

  for (int i = 1; i < numVerts; ++i)
  {
    minValues = Vector3::Min(minValues, buf0[i]);
    maxValues = Vector3::Max(maxValues, buf0[i]);
  }

  // create span array, and scan convert all the edges
  int maxZ = lround(maxValues.z / 10);
  int minZ = lround(minValues.z / 10);
  int sizeZ = maxZ - minZ + 1;
  vector<pair<int, int>> spans(sizeZ);

  for (int i = 0; i < sizeZ; ++i)
  {
    spans[i].first = INT_MAX;
    spans[i].second = -INT_MAX;
  }

  for (int i = 0; i < numVerts; ++i)
  {
    Vector3 vv0 = buf0[i] / 10;
    Vector3 vv1 = buf0[(i+1) % numVerts] / 10;

    Vector3& v0 = vv0.z > vv1.z ? vv0 : vv1;
    Vector3& v1 = vv0.z > vv1.z ? vv1 : vv0;

    int sy = lround(v0.z);
    int ey = lround(v1.z);
    int cy = sy - ey;
    if (cy == 0)
      continue;

    float dz = fabsf(v1.z - v0.z);
    float dx = v1.x - v0.x;
    float dxdz = dx / dz;

    float z = v0.z;
    float x = v0.x;

    for (int y = sy; y >= ey; --y)
    {
      int intX = lround(x);
      int yy = y - minZ;
      spans[yy].first = min(spans[yy].first, intX);
      spans[yy].second = max(spans[yy].second, intX);
      x += dxdz;
    }
  }

  Rasterize(Vector3(10, 30, 10), minZ, spans, buf, &_numVerts);
}

//------------------------------------------------------------------------------
bool Landscape::Render()
{
  rmt_ScopedCPUSample(Landscape_Render);
  static Color black(0, 0, 0, 0);
  PostProcess* postProcess = GRAPHICS.GetPostProcess();

  ScopedRenderTarget rt(DXGI_FORMAT_R16G16B16A16_FLOAT, BufferFlags(BufferFlag::CreateSrv) | BufferFlag::CreateDepthBuffer);

  u32 dimX, dimY;
  GRAPHICS.GetTextureSize(rt._handle, &dimX, &dimY);
  _cbPerFrame.dim.x = (float)dimX;
  _cbPerFrame.dim.y = (float)dimY;
  _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::VertexShader, 0);
  _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::PixelShader, 0);

  // Render the sky
  _ctx->SetRenderTarget(rt._handle, &black);
  _ctx->SetGpuObjects(_skyGpuObjects);
  _ctx->Draw(3, 0);

  // Render the landscape
  _ctx->SetRenderTarget(rt._handle, nullptr);

  float* buf = _ctx->MapWriteDiscard<float>(_landscapeGpuObjects._vb);
  RasterizeLandscape(buf);
  _ctx->Unmap(_landscapeGpuObjects._vb);

  _ctx->SetGpuObjects(_landscapeGpuObjects);
  _ctx->SetGpuState(_landscapeState);

  _ctx->Draw(_numVerts, 0);

  // outline
  ScopedRenderTarget rtOutline(DXGI_FORMAT_R16G16B16A16_FLOAT);
  postProcess->Execute({ rt._handle }, rtOutline._handle, _edgeGpuObjects._ps, false);

  postProcess->Execute({ rt._handle, rtOutline._handle }, 
    GRAPHICS.GetBackBuffer(), _compositeGpuObjects._ps, false);

  return true;
}

//------------------------------------------------------------------------------
bool Landscape::InitAnimatedParameters()
{
  return true;
}

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Landscape::RenderParameterSet()
{
  ImGui::InputInt("NumVerts", (int*)&_numVerts);
  ImGui::InputFloat("Pitch", &_camera._pitch);

  if (ImGui::Button("Reset"))
    Reset();

}
#endif

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Landscape::SaveParameterSet()
{
  OutputBuffer buf;
  _settings.camera.pos = _camera._pos;
  _settings.camera.yaw = _camera._yaw;
  _settings.camera.pitch = _camera._pitch;
  _settings.camera.roll = _camera._roll;
  Serialize(buf, _settings);
  if (FILE* f = fopen(_configName.c_str(), "wt"))
  {
    fwrite(buf._buf.data(), 1, buf._ofs, f);
    fclose(f);
  }
}
#endif

//------------------------------------------------------------------------------
void Landscape::Reset()
{
  _camera._pos = Vector3(0.f, 0.f, 0.f);
  _camera._pitch = _camera._yaw = _camera._roll = 0.f;
}

//------------------------------------------------------------------------------
bool Landscape::Close()
{
  return true;
}

//------------------------------------------------------------------------------
Effect* Landscape::Create(const char* name, u32 id)
{
  return new Landscape(name, id);
}

//------------------------------------------------------------------------------
const char* Landscape::Name()
{
  return "landscape";
}

//------------------------------------------------------------------------------
void Landscape::Register()
{
  DEMO_ENGINE.RegisterFactory(Name(), Landscape::Create);
}
