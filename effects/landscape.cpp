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


inline bool PlaneDot(const Plane& p, const Vector3& v)
{
  return p.x * v.x + p.y * v.y + p.z * v.z + p.w < 0;
}

bool GridInsideFrustum(
  const Plane* planes,
  const Vector3& v0,
  const Vector3& v1,
  const Vector3& v2,
  const Vector3& v3,
  const Vector3& v4,
  const Vector3& v5,
  const Vector3& v6,
  const Vector3& v7)
{
  for (int i = 0; i < 6; ++i)
  {
    const Plane& p = planes[i];
    if (PlaneDot(p, v0) && PlaneDot(p, v1) && PlaneDot(p, v2) && PlaneDot(p, v3) &&
        PlaneDot(p, v4) && PlaneDot(p, v5) && PlaneDot(p, v6) && PlaneDot(p, v7))
      return false;
  }
  return true;
}

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

u8 culled[1024*1024];

void SlowAndSteady(int width, int height, const Matrix& viewProj, const Vector3& scale, float* verts, u32* numVerts)
{
  Plane planes[6];
  ExtractPlanes(planes, viewProj);

  Vector3 size(scale.x * ((float)width-1), 0, scale.z * ((float)height-1));
  Vector3 ofs(-size.x / 2, 0, -size.z/2);

  int triIdx = 0;
  int hOfs = height-1;

  Vector3 v0, v1, v2, v3;
  Vector3 n0, n1;

  memset(culled, 0, 1024*1024);
  int BLOCK_SIZE = 4;

  // create the vertices
  for (int i = 0; i < height-1; ++i) {

    for (int j = 0; j < width-1; ++j) {

      if (culled[i*width+j])
        continue;

      // 1--2
      // |  |
      // 0--3

      float xx0 = ofs.x + (float)(j+0) * scale.x;
      float xx1 = ofs.x + (float)(j+1) * scale.x;
      float zz0 = ofs.z + (float)(i+0) * scale.z;
      float zz1 = ofs.z + (float)(i+1) * scale.z;

      v0.x = xx0; v0.z = zz0;
      v1.x = xx0; v1.z = zz1;
      v2.x = xx1; v2.z = zz1;
      v3.x = xx1; v3.z = zz0;

      // frustum cull in grids
      if ((i % BLOCK_SIZE == 0) && (j % BLOCK_SIZE == 0))
      {
        float xx0 = ofs.x + (float)(j+0) * scale.x;
        float xx1 = ofs.x + (float)(j+BLOCK_SIZE) * scale.x;
        float zz0 = ofs.z + (float)(i+0) * scale.z;
        float zz1 = ofs.z + (float)(i+BLOCK_SIZE) * scale.z;

        if (!GridInsideFrustum(planes, 
          Vector3(xx0, +scale.y, zz0), Vector3(xx0, +scale.y, zz1), Vector3(xx1, +scale.y, zz1), Vector3(xx1, +scale.y, zz0),
          Vector3(xx0, -scale.y, zz0), Vector3(xx0, -scale.y, zz1), Vector3(xx1, -scale.y, zz1), Vector3(xx1, -scale.y, zz0)))
        {
          int ie = i+BLOCK_SIZE >= 1024 ? 1024 : i+BLOCK_SIZE;
          int je = j+BLOCK_SIZE >= 1024 ? 1024 : j+BLOCK_SIZE;
          for (int ii = i; ii < ie; ++ii)
          {
            for (int jj = j; jj < je; ++jj)
            {
              culled[ii*width+jj] = 1;
            }
          }
          continue;
        }
      }

#if 1
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
#else
      auto it = noiseCache.find(i*width+j);
      if (it == noiseCache.end())
      {
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

        noiseCache[i*width+j] ={ v0.y, v1.y, v2.y, v3.y, n0, n1 };
      }
      else
      {
        const NoiseValues& values = it->second;
        v0.y = values.v0;
        v1.y = values.v1;
        v2.y = values.v2;
        v3.y = values.v3;
        n0 = values.n0;
        n1 = values.n1;
      }
#endif


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


void AddLandscapeQuad(
  float xx0, float xx1, float zz0, float zz1,
  float scaleX, float scaleZ, float scaleY,
  float* verts)
{
  Vector3 v0(xx0, 0, zz0);
  Vector3 v1(xx0, 0, zz1);
  Vector3 v2(xx1, 0, zz1);
  Vector3 v3(xx1, 0, zz0);

  v0.y = scaleY * stb_perlin_noise3(scaleX * xx0, 0, scaleZ * zz0);
  v1.y = scaleY * stb_perlin_noise3(scaleX * xx0, 0, scaleZ * zz1);
  v2.y = scaleY * stb_perlin_noise3(scaleX * xx1, 0, scaleZ * zz1);
  v3.y = scaleY * stb_perlin_noise3(scaleX * xx1, 0, scaleZ * zz0);

  Vector3 e1, e2;
  e1 = v2 - v1;
  e2 = v0 - v1;
  Vector3 n0 = Cross(e1, e2);
  n0.Normalize();

  e1 = v0 - v3;
  e2 = v2 - v3;
  Vector3 n1 = Cross(e1, e2);
  n1.Normalize();

  // 0, 1, 3
  Vector3ToFloat(&verts[0], v0);
  Vector3ToFloat(&verts[3], n0);
  Vector3ToFloat(&verts[6], v1);
  Vector3ToFloat(&verts[9], n0);
  Vector3ToFloat(&verts[12], v3);
  Vector3ToFloat(&verts[15], n0);
  verts += 18;

  // 3, 1, 2
  Vector3ToFloat(&verts[0], v3);
  Vector3ToFloat(&verts[3], n1);
  Vector3ToFloat(&verts[6], v1);
  Vector3ToFloat(&verts[9], n1);
  Vector3ToFloat(&verts[12], v2);
  Vector3ToFloat(&verts[15], n1);
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

  INIT_FATAL(RESOURCE_MANAGER.LoadFile("gfx/landscape1.png", &buf));
  int x, y, n;
  u8* data = stbi_load_from_memory((const u8*)buf.data(), (int)buf.size(), &x, &y, &n, 4);

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
  SlowAndSteady(1024, 1024, _camera._view * _camera._proj, Vector3(10, 30, 10), buf, &_numVerts);
  _ctx->Unmap(_landscapeGpuObjects._vb);

  _ctx->SetGpuObjects(_landscapeGpuObjects);
  _ctx->SetGpuState(_landscapeState);

  //_ctx->Draw(_landscapeGpuObjects._numVerts, 0);
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
