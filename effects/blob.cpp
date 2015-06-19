#include "blob.hpp"
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

//------------------------------------------------------------------------------
Blob::Blob(const string &name, u32 id)
  : Effect(name, id)
{
#if WITH_IMGUI
  PROPERTIES.Register(Name(),
    bind(&Blob::RenderParameterSet, this),
    bind(&Blob::SaveParameterSet, this));

  PROPERTIES.SetActive(Name());
#endif
}

//------------------------------------------------------------------------------
Blob::~Blob()
{
}

//------------------------------------------------------------------------------
bool Blob::Init(const char* configFile)
{
  BEGIN_INIT_SEQUENCE();

  _configName = configFile;
  vector<char> buf;
  INIT_FATAL(RESOURCE_MANAGER.LoadFile(configFile, &buf));

  INIT(ParseBlobSettings(InputBuffer(buf), &_settings));
  _camera._pitch = _settings.camera.pitch;
  _camera._yaw = _settings.camera.yaw;
  _camera._roll = _settings.camera.roll;
  _camera._pos = _settings.camera.pos;

  // load the blob mesh
  MeshLoader loader;
  INIT(loader.Load("gfx/crystals_flat.boba"));
  INIT(CreateScene(loader, &_scene));
//   u32 blobVertexFlags = 0;
//   INIT(CreateBuffersFromMesh(loader, nullptr, &blobVertexFlags, &_blobGpuObjects));
   //INIT(_blobGpuObjects.LoadShadersFromFile("shaders/out/blob", "VsMesh", nullptr, "PsMesh", _scene.meshes[0].vertexFormat));
  {
    CD3D11_RASTERIZER_DESC rasterDesc = CD3D11_RASTERIZER_DESC(CD3D11_DEFAULT());
    rasterDesc.CullMode = D3D11_CULL_NONE;
    INIT(_blobState.Create(nullptr, nullptr, &rasterDesc));
  }

  INIT(_cbPerFrame.Create());
  int w, h;
  GRAPHICS.GetBackBufferSize(&w, &h);
  _cbPerFrame.dim.x = (float)w;
  _cbPerFrame.dim.y = (float)h;

  InitLines();
  vector<D3D11_INPUT_ELEMENT_DESC> elementDesc = {
    CD3D11_INPUT_ELEMENT_DESC("POSITION", DXGI_FORMAT_R32G32B32_FLOAT),
    CD3D11_INPUT_ELEMENT_DESC("POSITION", DXGI_FORMAT_R32G32B32_FLOAT),
    CD3D11_INPUT_ELEMENT_DESC("TEXCOORD", DXGI_FORMAT_R32_FLOAT),
    CD3D11_INPUT_ELEMENT_DESC("TEXCOORD", DXGI_FORMAT_R32_FLOAT),
    CD3D11_INPUT_ELEMENT_DESC("TEXCOORD", DXGI_FORMAT_R32G32B32A32_FLOAT),
  };

  INIT(_lineObjects.LoadVertexShader("shaders/out/lines", "VsMain", 0, &elementDesc));
  INIT(_lineObjects.LoadPixelShader("shaders/out/lines", "PsMain"));

  // each line is 6 tries, so 18 verts
  vector<int> lineIndices(MAX_LINES * 3 * 6);
  u32 idx = 0;
  for (int i = 0; i < (int)MAX_LINES; ++i)
  {
    lineIndices[idx++] = i * 8;
    lineIndices[idx++] = i * 8 + 2;
    lineIndices[idx++] = i * 8 + 3;
    lineIndices[idx++] = i * 8;
    lineIndices[idx++] = i * 8 + 3;
    lineIndices[idx++] = i * 8 + 1;

    lineIndices[idx++] = i * 8 + 2;
    lineIndices[idx++] = i * 8 + 4;
    lineIndices[idx++] = i * 8 + 5;
    lineIndices[idx++] = i * 8 + 2;
    lineIndices[idx++] = i * 8 + 5;
    lineIndices[idx++] = i * 8 + 3;

    lineIndices[idx++] = i * 8 + 4;
    lineIndices[idx++] = i * 8 + 6;
    lineIndices[idx++] = i * 8 + 7;
    lineIndices[idx++] = i * 8 + 4;
    lineIndices[idx++] = i * 8 + 7;
    lineIndices[idx++] = i * 8 + 5;
  }

  _lineObjects.CreateIndexBuffer((u32)lineIndices.size() * sizeof(int), DXGI_FORMAT_R32_UINT, lineIndices.data());
  _lineObjects.CreateDynamicVb(MAX_LINES * 8 * sizeof(LineElement), sizeof(LineElement));

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
void Blob::AddLine(const Vector3& p0, const Vector3& p1, float radius, float aspectRatio)
{
  assert(_vtxIndex + 8 <= MAX_LINES);

  _lineElements[_vtxIndex++] = { p0, p1, radius, aspectRatio, { 1.0f, 0.0f, -1.0f, -1.0f } };
  _lineElements[_vtxIndex++] = { p0, p1, radius, aspectRatio, { 1.0f, 0.0f, -1.0f, 1.0f } };
  _lineElements[_vtxIndex++] = { p0, p1, radius, aspectRatio, { 1.0f, 0.0f, 0.0f, -1.0f } };
  _lineElements[_vtxIndex++] = { p0, p1, radius, aspectRatio, { 1.0f, 0.0f, 0.0f, 1.0f } };
  _lineElements[_vtxIndex++] = { p0, p1, radius, aspectRatio, { 0.0f, 1.0f, 0.0f, -1.0f } };
  _lineElements[_vtxIndex++] = { p0, p1, radius, aspectRatio, { 0.0f, 1.0f, 0.0f, 1.0f } };
  _lineElements[_vtxIndex++] = { p0, p1, radius, aspectRatio, { 0.0f, 1.0f, 1.0f, -1.0f } };
  _lineElements[_vtxIndex++] = { p0, p1, radius, aspectRatio, { 0.0f, 1.0f, 1.0f, 1.0f } };

  _numLineTris += 6;
}

//------------------------------------------------------------------------------
void Blob::InitLines()
{
  int textureSize = 16;
  vector<u32> bytes(textureSize * textureSize);

  for (int i = 0; i < textureSize; ++i)
  {
    for (int j = 0; j < textureSize; ++j)
    {
      float t = sqrt(float(i*i) + float(j*j)) / float(textureSize);
      t = Clamp(0.f, 1.f, t);
      t = SmoothStep(0.0f, 1.0f, t);
      u32 val = 255 - (u32)(255.0f * t);

      bytes[i*textureSize+j] = 0x00ffffff + (val<<24);
    }
  }

  CD3D11_SAMPLER_DESC samplerDesc = CD3D11_SAMPLER_DESC(CD3D11_DEFAULT());
  samplerDesc.Filter = D3D11_FILTER_MIN_MAG_LINEAR_MIP_POINT;
  _lineSampler = GRAPHICS.CreateSamplerState(samplerDesc);

  _lineTexture = GRAPHICS.CreateTexture(
    textureSize, textureSize, DXGI_FORMAT_R8G8B8A8_UNORM, bytes.data(), textureSize * sizeof(u32));
}

//------------------------------------------------------------------------------
bool Blob::Update(const UpdateState& state)
{
  UpdateCameraMatrix(state);
  return true;
}

//------------------------------------------------------------------------------
void Blob::UpdateCameraMatrix(const UpdateState& state)
{
  _camera.Update(state);

  if (!_scene.cameras.empty())
  {
    const scene::Camera& cam = *_scene.cameras.back();
    Vector3 pos = cam.mtxLocal.Translation();
    Vector3 target = pos + 1.f * cam.mtxLocal.Backward();
    Vector3 up = cam.mtxLocal.Up();
    Matrix view = Matrix::CreateLookAt(pos, target, up);

    int w, h;
    GRAPHICS.GetBackBufferSize(&w, &h);
    float aspectRatio = (float)w / h;

    Matrix proj = Matrix::CreatePerspectiveFieldOfView(cam.verticalFov, aspectRatio, cam.nearPlane, cam.farPlane);

    Matrix viewProj = view * proj;

    _cbPerFrame.world = Matrix::Identity();
    _cbPerFrame.view = view.Transpose();
    _cbPerFrame.proj = proj.Transpose();
    _cbPerFrame.viewProj = viewProj.Transpose();
    _cbPerFrame.cameraPos = pos;
    _cbPerFrame.cameraLookAt = target;
    _cbPerFrame.cameraUp = up;

  }
  else
  {
    Matrix view = _camera._view;
    Matrix proj = _camera._proj;

    Matrix viewProj = view * proj;

    _cbPerFrame.world = Matrix::Identity();
    _cbPerFrame.view = view.Transpose();
    _cbPerFrame.proj = proj.Transpose();
    _cbPerFrame.viewProj = viewProj.Transpose();
    _cbPerFrame.cameraPos = _camera._pos;
    _cbPerFrame.cameraLookAt = _camera._target;
    _cbPerFrame.cameraUp = _camera._up;
  }
}

//------------------------------------------------------------------------------
bool Blob::Render()
{
  rmt_ScopedCPUSample(Blob_Render);

  static Color black(.1f, .1f, .1f, 0);

  _ctx->SetSwapChain(GRAPHICS.DefaultSwapChain(), black);
  _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::PixelShader, 0);

  _ctx->SetGpuState(_blobState);

  for (const scene::Mesh* mesh : _scene.meshes)
  {
    Matrix mtx = mesh->mtxGlobal;
//     u32 parentId = mesh->parentId;
//     if (parentId != ~0u)
//     {
//       mtx = _scene.baseObjects[parentId]->mtx * mesh->mtx;
//     }
//     else
//     {
//       mtx = mesh->mtx;
//     }
      
    _cbPerFrame.world = mtx.Transpose();
    _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::VertexShader, 0);

    _ctx->SetGpuObjects(mesh->gpuObjects);
    for (const scene::Mesh::MaterialGroup& mg : mesh->materialGroups)
    {
      const scene::Material* mat = _scene.materials[mg.materialId];

      _cbPerFrame.diffuse = mat->color.color;
      _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::PixelShader, 0);

      _ctx->DrawIndexed(mg.indexCount, mg.startIndex, 0);
    }
  }



#if 0

  float lineRadius = 2.0f;
  int w, h;
  GRAPHICS.GetBackBufferSize(&w, &h);

  // Translate radius in pixels to NDC.
  float radius = 2.0f * (float)lineRadius / w;
  float aspect = (float)w / h;

  // ground
  Vector3 p00(-9, 6, 0);
  Vector3 p01(8, 6, 0);
  Vector3 p02(8, -6, 0);
  Vector3 p03(-9, -6, 0);

  // beltline
  Vector3 p04(-11, 8, 3);
  Vector3 p05(11, 8, 3);
  Vector3 p06(11, -8, 3);
  Vector3 p07(-11, -8, 3);

  // turret bottom
  Vector3 p08(-8, 4, 5);
  Vector3 p09(3, 4, 5);
  Vector3 p10(3, -4, 5);
  Vector3 p11(-8, -4, 5);

  // turret peak
  Vector3 p12(-6, 2, 8);
  Vector3 p13(-6, -2, 8);

  // gun
  //   muzzle
  Vector3 p14(10, 0.5, 6);
  Vector3 p15(10, -0.5, 6);
  Vector3 p16(10, -0.5, 7);
  Vector3 p17(10, 0.5, 7);
  //   base
  Vector3 p18(0, 0.5, 6);
  Vector3 p19(0, -0.5, 6);
  Vector3 p20(-3, -0.5, 7);
  Vector3 p21(-3, 0.5, 7);

  // radar
  Vector3 p22(-6, 0, 8);
  Vector3 p23(-6, -1, 8.5);
  Vector3 p24(-5.5, -2, 9);
  Vector3 p25(-5.5, -2, 9.5);
  Vector3 p26(-6, -1, 10);
  Vector3 p27(-6, 1, 10);
  Vector3 p28(-5.5, 2, 9.5);
  Vector3 p29(-5.5, 2, 9);
  Vector3 p30(-6, 1, 8.5);
  Vector3 p31(-6, 0, 8.5);

  // 3 bands, bottom up
  AddLine(p00, p01, radius, aspect);
  AddLine(p01, p02, radius, aspect);
  AddLine(p02, p03, radius, aspect);
  AddLine(p03, p00, radius, aspect);

  AddLine(p04, p05, radius, aspect);
  AddLine(p05, p06, radius, aspect);
  AddLine(p06, p07, radius, aspect);
  AddLine(p07, p04, radius, aspect);

  AddLine(p08, p09, radius, aspect);
  AddLine(p09, p10, radius, aspect);
  AddLine(p10, p11, radius, aspect);
  AddLine(p11, p08, radius, aspect);

  // vertical joins for bottom
  AddLine(p00, p04, radius, aspect);
  AddLine(p01, p05, radius, aspect);
  AddLine(p02, p06, radius, aspect);
  AddLine(p03, p07, radius, aspect);

  AddLine(p08, p04, radius, aspect);
  AddLine(p09, p05, radius, aspect);
  AddLine(p10, p06, radius, aspect);
  AddLine(p11, p07, radius, aspect);

  // turret
  AddLine(p08, p12, radius, aspect);
  AddLine(p13, p12, radius, aspect);
  AddLine(p11, p13, radius, aspect);
  AddLine(p10, p13, radius, aspect);
  AddLine(p09, p12, radius, aspect);

  // gun
  AddLine(p14, p15, radius, aspect);
  AddLine(p15, p16, radius, aspect);
  AddLine(p16, p17, radius, aspect);
  AddLine(p17, p14, radius, aspect);

  AddLine(p18, p19, radius, aspect);
  AddLine(p19, p20, radius, aspect);
  AddLine(p20, p21, radius, aspect);
  AddLine(p21, p18, radius, aspect);

  AddLine(p14, p18, radius, aspect);
  AddLine(p15, p19, radius, aspect);
  AddLine(p16, p20, radius, aspect);
  AddLine(p17, p21, radius, aspect);

  // radar
  AddLine(p23, p24, radius, aspect);
  AddLine(p24, p25, radius, aspect);
  AddLine(p25, p26, radius, aspect);
  AddLine(p26, p27, radius, aspect);
  AddLine(p27, p28, radius, aspect);
  AddLine(p28, p29, radius, aspect);
  AddLine(p29, p30, radius, aspect);
  AddLine(p30, p23, radius, aspect);

  AddLine(p23, p26, radius, aspect);
  AddLine(p27, p30, radius, aspect);

  AddLine(p22, p31, radius, aspect);

  LineElement* data = _ctx->MapWriteDiscard<LineElement>(_lineObjects._vb);
  memcpy(data, _lineElements, _vtxIndex * sizeof(LineElement));
  _ctx->Unmap(_lineObjects._vb);

  _ctx->SetGpuState(_blobState);
  _ctx->SetGpuObjects(_lineObjects);
  _ctx->SetSamplerState(_lineSampler);
  _ctx->SetShaderResource(_lineTexture);
  _ctx->DrawIndexed(_numLineTris * 3, 0, 0);
  _vtxIndex = 0;
  _numLineTris = 0;
#endif
  return true;
}

//------------------------------------------------------------------------------
bool Blob::InitAnimatedParameters()
{
  return true;
}

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Blob::RenderParameterSet()
{
  ImGui::SliderFloat("roughness", &_cbPerFrame.cook.x, 0.f, 2.0f);
  ImGui::SliderFloat("F0", &_cbPerFrame.cook.y, 0.f, 1.f);

  if (ImGui::Button("Reset"))
    Reset();
}
#endif

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Blob::SaveParameterSet()
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
void Blob::Reset()
{
  _camera._pos = Vector3(0.f, 0.f, 0.f);
  _camera._pitch = _camera._yaw = _camera._roll = 0.f;
}

//------------------------------------------------------------------------------
bool Blob::Close()
{
  return true;
}

//------------------------------------------------------------------------------
Effect* Blob::Create(const char* name, u32 id)
{
  return new Blob(name, id);
}

//------------------------------------------------------------------------------
const char* Blob::Name()
{
  return "blob";
}

//------------------------------------------------------------------------------
void Blob::Register()
{
  DEMO_ENGINE.RegisterFactory(Name(), Blob::Create);
}
