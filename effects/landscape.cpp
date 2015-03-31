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

static const Vector3 ZERO3(0,0,0);

int Landscape::Boid::nextId;

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

Vector3 NOISE_SCALE(10.f, 30.f, 10.f);

float NoiseAtPoint(const Vector3& v)
{
  Vector3 size(10 * 1024, 10 * 1024, 10 * 1024);
  return NOISE_SCALE.y * stb_perlin_noise3(256 * v.x / size.x, 0, 256 * v.z / size.z);
}

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

  // create mesh from landscape
  u32 vertexFlags = VF_POS | VF_NORMAL;
  u32 vertexSize = sizeof(PosNormal);
  INIT(_landscapeGpuObjects.CreateDynamicVb(1024*1024*6*vertexSize, vertexSize));

  INIT(_landscapeGpuObjects.LoadShadersFromFile("shaders/out/landscape", "VsLandscape", nullptr, "PsLandscape", vertexFlags));
  INIT(_landscapeState.Create());

  INIT(_edgeGpuObjects.LoadShadersFromFile("shaders/out/landscape", "VsQuad", nullptr, "PsEdgeDetect"));
  INIT(_skyGpuObjects.LoadShadersFromFile("shaders/out/landscape", "VsQuad", nullptr, "PsSky"));

  INIT(_compositeGpuObjects.LoadShadersFromFile("shaders/out/landscape", "VsQuad", nullptr, "PsComposite"));
  INIT(_compositeState.Create());

  MeshLoader loader;
  INIT(loader.Load("gfx/boids.boba"));
  u32 boidsVertexFlags = 0;
  INIT(CreateBuffersFromMesh(loader, "Pyramid", &boidsVertexFlags, &_boidsMesh));
  INIT(_boidsMesh.LoadShadersFromFile("shaders/out/landscape", "VsBoids", nullptr, "PsBoids", boidsVertexFlags));
  InitBoids();

  int w, h;
  INIT(_cbPerFrame.Create());
  GRAPHICS.GetBackBufferSize(&w, &h);

//  _camera._pos = Vector3(0, 200, 500);

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
void Landscape::InitBoids()
{
  for (int i = 0; i < 10; ++i)
  {
    Flock* flock = new Flock();
    vector<Boid>& boids = flock->boids;

    float s = 200.f;
    Vector3 center(randf(-s, s), 0, randf(-s, s));
    // Create a waypoint for the flock
    float angle = randf(-XM_PI, XM_PI);
    float dist = randf(100.f, 200.f);
    flock->nextWaypoint = center + Vector3(dist * cos(angle), 0, dist * sin(angle));
    flock->wanderAngle = angle;

    // Create the boids
    for (int j = 0; j < 50; ++j)
    {
      Boid boid(flock);
      boid.pos = center + Vector3(randf(-20.f, 20.f), 0, randf(-20.f, 20.f));
      boid.pos.y = 20 + NoiseAtPoint(boid.pos);
      boid.force = 10 * Vector3(randf(-20.f, 20.f), 0, randf(-20.f, 20.f));
      boids.push_back(boid);
    }
    _flocks.push_back(flock);
  }

}

//------------------------------------------------------------------------------
Vector3 Landscape::BoidSeparation(const Boid& boid)
{
  // return a force away from any close boids
  Vector3 avg(0, 0, 0);

  float cnt = 0.f;
  for (const Boid& b : boid.flock->boids)
  {
    if (b.id == boid.id)
      continue;

    float dist = Vector3::Distance(boid.pos, b.pos);
    if (dist > _settings.boids.separation_distance)
      continue;

    Vector3 f = Normalize(boid.pos - b.pos);
    avg += 1.f / dist * f;
    cnt += 1.f;
  }

  if (cnt == 0.f)
    return avg;

  avg /= cnt;

  // Reynolds uses: steering = desired - current
  Vector3 desired = Normalize(avg) * _settings.boids.max_speed;
  Vector3 steering = desired - boid.vel;
  return steering;
}

//------------------------------------------------------------------------------
Vector3 Landscape::BoidCohesion(const Boid& boid)
{
  // Return a force towards the average boid position
  Vector3 avg(0, 0, 0);
  
  float cnt = 0.f;
  for (const Boid& b : boid.flock->boids)
  {
    if (b.id == boid.id)
      continue;

    float dist = Vector3::Distance(boid.pos, b.pos);
    if (dist > _settings.boids.cohesion_distance)
      continue;

    avg += b.pos;
    cnt += 1.f;
  }

  if (cnt == 0.f)
    return avg;

  avg /= cnt;
  return Seek(boid, avg);
}

//------------------------------------------------------------------------------
Vector3 Landscape::BoidAlignment(const Boid& boid)
{
  // return a force to align the boids velocity with the average velocity
  Vector3 avg(0, 0, 0);

  float cnt = 0.f;
  for (const Boid& b : boid.flock->boids)
  {
    if (b.id == boid.id)
      continue;

    float dist = Vector3::Distance(boid.pos, b.pos);
    if (dist > _settings.boids.cohesion_distance)
      continue;

    avg += b.vel;
    cnt += 1.f;
  }

  if (cnt == 0.f)
    return avg;

  avg /= cnt;
  Vector3 desired = Normalize(avg) * _settings.boids.max_speed;
  Vector3 steering = desired - boid.vel;
  return steering;
}

//------------------------------------------------------------------------------
Vector3 Landscape::LandscapeFollow(const Boid& boid)
{
  // return a force to keep the boid above the ground

  Vector3 probe = boid.pos + 0.001f * boid.vel;
  Vector3 d = probe;
  d.y = 20 + NoiseAtPoint(probe);

  // if the probe is above the terrain height, move towards the average
  if (probe.y > d.y)
    d.y = 0.5f * (probe.y + d.y);

  Vector3 desiredVel = Normalize(d - probe) * _settings.boids.max_speed;
  return desiredVel - boid.vel;
}

//------------------------------------------------------------------------------
Vector3 Landscape::Seek(const Boid& boid, const Vector3& target)
{
  Vector3 desiredVel = Normalize(target - boid.pos) * _settings.boids.max_speed;
  return desiredVel - boid.vel;
}

//------------------------------------------------------------------------------
Vector3 Landscape::ClampVector(const Vector3& force, float maxLength)
{
  float len = force.Length();
  if (len <= maxLength)
    return force;

  return maxLength * Normalize(force);
}

//------------------------------------------------------------------------------
void Landscape::UpdateBoids(const UpdateState& state)
{
  rmt_ScopedCPUSample(Boids_Update);

  float dt = 1.0f / state.frequency;

  for (Flock* flock : _flocks)
  {
    // check if the flock has reached its waypoint
    float closestDist = FLT_MAX;
    for (Boid& b : flock->boids)
    {
      closestDist = min(closestDist, Vector3::Distance(b.pos, flock->nextWaypoint));
      if (closestDist < _settings.boids.waypoint_radius)
      {
        // new waypoint
        float angle = flock->wanderAngle + randf(-XM_PI/4, XM_PI/4);
        float dist = randf(100.f, 200.f);
        flock->nextWaypoint += Vector3(dist * cos(angle), 0, dist * sin(angle));
        flock->wanderAngle = angle;
        break;
      }
    }

    for (Boid& b : flock->boids)
    {
      b.force =
        _settings.boids.separation_scale * BoidSeparation(b) +
        _settings.boids.cohesion_scale * BoidCohesion(b) +
        _settings.boids.alignment_scale * BoidAlignment(b) +
        _settings.boids.follow_scale * LandscapeFollow(b) +
        _settings.boids.wander_scale * Seek(b, flock->nextWaypoint);

      b.force = ClampVector(b.force, _settings.boids.max_force);

      // f = m * a
      b.acc = b.force;

      // v += dt * a;
      b.vel = ClampVector(b.vel + dt * b.acc, _settings.boids.max_speed);

      // p += dt * v;
      b.pos += dt * b.vel;

      b.force = ZERO3;
      b.acc = ZERO3;

      //b.force = 1.f * -b.vel;
    }
  }
}

//------------------------------------------------------------------------------
bool Landscape::Update(const UpdateState& state)
{
  UpdateBoids(state);
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
int Round(float v)
{
  return v < 0 ? (int)floor(v) : (int)ceil(v);
}

//------------------------------------------------------------------------------
void Landscape::RasterizeLandscape(float* buf)
{
  rmt_ScopedCPUSample(Landscape_Rasterize);

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
  int maxZ = Round(maxValues.z / 10);
  int minZ = Round(minValues.z / 10);
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

    int sy = Round(v0.z);
    int ey = Round(v1.z);
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
      int intX = Round(x);
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

  _cbPerFrame.world = Matrix::Identity();

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


  if (_drawLandscape)
  {
    float* buf = _ctx->MapWriteDiscard<float>(_landscapeGpuObjects._vb);
    RasterizeLandscape(buf);
    _ctx->Unmap(_landscapeGpuObjects._vb);

    _ctx->SetGpuObjects(_landscapeGpuObjects);
    _ctx->SetGpuState(_landscapeState);
    _ctx->Draw(_numVerts, 0);
  }
  else
  {
    _ctx->SetGpuState(_landscapeState);
  }

  _ctx->SetGpuObjects(_boidsMesh);

  for (const Flock* flock : _flocks)
  {
    for (const Boid& boid: flock->boids)
    {
      Matrix mtxRot = Matrix::Identity();
      Vector3 dir = Normalize(boid.vel);
      Vector3 up(0,1,0);
      Vector3 right = Cross(up, dir);
      up = Cross(dir, right);
      mtxRot.Backward(dir);
      mtxRot.Up(up);
      mtxRot.Right(right);
      mtxRot.Translation(boid.pos);
      _cbPerFrame.world = mtxRot.Transpose();
      _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::VertexShader, 0);
      _ctx->DrawIndexed(_boidsMesh._numIndices, 0, 0);
    }
  }

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
  ImGui::Checkbox("Render landscape", &_drawLandscape);
  ImGui::InputInt("NumVerts", (int*)&_numVerts);
  ImGui::SliderFloat("Separation", &_settings.boids.separation_scale, 0.1f, 10.f);
  ImGui::SliderFloat("Cohension", &_settings.boids.cohesion_scale, 0.1f, 10.f);
  ImGui::SliderFloat("Alignment", &_settings.boids.alignment_scale, 0.1f, 10.f);
  ImGui::SliderFloat("Wander", &_settings.boids.wander_scale, 1.f, 25.f);
  ImGui::SliderFloat("Follow", &_settings.boids.follow_scale, 1.f, 25.f);
  ImGui::SliderFloat("MaxSpeed", &_settings.boids.max_speed, 5.f, 100.f);
  ImGui::SliderFloat("MaxForce", &_settings.boids.max_force, 5.f, 100.f);
  ImGui::SliderFloat("SepDist", &_settings.boids.separation_distance, 1.f, 100.f);
  ImGui::SliderFloat("CohDist", &_settings.boids.cohesion_distance, 1.f, 100.f);

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
