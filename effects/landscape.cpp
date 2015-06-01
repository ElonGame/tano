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
#include "../debug_api.hpp"
#include "../tano_math.hpp"
#include "../perlin2d.hpp"

using namespace tano;
using namespace bristol;

static const Vector3 ZERO3(0,0,0);

static float GRID_SIZE = 10;

Perlin2D perlin;

float NoiseAtPoint(const Vector3& v);

//------------------------------------------------------------------------------
Landscape::Flock::Flock(int numBoids)
{
  boids.Init(numBoids);
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

  perlin.Init();
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
  _freeflyCamera._pitch = _settings.camera.pitch;
  _freeflyCamera._yaw = _settings.camera.yaw;
  _freeflyCamera._roll = _settings.camera.roll;
  _freeflyCamera._pos = _settings.camera.pos;

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

  Reset();
  //InitBoids();

  int w, h;
  INIT(_cbPerFrame.Create());
  GRAPHICS.GetBackBufferSize(&w, &h);

//  _camera._pos = Vector3(0, 200, 500);

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
void Landscape::InitBoids()
{
  SeqDelete(&_flocks);
  SAFE_DELETE(_behaviorSeek);
  SAFE_DELETE(_behaviorSeparataion);
  SAFE_DELETE(_behaviorCohesion);
  SAFE_DELETE(_behaviorAlignment);
  SAFE_DELETE(_landscapeFollow);

  const BoidSettings& b = _settings.boids;
  _behaviorSeek = new BehaviorSeek(b.max_force, b.max_speed);
  _behaviorSeparataion = new BehaviorSeparataion(b.max_force, b.max_speed, b.separation_distance);
  _behaviorCohesion = new BehaviorCohesion(b.max_force, b.max_speed, b.cohesion_distance);
  _behaviorAlignment = new BehaviorAlignment(b.max_force, b.max_speed, b.cohesion_distance);
  _landscapeFollow = new BehaviorLandscapeFollow(b.max_force, b.max_speed);

  for (int i = 0; i < _settings.boids.num_flocks; ++i)
  {
    Flock* flock = new Flock(_settings.boids.boids_per_flock);
    flock->boids._maxSpeed = b.max_speed;

    float sum = b.wander_scale + b.separation_scale + b.cohesion_scale + b.alignment_scale;
    flock->boids.AddKinematics(_behaviorSeek, _settings.boids.wander_scale / sum);
    flock->boids.AddKinematics(_behaviorSeparataion, _settings.boids.separation_scale / sum);
    flock->boids.AddKinematics(_behaviorCohesion, _settings.boids.cohesion_scale / sum);
    flock->boids.AddKinematics(_behaviorAlignment, _settings.boids.alignment_scale / sum);
    flock->boids.AddKinematics(_landscapeFollow, _settings.boids.follow_scale / sum);

    float s = 200.f;
    Vector3 center(randf(-s, s), 0, randf(-s, s));

    // Create a waypoint for the flock
    float angle = randf(-XM_PI, XM_PI);
    float dist = randf(100.f, 200.f);
    flock->nextWaypoint = center + Vector3(dist * cos(angle), 0, dist * sin(angle));
    flock->nextWaypoint.y = 20 + NoiseAtPoint(flock->nextWaypoint);
    flock->wanderAngle = angle;

    _behaviorSeek->target = flock->nextWaypoint;

    // Init the boids
    for (DynParticles::Body& b : flock->boids)
    {
      b.pos = center + Vector3(randf(-20.f, 20.f), 0, randf(-20.f, 20.f));
      b.pos.y = 20 + NoiseAtPoint(b.pos);
      b.force = 10 * Vector3(randf(-20.f, 20.f), 0, randf(-20.f, 20.f));
    }
    _flocks.push_back(flock);
  }
}

//------------------------------------------------------------------------------
void BehaviorLandscapeFollow::Update(DynParticles::Body* bodies, int numBodies, float weight, const UpdateState& state)
{
  for (DynParticles::Body* b = bodies; b != bodies + numBodies; ++b)
  {
    // return a force to keep the boid above the ground

    Vector3 probe = b->pos + Normalize(b->vel) * maxSpeed;
    Vector3 d = probe;
    d.y = 20 + NoiseAtPoint(probe);

    // if the probe is above the terrain height, move towards the average
    if (probe.y > d.y)
      d = 0.5f * (probe + d);

    Vector3 desiredVel = Normalize(d - probe) * maxSpeed;
    b->force += ClampVector(desiredVel - b->vel, maxForce);
  }
}
#if 0

//------------------------------------------------------------------------------
Vector3 Landscape::LandscapeFollow(const Boid& boid)
{
  // return a force to keep the boid above the ground

  Vector3 tmp = boid.vel;
  tmp.Normalize();
  Vector3 probe = boid.pos + tmp * _settings.boids.max_speed;
  Vector3 d = probe;
  d.y = 20 + NoiseAtPoint(probe);

  Matrix view = _camera._view;
  Matrix proj = _camera._proj;
  Matrix viewProj = view * proj;

  DEBUG_API.SetTransform(Matrix::Identity(), viewProj);
  DEBUG_API.AddDebugLine(boid.pos, d, Color(1,1,1));

  // if the probe is above the terrain height, move towards the average
  if (probe.y > d.y)
    d = 0.5f * (probe + d);

  tmp = d - probe;
  tmp.Normalize();
  Vector3 desiredVel = tmp * _settings.boids.max_speed;
  return desiredVel - boid.vel;
}

#endif
//------------------------------------------------------------------------------
void Landscape::UpdateBoids(const UpdateState& state)
{
  rmt_ScopedCPUSample(Boids_Update);

  float dt = 1.0f / state.frequency;

  for (Flock* flock : _flocks)
  {
    // check if the flock has reached its waypoint
    float closestDist = FLT_MAX;
    Vector3 center(0,0,0);
    for (DynParticles::Body& b : flock->boids)
    {
      if (Vector3::Distance(b.pos, flock->nextWaypoint) < _settings.boids.waypoint_radius)
      {
        float angle = flock->wanderAngle + randf(-XM_PI / 2, XM_PI / 2);
        float dist = randf(300.f, 400.f);
        flock->nextWaypoint += Vector3(dist * cos(angle), 0, dist * sin(angle));
        flock->nextWaypoint.y = 20 + NoiseAtPoint(flock->nextWaypoint);

        flock->wanderAngle = angle;
        break;
      }
    }

    _behaviorSeek->target = flock->nextWaypoint;
    flock->boids.Update(state);
  }
}

//------------------------------------------------------------------------------
bool Landscape::Update(const UpdateState& state)
{
  UpdateBoids(state);
  UpdateCameraMatrix(state);
  return true;
}

//------------------------------------------------------------------------------
void Landscape::UpdateCameraMatrix(const UpdateState& state)
{
  if (!_flocks.empty())
  {
    const IoState& state = TANO.GetIoState();
    if (state.keysPressed['1'])
      _followFlock = (_followFlock + 1) % _flocks.size();

    if (state.keysPressed['2'])
      _followFlock = (_followFlock - 1) % _flocks.size();

    if (_followFlock != -1 && _followFlock < _flocks.size())
    {
      _followCamera.SetFollowTarget(_flocks[_followFlock]->boids._center);
    }
  }

  if (_useFreeFlyCamera || _flocks.empty())
    _curCamera = &_freeflyCamera;
  else
    _curCamera = &_followCamera;

  _curCamera->Update(state);
  Matrix view = _curCamera->_view;
  Matrix proj = _curCamera->_proj;
  Matrix viewProj = view * proj;

  // compute size of frustum
  float farW = _curCamera->_farPlane * tan(_curCamera->_fov);
  Vector3 v0(-farW, 0, _curCamera->_farPlane);
  Vector3 v1(+farW, 0, _curCamera->_farPlane);

  float nearW = _curCamera->_nearPlane * tan(_curCamera->_fov);
  Vector3 v2(-nearW, 0, _curCamera->_nearPlane);
  Vector3 v3(+nearW, 0, _curCamera->_nearPlane);

  v0 = Vector3::Transform(v0 + _curCamera->_pos, _curCamera->_mtx);
  v1 = Vector3::Transform(v1 + _curCamera->_pos, _curCamera->_mtx);
  v2 = Vector3::Transform(v2 + _curCamera->_pos, _curCamera->_mtx);
  v3 = Vector3::Transform(v3 + _curCamera->_pos, _curCamera->_mtx);

  _cbPerFrame.world = Matrix::Identity();
  _cbPerFrame.view = view.Transpose();
  _cbPerFrame.proj = proj.Transpose();
  _cbPerFrame.viewProj = viewProj.Transpose();
  _cbPerFrame.cameraPos = _curCamera->_pos;
  _cbPerFrame.cameraLookAt = _curCamera->_target;
  _cbPerFrame.cameraUp = _curCamera->_up;

  DEBUG_API.SetTransform(Matrix::Identity(), viewProj);
}

//------------------------------------------------------------------------------
inline int Expand(float v)
{
  return v < 0 ? (int)floorf(v) : (int)ceilf(v);
}

inline int RoundUp(float v)
{
  return v < 0 ? (int)floorf(v) : (int)ceilf(v);
}

inline int RoundDown(float v)
{
  return v < 0 ? (int)ceilf(v) : (int)floorf(v);
}

//------------------------------------------------------------------------------
void Vector3ToFloat(float* buf, const V3& v)
{
  buf[0] = v.x;
  buf[1] = v.y;
  buf[2] = v.z;
}

//------------------------------------------------------------------------------
float NoiseAtPoint(const Vector3& v)
{
  static const Vector3 NOISE_SCALE(10.f, 30.f, 10.f);
  static const Vector3 size(10 * 1024, 10 * 1024, 10 * 1024);
  return NOISE_SCALE.y * perlin.Value(256 * v.x / size.x, 256 * v.z / size.z);
}

//------------------------------------------------------------------------------
void Rasterize(
  const Vector3& scale,
  int startZ, const vector<pair<float, float>>& spans,
  float* verts, u32* numVerts)
{
  V3 v0, v1, v2, v3;
  V3 n0, n1;
  V3 size(10 * 1024, 10 * 1024, 10 * 1024);

  int triIdx = 0;
  for (int idx = 0; idx < (int)spans.size(); ++idx)
  {
    int i = startZ + idx;
    int a = floorf(spans[idx].first / GRID_SIZE);
    int b = ceilf(spans[idx].second / GRID_SIZE);
    for (int j = a; j <= b; ++j)
    {
      // 1--2
      // |  |
      // 0--3

      float xx0 = (float)(j + 0) * scale.x;
      float xx1 = (float)(j + 1) * scale.x;
      float zz0 = (float)(i + 0) * scale.z;
      float zz1 = (float)(i + 1) * scale.z;

      v0.x = xx0; 
      v0.z = zz0;
      v1.x = xx0; 
      v1.z = zz1;
      v2.x = xx1; 
      v2.z = zz1;
      v3.x = xx1; 
      v3.z = zz0;

      v0.y = scale.y * perlin.Value(256 * xx0 / size.x, 256 * zz0 / size.z);
      v1.y = scale.y * perlin.Value(256 * xx0 / size.x, 256 * zz1 / size.z);
      v2.y = scale.y * perlin.Value(256 * xx1 / size.x, 256 * zz1 / size.z);
      v3.y = scale.y * perlin.Value(256 * xx1 / size.x, 256 * zz0 / size.z);

      V3 e1, e2;
      e1 = v2 - v1;
      e2 = v0 - v1;
      n0 = Cross(e1, e2);
      n0 = Normalize(n0);

      e1 = v0 - v3;
      e2 = v2 - v3;
      n1 = Cross(e1, e2);
      n1 = Normalize(n1);

      // 0, 1, 3
      Vector3ToFloat(&verts[triIdx * 18 + 0], v0);
      Vector3ToFloat(&verts[triIdx * 18 + 3], n0);
      Vector3ToFloat(&verts[triIdx * 18 + 6], v1);
      Vector3ToFloat(&verts[triIdx * 18 + 9], n0);
      Vector3ToFloat(&verts[triIdx * 18 + 12], v3);
      Vector3ToFloat(&verts[triIdx * 18 + 15], n0);
      ++triIdx;

      Vector3ToFloat(&verts[triIdx * 18 + 0], v3);
      Vector3ToFloat(&verts[triIdx * 18 + 3], n1);
      Vector3ToFloat(&verts[triIdx * 18 + 6], v1);
      Vector3ToFloat(&verts[triIdx * 18 + 9], n1);
      Vector3ToFloat(&verts[triIdx * 18 + 12], v2);
      Vector3ToFloat(&verts[triIdx * 18 + 15], n1);
      ++triIdx;
    }
  }

  *numVerts = triIdx * 3;
}

//------------------------------------------------------------------------------
void Landscape::RasterizeLandscape(float* buf)
{
  rmt_ScopedCPUSample(Landscape_Rasterize);

  // Create a large rect around the camera, and clip it using the camera planes
  Plane planes[6];
  ExtractPlanes(_curCamera->_view * _curCamera->_proj, true, planes);

  float ofs = 2 * _curCamera->_farPlane;
  Vector3 c = _curCamera->_pos;
  //c.y = 0;
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
      return;
    memcpy(buf0, buf1, numVerts * sizeof(Vector3));
  }

  struct Edge
  {
    Vector3 a, b;
  };

  // Create edges, and sort them based on distance to camera (furthest point first)
  //vector<Edge> edges;
  //Vector3 camPos = _curCamera->_pos;
  //for (int i = 0; i < numVerts; ++i)
  //{
  //  Vector3 a = buf0[i];
  //  Vector3 b = buf0[(i+1) % numVerts];

  //  if (a.z == b.z)
  //    continue;

  //  if (Vector3::DistanceSquared(camPos, a) > Vector3::DistanceSquared(camPos, b))
  //    edges.push_back({a, b});
  //  else
  //    edges.push_back({b, a});
  //}

  //Vector3 minValues = buf0[0];
  //Vector3 maxValues = buf0[0];

  Vector3 camPos = _curCamera->_pos;
  float maxDist = Vector3::Distance(buf0[0], camPos);
  float minDist = Vector3::Distance(buf0[0], camPos);

  for (int i = 1; i < numVerts; ++i)
  {
    minDist = min(minDist, Vector3::Distance(buf0[i], camPos));
    maxDist = max(maxDist, Vector3::Distance(buf0[i], camPos));
  }

  int maxZ = ceilf(maxDist / GRID_SIZE);
  int minZ = floorf(minDist / GRID_SIZE);
  //int sizeZ = Expand((maxValues.z - minValues.z)) / GRID_SIZE;
  int sizeZ = maxZ - minZ;
  vector<pair<float, float>> spans(sizeZ);

  for (int i = 0; i < sizeZ; ++i)
  {
    spans[i].first = FLT_MAX;
    spans[i].second = -FLT_MAX;
  }

  // scan convert all the edges, and save min/max values
  for (int i = 0; i < numVerts; ++i)
  {
    Vector3 vv0 = buf0[i];
    Vector3 vv1 = buf0[(i+1) % numVerts];

    // v0.z > v1.z
    Vector3& v0 = vv0.z > vv1.z ? vv0 : vv1;
    Vector3& v1 = vv0.z > vv1.z ? vv1 : vv0;

    // calc number of grids this edge spans
    int numGrids = ceilf((v0.z - v1.z) / GRID_SIZE);
    if (numGrids == 0)
      continue;

    float dz = v0.z - v1.z;
    float dx = v0.x - v1.x;
    float dxdz = dx / numGrids;
    float dzdc = dz / numGrids;

    float x = v0.x;
    float z = v0.z;

    for (int i = 0; i < numGrids; ++i)
    {
      //int idx = (z - minValues.z) / GRID_SIZE;
      int idx = ((z - camPos.z) - minDist) / GRID_SIZE;
      spans[idx].first = min(spans[idx].first, x);
      spans[idx].second = max(spans[idx].second, x);
      x -= dxdz;
      z -= dzdc;
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

  if (_renderLandscape)
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

  if (_renderBoids)
  {
    _ctx->SetGpuObjects(_boidsMesh);

    Matrix view = _curCamera->_view;
    Matrix proj = _curCamera->_proj;
    Matrix viewProj = view * proj;

    for (const Flock* flock : _flocks)
    {
      DEBUG_API.AddDebugLine(flock->boids._center, flock->nextWaypoint, Color(1, 1, 1));
      DEBUG_API.AddDebugSphere(flock->nextWaypoint, 10, Color(1, 1, 1));

      for (const DynParticles::Body& b : flock->boids)
      {
        Matrix mtxRot = Matrix::Identity();
        Vector3 velN = b.vel;
        velN.Normalize();
        Vector3 dir = b.vel.LengthSquared() ? velN : Vector3(0, 0, 1);
        Vector3 up(0, 1, 0);
        Vector3 right = Cross(up, dir);
        up = Cross(dir, right);
        mtxRot.Backward(dir);
        mtxRot.Up(up);
        mtxRot.Right(right);
        mtxRot.Translation(b.pos);
        _cbPerFrame.world = mtxRot.Transpose();
        _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::VertexShader, 0);
        _ctx->DrawIndexed(_boidsMesh._numIndices, 0, 0);

        DEBUG_API.AddDebugLine(b.pos, b.pos + b.vel, Color(1, 0, 0));
        DEBUG_API.AddDebugLine(b.pos, b.pos + 10 * up, Color(0, 1, 0));
        DEBUG_API.AddDebugLine(b.pos, b.pos + 10 * right, Color(0, 0, 1));
      }
    }

  }

  // outline. wtf is this thing? :)
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
  auto UpdateWeight = [this](ParticleKinematics* k, float w) {
    for (Flock* f : _flocks)
    {
      f->boids.UpdateWeight(k, w);
    }
  };

  ImGui::Checkbox("Render landscape", &_renderLandscape);
  ImGui::Checkbox("Render boids", &_renderBoids);
  ImGui::InputInt("NumVerts", (int*)&_numVerts);
  ImGui::InputInt("NumFlocks", &_settings.boids.num_flocks);
  ImGui::InputInt("BoidsPerFlock", &_settings.boids.boids_per_flock);
  bool newWeights = false;
  newWeights |= ImGui::SliderFloat("Separation", &_settings.boids.separation_scale, 0, 100);
  newWeights |= ImGui::SliderFloat("Cohension", &_settings.boids.cohesion_scale, 0, 100);
  newWeights |= ImGui::SliderFloat("Alignment", &_settings.boids.alignment_scale, 0, 100);
  newWeights |= ImGui::SliderFloat("Wander", &_settings.boids.wander_scale, 0, 100);
  newWeights |= ImGui::SliderFloat("Follow", &_settings.boids.follow_scale, 0, 100);

  if (newWeights)
  {
    BoidSettings& b = _settings.boids;
    float sum = b.wander_scale + b.separation_scale + b.cohesion_scale + b.alignment_scale + b.follow_scale;
    UpdateWeight(_behaviorSeparataion, _settings.boids.separation_scale / sum);
    UpdateWeight(_behaviorCohesion, _settings.boids.cohesion_scale / sum);
    UpdateWeight(_behaviorAlignment, _settings.boids.alignment_scale / sum);
    UpdateWeight(_behaviorSeek, _settings.boids.wander_scale / sum);
    UpdateWeight(_landscapeFollow, _settings.boids.follow_scale / sum);
  }

  ImGui::SliderFloat("MaxSpeed", &_settings.boids.max_speed, 10.f, 1000.f);
  ImGui::SliderFloat("MaxForce", &_settings.boids.max_force, 10.f, 1000.f);
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
  _settings.camera.pos = _freeflyCamera._pos;
  _settings.camera.yaw = _freeflyCamera._yaw;
  _settings.camera.pitch = _freeflyCamera._pitch;
  _settings.camera.roll = _freeflyCamera._roll;
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
  _freeflyCamera._pos = Vector3(0.f, 10.f, 0.f);
  _freeflyCamera._pitch = 0.f;
  _freeflyCamera._yaw = 0.f;
  _freeflyCamera._roll = 0.f;
  _freeflyCamera._yaw = XM_PI;

  InitBoids();
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
