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
#include "../fullscreen_effect.hpp"
#include "../debug_api.hpp"
#include "../tano_math.hpp"
#include "../scheduler.hpp"
#include "../arena_allocator.hpp"
#include "../perlin2d.hpp"
#include "../stop_watch.hpp"
#include "../blackboard.hpp"
#include "../smooth_driver.hpp"

using namespace tano;
using namespace tano::scheduler;
using namespace bristol;
using namespace DirectX;

static const Vector3 ZERO3(0, 0, 0);
static const float GRID_SIZE = 5;
static const float NOISE_HEIGHT = 50;
static const float NOISE_SCALE_X = 0.01f;
static const float NOISE_SCALE_Z = 0.01f;

static const float SPLINE_RADIUS = 500;

struct FlockTiming
{
  float time;
  int idx;
};

static const float FLOCK_FADE = 0.5f;

vector<FlockTiming> FLOCK_TIMING = { 
  { 0.0f, 5 },
  { 5.0f, 2 },
  { 10.0f, 6 },
  { 15.0f, 8 },
  { 20.0f, 3 },
  { 25.0f, 9 },
  { 30.0f, 1 },
  { 1000, 0 },
};

#define DEBUG_DRAW_PATH 0
#define PROFILE_UPDATES 0

int Landscape::Chunk::nextId = 1;

//------------------------------------------------------------------------------
struct BehaviorGravity : public ParticleKinematics
{
  BehaviorGravity(float maxForce, float maxSpeed) : ParticleKinematics(maxForce, maxSpeed) {}
  virtual void Update(const ParticleKinematics::UpdateParams& params)
  {
    //float gravity = BLACKBOARD.GetFloatVar("landscape.gravity");
    //V3* force = bodies->force;
    //int numBodies = bodies->numBodies;

    //XMVECTOR g = XMVectorReplicate(gravity);

    //for (int i = 0; i < numBodies; ++i)
    //{
    //  force[i] = XMVectorAdd(
    //    force[i],
    //    XMVectorAdd(force[i], g));
    //}
  }
};

//------------------------------------------------------------------------------
float NoiseAtPoint(const V3& v)
{
  return NOISE_HEIGHT * Perlin2D::Value(NOISE_SCALE_X * v.x, NOISE_SCALE_Z * v.z);
}

//------------------------------------------------------------------------------
Landscape::Flock::Flock(const BoidSettings& settings)
{
  boids.Init(settings.boids_per_flock);
  seek = new BehaviorSeek(settings.max_force, settings.max_speed);
}

//------------------------------------------------------------------------------
Landscape::Flock::~Flock()
{
  SAFE_DELETE(seek);
}

//------------------------------------------------------------------------------
Landscape::Landscape(const string& name, const string& config, u32 id) : BaseEffect(name, config, id)
{
}

//------------------------------------------------------------------------------
Landscape::~Landscape()
{
}

//------------------------------------------------------------------------------
bool Landscape::OnConfigChanged(const vector<char>& buf)
{
  return ParseLandscapeSettings(InputBuffer(buf), &_settings);
}

//------------------------------------------------------------------------------
bool Landscape::Init()
{
#if PROFILE_UPDATES
  StopWatch s;
  s.Start(); 
  Reset();

  FixedUpdateState state;
  state.delta = 1.0f / 100.f;
  for (int i = 0; i < 10000; ++i)
  {
    UpdateBoids(state);
  }

  double elapsed = s.Stop();
  LOG_INFO("Elapsed: ", elapsed);
  return false;
#endif

  BEGIN_INIT_SEQUENCE();

  _freeflyCamera.FromProtocol(_settings.camera);

  {
    // Landscape
    u32 vertexFlags = VF_POS | VF_NORMAL;
    u32 vertexSize = sizeof(PosNormal);
    INIT_FATAL(_landscapeGpuObjects.CreateDynamicVb(1024 * 1024 * 6 * vertexSize, vertexSize));

    u32 maxQuads = 1024 * 1024;
    vector<u32> indices = GenerateQuadIndices(maxQuads);
    INIT_FATAL(_landscapeGpuObjects.CreateIndexBuffer(
        (u32)indices.size() * sizeof(u32), DXGI_FORMAT_R32_UINT, GenerateQuadIndices(maxQuads).data()));

    INIT_FATAL(
        _landscapeGpuObjects.LoadVertexShader("shaders/out/landscape.landscape", "VsLandscape", vertexFlags));
    INIT_FATAL(_landscapeGpuObjects.LoadGeometryShader("shaders/out/landscape.landscape", "GsLandscape"));
    INIT_FATAL(_landscapeGpuObjects.LoadPixelShader("shaders/out/landscape.landscape", "PsLandscape"));

    // Blend desc that doesn't write to the emissive channel
    CD3D11_BLEND_DESC blendDescAlphaNoEmissive = blendDescBlendSrcAlpha;
    blendDescAlphaNoEmissive.IndependentBlendEnable = TRUE;
    blendDescAlphaNoEmissive.RenderTarget[1].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALPHA;

    CD3D11_BLEND_DESC blendDescNoEmissive = CD3D11_BLEND_DESC(CD3D11_DEFAULT());
    blendDescNoEmissive.IndependentBlendEnable = TRUE;
    blendDescNoEmissive.RenderTarget[1].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALPHA;

    INIT_FATAL(_landscapeState.Create(nullptr, &blendDescAlphaNoEmissive, &rasterizeDescCullNone));
    INIT_FATAL(_landscapeLowerState.Create(nullptr, &blendDescNoEmissive, &rasterizeDescCullNone));
  }

  // clang-format off
  INIT_FATAL(_skyBundle.Create(BundleOptions()
    .DepthStencilDesc(depthDescDepthDisabled)
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/landscape.sky", "PsSky")));

  INIT_FATAL(_compositeBundle.Create(BundleOptions()
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/landscape.composite", "PsComposite")));

  INIT_FATAL(_lensFlareBundle.Create(BundleOptions()
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/landscape.lensflare", "PsLensFlare")));

  CD3D11_BLEND_DESC particleBlendDesc(blendDescBlendOneOne);
  particleBlendDesc.RenderTarget[1].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALPHA;

  INIT(_particleBundle.Create(BundleOptions()
    .DynamicVb(1024 * 1024 * 6, sizeof(Vector3))
    .VertexShader("shaders/out/landscape.particle", "VsParticle")
    .GeometryShader("shaders/out/landscape.particle", "GsParticle")
    .PixelShader("shaders/out/landscape.particle", "PsParticle")
    .InputElement(CD3D11_INPUT_ELEMENT_DESC("POSITION", DXGI_FORMAT_R32G32B32_FLOAT))
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_POINTLIST)
    .DepthStencilDesc(depthDescDepthWriteDisabled)
    .BlendDesc(particleBlendDesc)
    .RasterizerDesc(rasterizeDescCullNone)));

  INIT(_boidsBundle.Create(BundleOptions()
    .DynamicVb(1024 * 1024 * 6, sizeof(Vector3))
    .VertexShader("shaders/out/landscape.particle", "VsParticle")
    .GeometryShader("shaders/out/landscape.particle", "GsParticle")
    .PixelShader("shaders/out/landscape.particle", "PsParticle")
    .InputElement(CD3D11_INPUT_ELEMENT_DESC("POSITION", DXGI_FORMAT_R32G32B32_FLOAT))
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_POINTLIST)
    .DepthStencilDesc(depthDescDepthWriteDisabled)
    .BlendDesc(particleBlendDesc)
    .RasterizerDesc(rasterizeDescCullNone)));
  // clang-format on

  INIT_FATAL(_cbLensFlare.Create());
  INIT_FATAL(_cbComposite.Create());
  INIT_FATAL(_cbSky.Create());
  INIT_FATAL(_cbLandscape.Create());
  INIT_FATAL(_cbParticle.Create());

  // Particles
  INIT_RESOURCE_FATAL(_particleTexture, RESOURCE_MANAGER.LoadTexture(_settings.particle_texture.c_str()));

  Reset();

  const BoidSettings& b = _settings.boids;
  _followCamera.AddKinematic(new BehaviorLandscapeFollow(b.max_force, b.max_speed));
  _followCamera.AddKinematic(new BehaviorGravity(b.max_force, b.max_speed));

  _flockCamera.flock = _flocks[0];

  {
    FixedUpdateState state;
    state.delta = 1.0f / 100;
    for (int i = 0; i < 250; ++i)
    {
      UpdateBoids(state);
    }
  }

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
void Landscape::InitBoids()
{
  for (Flock* flock : _flocks)
    delete flock;
  _flocks.Clear();
  SAFE_DELETE(_behaviorSeparataion);
  SAFE_DELETE(_behaviorCohesion);
  SAFE_DELETE(_behaviorAlignment);
  SAFE_DELETE(_landscapeFollow);

  const BoidSettings& b = _settings.boids;
  _behaviorSeparataion = new BehaviorSeparataion(b.max_force, b.max_speed, b.separation_distance);
  _behaviorCohesion = new BehaviorCohesion(b.max_force, b.max_speed, b.cohesion_distance);
  _behaviorAlignment = new BehaviorAlignment(b.max_force, b.max_speed, b.cohesion_distance);
  _landscapeFollow = new BehaviorLandscapeFollow(b.max_force, b.max_speed);

  vector<V3> controlPoints;
  int numPts = 100;
  float angleInc = 2 * XM_PI / numPts;
  float angle = 0;
  for (int i = 0; i < numPts; ++i)
  {
    V3 pt;
    pt.x = SPLINE_RADIUS * sin(angle);
    pt.z = SPLINE_RADIUS * cos(angle);
    pt.y = 20 + NoiseAtPoint(pt);
    controlPoints.push_back(pt);
    angle += angleInc;
  }

  _spline.Create(controlPoints.data(), (int)controlPoints.size());

  for (int i = 0; i < _settings.boids.num_flocks; ++i)
  {
    Flock* flock = new Flock(_settings.boids);
    flock->boids._maxSpeed = b.max_speed;

    float sum = b.wander_scale + b.separation_scale + b.cohesion_scale + b.alignment_scale + b.follow_scale;
    // Each flock gets its own seek behavior, because they need per flock information
    flock->boids.AddKinematics(flock->seek, _settings.boids.wander_scale / sum);
    flock->boids.AddKinematics(_behaviorSeparataion, _settings.boids.separation_scale / sum);
    flock->boids.AddKinematics(_behaviorCohesion, _settings.boids.cohesion_scale / sum);
    //flock->boids.AddKinematics(_behaviorAlignment, _settings.boids.alignment_scale / sum);
    flock->boids.AddKinematics(_landscapeFollow, _settings.boids.follow_scale / sum);

    int pointIdx = rand() % _spline._controlPoints.size();

    V3 center = V3(
      _spline._controlPoints[pointIdx].x,
      _spline._controlPoints[pointIdx].y,
      _spline._controlPoints[pointIdx].z);

    // Init the boids
    V3* pos = flock->boids._bodies.pos;
    V3* force = flock->boids._bodies.force;
    for (int i = 0; i < flock->boids._bodies.numBodies; ++i)
    {
      V3 pp(randf(-20.f, 20.f), 0, randf(-20.f, 20.f));
      float h = NoiseAtPoint(pp);
      pos[i] = center + V3(pp.x, h, pp.z);
      force[i] = V3(randf(-20.f, 20.f), 0, randf(-20.f, 20.f));
    }

    _flocks.Append(flock);
  }
}

//------------------------------------------------------------------------------
void BehaviorLandscapeFollow::Update(const ParticleKinematics::UpdateParams& params)
{
  // NOTE! This is called from the schedular threads, so setting namespace
  // on the blackboard will probably break :)
  float clearance = BLACKBOARD.GetFloatVar("landscape.landscapeClearance");
  V3* pos = params.bodies->pos;
  V3* acc = params.bodies->acc;
  V3* vel = params.bodies->vel;
  V3* force = params.bodies->force;
  int numBodies = params.bodies->numBodies;

  float ff = BLACKBOARD.GetFloatVar("landscape.pushForce");
  V3 pushForce(0, ff, 0);
  for (int i = params.start; i < params.end; ++i)
  {
    V3 curPos = pos[i];
    V3 curVel = vel[i];
    float h = NoiseAtPoint(curPos + V3(curVel.x, 0, curVel.z));

    V3 target = curPos;
    target.y = h + clearance;

    float dist = Distance(curPos, target);
    float scale = 1;
    if (dist < 20)
      scale = dist / 20;

    V3 desiredVel = maxSpeed * Normalize(target - curPos);
    force[i] += params.weight * ClampVector(desiredVel - vel[i], scale * ff);
  }
}

//------------------------------------------------------------------------------
void Landscape::UpdateFlock(const scheduler::TaskData& data)
{
  FlockKernelData* flockData = (FlockKernelData*)data.kernelData.data;
  Flock* flock = flockData->flock;

  V3 pp = flockData->target;
  flock->seek->target = pp; // XMLoadFloat3(&XMFLOAT3(pp.x, pp.y, pp.z));

  flock->boids.Update(flockData->updateState, false);
}

//------------------------------------------------------------------------------
void Landscape::UpdateBoids(const FixedUpdateState& state)
{
  rmt_ScopedCPUSample(Boids_Update);

  float dt = state.delta;

  SimpleAppendBuffer<TaskId, 2048> chunkTasks;

  V3 splineTarget = _spline.Interpolate(state.localTime.TotalSecondsAsFloat() * _settings.spline_speed);
  for (Flock* flock : _flocks)
  {
    FlockKernelData* data = (FlockKernelData*)g_ScratchMemory.Alloc(sizeof(FlockKernelData));
    *data = FlockKernelData{flock, splineTarget, _settings.boids.waypoint_radius, state};
    KernelData kd;
    kd.data = data;
    kd.size = sizeof(FlockKernelData);
    chunkTasks.Append(SCHEDULER.AddTask(kd, UpdateFlock));
  }

  for (const TaskId& taskId : chunkTasks)
    SCHEDULER.Wait(taskId);
}

//------------------------------------------------------------------------------
bool Landscape::Update(const UpdateState& state)
{
  float t = state.localTime.TotalSecondsAsFloat();
  V3 pp = _spline.Interpolate(t * _settings.spline_speed);

  //_followCamera.SetFollowTarget(XMLoadFloat3(&XMFLOAT3(pp.x, pp.y, pp.z)));
  _followCamera.SetFollowTarget(pp);

  const BoidSettings& b = _settings.boids;
  float ks = b.separation_scale * sinf(state.localTime.TotalSecondsAsFloat());
  float kc = b.cohesion_scale * cosf(state.localTime.TotalSecondsAsFloat());

  float sum = b.wander_scale + ks + kc + b.alignment_scale + b.follow_scale;

  for (Flock* f : _flocks)
  {
    f->boids.UpdateWeight(_behaviorSeparataion, ks / sum);
    f->boids.UpdateWeight(_behaviorCohesion, kc / sum);
    f->boids.UpdateWeight(_behaviorAlignment, b.alignment_scale / sum);
    f->boids.UpdateWeight(_landscapeFollow, b.follow_scale / sum);
    f->boids.UpdateWeight(f->seek, b.wander_scale / sum);
  }

  UpdateCameraMatrix(state);
  return true;
}

//------------------------------------------------------------------------------
bool Landscape::FixedUpdate(const FixedUpdateState& state)
{
  UpdateBoids(state);

  if (_curCamera == &_flockCamera)
  {
    if (_curFlockIdx < (int)FLOCK_TIMING.size() - 1)
    {
      float tt = state.localTime.TotalSecondsAsFloat();
      float next = FLOCK_TIMING[_curFlockIdx + 1].time;
      if (_curFlockIdx == -1 || tt >= next)
      {
        // time to swap flock!
        _curFlockIdx++;
        _followFlock = FLOCK_TIMING[_curFlockIdx].idx;
        _flockCamera.flock = _flocks[_followFlock];
        _flockCamera._pos = ToVector3(_flockCamera.flock->boids._center);
        _flockCamera._pos.x += randf(-20, 20);
        _flockCamera._pos.y += randf(5, 10);
        _flockCamera._pos.z += randf(5, 20);
      }
      else
      {
        float cur = FLOCK_TIMING[_curFlockIdx].time;
        if (next - tt < FLOCK_FADE)
        {
          // fade to black
          _flockFade = (next - tt) / FLOCK_FADE;
        }
        else if (tt - cur < FLOCK_FADE)
        {
          _flockFade = (tt - cur) / FLOCK_FADE;
        }
        else
        {
          _flockFade = 1;
        }
      }
    }
  }
  
  _curCamera->Update(state);
  return true;
}

//------------------------------------------------------------------------------
void Landscape::UpdateCameraMatrix(const UpdateState& state)
{
  const IoState& ioState = TANO.GetIoState();

  //if (!_flocks.Empty())
  //{
  //  if (g_KeyUpTrigger.IsTriggered('1'))
  //  {
  //    _followFlock = (_followFlock + 1) % _flocks.Size();
  //    _flockCamera.flock = _flocks[_followFlock];
  //    _flockCamera._pos = ToVector3(_flockCamera.flock->boids._center);
  //    _flockCamera._pos.x += randf(-20, 20);
  //    _flockCamera._pos.y += randf(5, 10);
  //    _flockCamera._pos.z += randf(5, 20);
  //  }

  //  if (g_KeyUpTrigger.IsTriggered('2'))
  //  {
  //    _followFlock = (_followFlock - 1) % _flocks.Size();
  //    _flockCamera.flock = _flocks[_followFlock];
  //    _flockCamera._pos = ToVector3(_flockCamera.flock->boids._center);
  //    _flockCamera._pos.x += randf(-20, 20);
  //    _flockCamera._pos.y += randf(5, 10);
  //    _flockCamera._pos.z += randf(5, 20);
  //  }
  //}

  if (g_KeyUpTrigger.IsTriggered('7'))
    _drawFlags ^= 0x1;

  if (g_KeyUpTrigger.IsTriggered('8'))
    _drawFlags ^= 0x2;

  if (g_KeyUpTrigger.IsTriggered('9'))
    _drawFlags ^= 0x4;

  Matrix view = _curCamera->_view;
  Matrix proj = _curCamera->_proj;
  Matrix viewProj = view * proj;

  RenderTargetDesc desc = GRAPHICS.GetBackBufferDesc();
  Vector4 dim((float)desc.width, (float)desc.height, 0, 0);
  _cbSky.ps0.dim = dim;
  _cbSky.ps0.cameraPos = _curCamera->_pos;
  _cbSky.ps0.cameraLookAt = _curCamera->_pos + _curCamera->_dir;

  _cbLandscape.vs0.world = Matrix::Identity();
  _cbLandscape.vs0.viewProj = viewProj.Transpose();
  _cbLandscape.vs0.cameraPos = _curCamera->_pos;

  // XXX: move scratch to shared
  //float beatHi = BLACKBOARD.GetFloatVar("Beat-Hi", state.globalTime.TotalSecondsAsFloat());
  //float beatLo = BLACKBOARD.GetFloatVar("Beat-Lo", state.globalTime.TotalSecondsAsFloat());
  float beatHi = 0;
  float beatLo = 0;
  _cbLandscape.vs0.musicParams = Vector4(beatHi, beatLo, 0, 0);

  _cbLandscape.gs0.dim = dim;

  // The depth value written to the z-buffer is after the w-divide,
  // but the z-value we compare against is still in proj-space, so
  // we'll need to do the backward transform:
  // f*(z-n) / (f-n)*z = zbuf => z = f*n / (f-zbuf(f-n))
  float n = _curCamera->_nearPlane;
  float f = _curCamera->_farPlane;

  _cbParticle.ps0.nearFar = Vector4(n, f, f * n, f - n);
  _cbParticle.gs0.world = Matrix::Identity();
  _cbParticle.gs0.viewProj = viewProj.Transpose();
  _cbParticle.gs0.cameraPos = _curCamera->_pos;

#if DEBUG_DRAW_PATH
  DEBUG_API.SetTransform(Matrix::Identity(), viewProj);
  float t = state.localTime.TotalSecondsAsFloat();
  DEBUG_API.AddDebugSphere(ToVector3(
    _spline.Interpolate(t * _settings.spline_speed)), 10, Color(1, 1, 1));
#endif
}

//------------------------------------------------------------------------------
inline void Vector3ToFloat(float* buf, const V3& v)
{
  buf[0] = v.x;
  buf[1] = v.y;
  buf[2] = v.z;
}

//------------------------------------------------------------------------------
inline void CopyPosNormal(float* buf, const V3& v, const V3& n)
{
  buf[0] = v.x;
  buf[1] = v.y;
  buf[2] = v.z;
  buf[3] = n.x;
  buf[4] = n.y;
  buf[5] = n.z;
}

//------------------------------------------------------------------------------
inline float SnapUp(float v, float snapSize)
{
  return snapSize * ceilf(v / snapSize);
};

//------------------------------------------------------------------------------
inline float SnapDown(float v, float snapSize)
{
  return snapSize * floorf(v / snapSize);
};

//------------------------------------------------------------------------------
Landscape::Chunk* Landscape::ChunkCache::FindChunk(float x, float y, int timestamp)
{
  auto it = _chunkLookup.find(make_pair(x, y));
  if (it == _chunkLookup.end())
    return nullptr;

  // because we don't delete chunks when they're overwritten, this might not be
  // the chunk we're looking for!
  Chunk* chunk = it->second;
  if (chunk->x != x || chunk->y != y)
  {
    _chunkLookup.erase(it);
    return nullptr;
  }

  chunk->lastAccessed = timestamp;
  return chunk;
}

//------------------------------------------------------------------------------
Landscape::Chunk* Landscape::ChunkCache::GetFreeChunk(float x, float y, int timestamp)
{
  Chunk* chunk;
  if (_used < CACHE_SIZE)
  {
    chunk = &_cache[_used++];
  }
  else
  {
    // cache is full, so replace the oldest element
    int oldestValue = _cache[0].lastAccessed;
    chunk = &_cache[0];
    for (int i = 1; i < CACHE_SIZE; ++i)
    {
      if (_cache[i].lastAccessed < oldestValue)
      {
        oldestValue = _cache[i].lastAccessed;
        chunk = &_cache[i];
      }
    }
  }

  chunk->x = x;
  chunk->y = y;
  float ofs = HALF_CHUNK_SIZE * GRID_SIZE;
  chunk->center = Vector3(x + ofs, 0, y - ofs);
  chunk->lastAccessed = timestamp;
  _chunkLookup[make_pair(x, y)] = chunk;
  return chunk;
}
//------------------------------------------------------------------------------
void Landscape::FillChunk(const TaskData& data)
{
  ChunkKernelData* chunkData = (ChunkKernelData*)data.kernelData.data;
  Chunk* chunk = chunkData->chunk;
  float x = chunkData->x;
  float z = chunkData->z;

  V3 v0, v1, v2, v3;
  V3 n0, n1;

  // first compute the noise values
  V3* noise = chunk->noiseValues;
  for (int i = 0; i <= CHUNK_SIZE; ++i)
  {
    for (int j = 0; j <= CHUNK_SIZE; ++j)
    {
      float xx0 = x + (j + 0) * GRID_SIZE;
      float zz0 = z + (i - 1) * GRID_SIZE;
      noise->x = xx0;
      noise->y = NOISE_HEIGHT * Perlin2D::Value(NOISE_SCALE_X * xx0, NOISE_SCALE_Z * zz0);
      noise->z = zz0;
      ++noise;
    }
  }

  int layerIncr[] = {1, 2};
  float* layerDest[] = {chunk->lowerData, chunk->upperData};
  float layerScale[] = {0.5f, 1.0f};
  for (int layer = 0; layer < 2; ++layer)
  {
    int incr = layerIncr[layer];
    float* dest = layerDest[layer];
    float scale = layerScale[layer];

    for (int i = 0; i < CHUNK_SIZE; i += incr)
    {
      for (int j = 0; j < CHUNK_SIZE; j += incr)
      {
        // 1--2
        // |  |
        // 0--3

        v0 = chunk->noiseValues[(i + 0) * (CHUNK_SIZE + 1) + (j + 0)];
        v1 = chunk->noiseValues[(i + incr) * (CHUNK_SIZE + 1) + (j + 0)];
        v2 = chunk->noiseValues[(i + incr) * (CHUNK_SIZE + 1) + (j + incr)];
        v3 = chunk->noiseValues[(i + 0) * (CHUNK_SIZE + 1) + (j + incr)];

        v0.y *= scale;
        v1.y *= scale;
        v2.y *= scale;
        v3.y *= scale;

        V3 e1, e2;
        e1 = v2 - v1;
        e2 = v0 - v1;
        n0 = Cross(e1, e2);
        n0 = Normalize(n0);

        e1 = v0 - v3;
        e2 = v2 - v3;
        n1 = Cross(e1, e2);
        n1 = Normalize(n1);

        CopyPosNormal(dest + 0, v0, n0);
        CopyPosNormal(dest + 6, v1, n0);
        CopyPosNormal(dest + 12, v2, n0);
        CopyPosNormal(dest + 18, v3, n1);
        dest += 24;
      }
    }
  }
}

//------------------------------------------------------------------------------
void Landscape::RasterizeLandscape()
{
  _curTick++;
  rmt_ScopedCPUSample(Landscape_Rasterize);

  // Create a large rect around the camera, and clip it using the camera planes
  Plane planes[6];
  ExtractPlanes(_curCamera->_view * _curCamera->_proj, true, planes);

  float ofs = 2 * _curCamera->_farPlane;
  Vector3 c = _curCamera->_pos;
  c.y = NOISE_HEIGHT;
  Vector3 buf0[16] = {c + Vector3(-ofs, 0, +ofs),
      c + Vector3(+ofs, 0, +ofs),
      c + Vector3(+ofs, 0, -ofs),
      c + Vector3(-ofs, 0, -ofs)};
  Vector3 buf1[16];

  int numVerts = 4;
  for (int i = 0; i < 6; ++i)
  {
    numVerts = ClipPolygonAgainstPlane(numVerts, buf0, planes[i], buf1);
    if (numVerts == 0)
      return;
    memcpy(buf0, buf1, numVerts * sizeof(Vector3));
  }

  Vector3 minPos(buf0[0]);
  Vector3 maxPos(buf0[0]);

  for (int i = 1; i < numVerts; ++i)
  {
    minPos = Vector3::Min(minPos, buf0[i]);
    maxPos = Vector3::Max(maxPos, buf0[i]);
  }

  // create a AABB for the clipped polygon
  float s = GRID_SIZE * CHUNK_SIZE;
  Vector3 topLeft(SnapDown(minPos.x, s), 0, SnapUp(maxPos.z, s));
  Vector3 topRight(SnapUp(maxPos.x, s), 0, SnapUp(maxPos.z, s));
  Vector3 bottomLeft(SnapDown(minPos.x, s), 0, SnapDown(minPos.z, s));
  Vector3 bottomRight(SnapUp(maxPos.x, s), 0, SnapDown(minPos.z, s));

  float x = topLeft.x;
  float z = topLeft.z;
  int chunkHits = 0;
  int chunkMisses = 0;

  V3 v0, v1, v2, v3;
  V3 n0, n1;

  SimpleAppendBuffer<TaskId, 2048> chunkTasks;
  SimpleAppendBuffer<Chunk*, 2048> chunks;

  for (float z = bottomLeft.z; z <= topLeft.z; z += s)
  {
    for (float x = bottomLeft.x; x <= bottomRight.x; x += s)
    {
      // check if the current chunk exists in the cache
      Chunk* chunk = _chunkCache.FindChunk(x, z, _curTick);
      if (chunk)
      {
        ++chunkHits;
      }
      else
      {
        ++chunkMisses;
        chunk = _chunkCache.GetFreeChunk(x, z, _curTick);
        ChunkKernelData* data = (ChunkKernelData*)g_ScratchMemory.Alloc(sizeof(ChunkKernelData));
        *data = ChunkKernelData{chunk, x, z};
        KernelData kd;
        kd.data = data;
        kd.size = sizeof(ChunkKernelData);
        chunkTasks.Append(SCHEDULER.AddTask(kd, FillChunk));
      }

      chunks.Append(chunk);
    }
  }

  for (const TaskId& taskId : chunkTasks)
    SCHEDULER.Wait(taskId);

  // sort the chunks by distance to camera (furthest first)
  Vector3 camPos = _curCamera->_pos;
  for (Chunk* chunk : chunks)
    chunk->dist = Vector3::DistanceSquared(camPos, chunk->center);

  sort(chunks.begin(),
      chunks.end(),
      [&](const Chunk* a, const Chunk* b)
      {
        return a->dist > b->dist;
      });

  // copy all the chunk data into the vertex buffer
  float* landscapeBuf = _ctx->MapWriteDiscard<float>(_landscapeGpuObjects._vb);
  V3* particleBuf = _ctx->MapWriteDiscard<V3>(_particleBundle.objects._vb);

  u32 numParticles = 0;
  for (const Chunk* chunk : chunks)
  {
    memcpy(landscapeBuf, chunk->upperData, Chunk::UPPER_DATA_SIZE * sizeof(float));
    landscapeBuf += Chunk::UPPER_DATA_SIZE;

    for (int i = 0 ; i < Chunk::UPPER_VERTS; ++i)
    {
      particleBuf->x = chunk->upperData[i * 6 + 0];
      particleBuf->y = chunk->upperData[i * 6 + 1];
      particleBuf->z = chunk->upperData[i * 6 + 2];
      ++particleBuf;
    }

    numParticles += Chunk::UPPER_VERTS;

    //for (int i = 0; i < CHUNK_SIZE; i += 2)
    //{
    //  for (int j = 0; j < CHUNK_SIZE; j += 2)
    //  {
    //    *particleBuf++ = chunk->noiseValues[i * (CHUNK_SIZE + 1) + j];
    //    numParticles++;
    //  }
    //}
  }
  _ctx->Unmap(_particleBundle.objects._vb);

  u32 numChunks = (u32)chunks.Size();

  _numUpperIndices = numChunks * Chunk::UPPER_INDICES;
  _numParticles = numParticles;

  for (const Chunk* chunk : chunks)
  {
    memcpy(landscapeBuf, chunk->lowerData, Chunk::LOWER_DATA_SIZE * sizeof(float));
    landscapeBuf += Chunk::LOWER_DATA_SIZE;
  }
  _numLowerIndices = numChunks * Chunk::LOWER_INDICES;

#if WITH_IMGUI
  TANO.AddPerfCallback([=]()
      {
        ImGui::Text("# particles: %d", numParticles);
        ImGui::Text("# chunks: %d", numChunks);
      });
#endif

  _ctx->Unmap(_landscapeGpuObjects._vb);
}

//------------------------------------------------------------------------------
void Landscape::RenderBoids(const ObjectHandle* renderTargets, ObjectHandle dsHandle)
{
  V3* boidPos = _ctx->MapWriteDiscard<V3>(_boidsBundle.objects._vb);

  int numBoids = 0;
  for (const Flock* flock : _flocks)
  {
    V3* pos = flock->boids._bodies.pos;
    memcpy(boidPos, pos, flock->boids._bodies.numBodies * sizeof(V3));
    boidPos += flock->boids._bodies.numBodies;
    numBoids += flock->boids._bodies.numBodies;
  }

  _ctx->Unmap(_boidsBundle.objects._vb);

  _cbParticle.Set(_ctx, 0);
  _ctx->SetBundleWithSamplers(_boidsBundle, ShaderType::PixelShader);

  // Unset the DSV, as we want to use it as a texture resource
  _ctx->SetRenderTargets(renderTargets, 2, ObjectHandle(), nullptr);
  ObjectHandle srv[] = {_particleTexture, dsHandle};
  _ctx->SetShaderResources(srv, 2, ShaderType::PixelShader);
  _ctx->Draw(numBoids, 0);
  _ctx->UnsetShaderResources(0, 2, ShaderType::PixelShader);
}

//------------------------------------------------------------------------------
bool Landscape::Render()
{
  rmt_ScopedCPUSample(Landscape_Render);
  static Color clearColor(0, 0, 0, 0);
  static const Color* clearColors[] = {&clearColor, &clearColor};
  FullscreenEffect* fullscreen = GRAPHICS.GetFullscreenEffect();

  ScopedRenderTargetFull rtColor(
      DXGI_FORMAT_R16G16B16A16_FLOAT, BufferFlag::CreateSrv, BufferFlag::CreateSrv);
  ScopedRenderTarget rtBloomEmissive(DXGI_FORMAT_R16G16B16A16_FLOAT);

  _cbComposite.ps0.tonemap = Vector4(
      _settings.tonemap.exposure,
      _settings.tonemap.min_white,
      _flockFade, 0);

  // We're using 2 render targets here. One for color, and one for bloom/emissive
  ObjectHandle renderTargets[] = {rtColor, rtBloomEmissive};
  _ctx->SetRenderTargets(renderTargets, 2, rtColor._dsHandle, &clearColors[0]);

  {
    // sky
    _cbSky.Set(_ctx, 0);
    _ctx->SetBundle(_skyBundle);
    _ctx->Draw(3, 0);
  }

  if (_renderLandscape)
  {
    RasterizeLandscape();

    _ctx->SetGpuObjects(_landscapeGpuObjects);
    _cbLandscape.Set(_ctx, 0);

    if (_drawFlags & DrawLower)
    {
      _ctx->SetGpuState(_landscapeLowerState);
      _ctx->DrawIndexed(_numLowerIndices, _numUpperIndices, 0);
    }

    if (_drawFlags & DrawUpper)
    {
      _ctx->SetGpuState(_landscapeState);
      _ctx->DrawIndexed(_numUpperIndices, 0, 0);
    }
  }

  if (_drawFlags & DrawParticles)
  {
    _cbParticle.Set(_ctx, 0);
    _ctx->SetBundleWithSamplers(_particleBundle, ShaderType::PixelShader);

    // Unset the DSV, as we want to use it as a texture resource
    _ctx->SetRenderTargets(renderTargets, 2, ObjectHandle(), nullptr);
    ObjectHandle srv[] = {_particleTexture, rtColor._dsHandle};
    _ctx->SetShaderResources(srv, 2, ShaderType::PixelShader);
    _ctx->Draw(_numParticles, 0);
    _ctx->UnsetShaderResources(0, 2, ShaderType::PixelShader);
  }

  if (_renderBoids)
  {
    RenderBoids(renderTargets, rtColor._dsHandle);
  }

  _ctx->UnsetRenderTargets(0, 2);

  ScopedRenderTarget rtColorBlurred(rtColor._desc, BufferFlag::CreateSrv | BufferFlag::CreateUav);
  fullscreen->Blur(rtColor, rtColorBlurred, rtColorBlurred._desc, 10, 1);

  ScopedRenderTarget rtEmissiveBlurred(rtColor._desc, BufferFlag::CreateSrv | BufferFlag::CreateUav);
  fullscreen->Blur(rtBloomEmissive, rtEmissiveBlurred, rtEmissiveBlurred._desc, 10, 1);

  RenderTargetDesc halfSize(
      rtColor._desc.width / 2, rtColor._desc.height / 2, DXGI_FORMAT_R16G16B16A16_FLOAT);
  ScopedRenderTarget rtScaleBias(halfSize);
  ScopedRenderTarget rtLensFlare(halfSize);

  {
    const LensFlareSettings& s = _settings.lens_flare;

    // lens flare
    fullscreen->ScaleBias(
        rtBloomEmissive, rtScaleBias, rtScaleBias._desc, s.scale_bias.scale, s.scale_bias.bias);

    _cbLensFlare.ps0.params = Vector4(s.dispersion, (float)s.num_ghosts, s.halo_width, s.strength);
    _cbLensFlare.Set(_ctx, 0);

    fullscreen->Execute(
        rtScaleBias, rtLensFlare, rtLensFlare._desc, ObjectHandle(), _lensFlareBundle.objects._ps, false);
  }

  {
    // composite
    static int showBuffer = 0;
    if (g_KeyUpTrigger.IsTriggered('B'))
      showBuffer = (showBuffer + 1) % 4;

    if (showBuffer == 0)
    {
      _cbComposite.Set(_ctx, 0);

      ObjectHandle inputs[] = {rtColor, rtColorBlurred, rtEmissiveBlurred, rtLensFlare};
      fullscreen->Execute(inputs,
          4,
          GRAPHICS.GetBackBuffer(),
          GRAPHICS.GetBackBufferDesc(),
          GRAPHICS.GetDepthStencil(),
          _compositeBundle.objects._ps,
          false);
    }
    else if (showBuffer == 1)
    {
      fullscreen->Copy(rtColorBlurred, GRAPHICS.GetBackBuffer(), GRAPHICS.GetBackBufferDesc(), false);
    }
    else if (showBuffer == 2)
    {
      fullscreen->Copy(rtEmissiveBlurred, GRAPHICS.GetBackBuffer(), GRAPHICS.GetBackBufferDesc(), false);
    }
    else
    {
      fullscreen->Copy(rtLensFlare, GRAPHICS.GetBackBuffer(), GRAPHICS.GetBackBufferDesc(), false);
    }
  }

#if DEBUG_DRAW_PATH
  for (int i = 0; i < _spline._controlPoints.size() - 1; ++i)
  {
    Vector3 p0(_spline._controlPoints[i].x, _spline._controlPoints[i].y, _spline._controlPoints[i].z);
    Vector3 p1(_spline._controlPoints[i + 1].x, _spline._controlPoints[i + 1].y, _spline._controlPoints[i + 1].z);
    DEBUG_API.AddDebugLine(p0, p1, Color(1, 1, 1));
  }
#endif

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
  auto UpdateWeight = [this](ParticleKinematics* k, float w)
  {
    for (Flock* f : _flocks)
    {
      f->boids.UpdateWeight(k, w);
    }
  };

  ImGui::SliderFloat("spline-speed", &_settings.spline_speed, 0, 20);

  static bool lensFlareConfig = false;
  ImGui::Checkbox("lens flare config", &lensFlareConfig);
  if (lensFlareConfig)
  {
    ImGui::SliderFloat("dispersion", &_settings.lens_flare.dispersion, 0, 2);
    ImGui::SliderInt("Num ghosts", &_settings.lens_flare.num_ghosts, 1, 10);
    ImGui::SliderFloat("Halo width", &_settings.lens_flare.halo_width, 0, 3);
    ImGui::SliderFloat("Strength", &_settings.lens_flare.strength, 0, 1);

    ImGui::SliderFloat("scale", &_settings.lens_flare.scale_bias.scale, 0, 3);
    ImGui::SliderFloat("bias", &_settings.lens_flare.scale_bias.bias, 0, 1);
  }

  ImGui::SliderFloat("Exposure", &_settings.tonemap.exposure, 0.1f, 20.0f);
  ImGui::SliderFloat("Min White", &_settings.tonemap.min_white, 0.1f, 20.0f);

  ImGui::Separator();

  ImGui::Checkbox("Render landscape", &_renderLandscape);
  ImGui::Checkbox("Render boids", &_renderBoids);
  static bool boidConfig = false;
  ImGui::Checkbox("boid config", &boidConfig);
  if (boidConfig)
  {
    //ImGui::InputInt("NumVerts", (int*)&_numUpperIndices);
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
      for (Flock* flock : _flocks)
        UpdateWeight(flock->seek, _settings.boids.wander_scale / sum);
      UpdateWeight(_landscapeFollow, _settings.boids.follow_scale / sum);
    }

    ImGui::SliderFloat("MaxSpeed", &_settings.boids.max_speed, 10.f, 1000.f);
    ImGui::SliderFloat("MaxForce", &_settings.boids.max_force, 10.f, 1000.f);
    ImGui::SliderFloat("SepDist", &_settings.boids.separation_distance, 1.f, 100.f);
    ImGui::SliderFloat("CohDist", &_settings.boids.cohesion_distance, 1.f, 100.f);
  }

  if (ImGui::Button("Reset"))
    Reset();
}
#endif

//------------------------------------------------------------------------------
#if WITH_IMGUI
void Landscape::SaveParameterSet(bool inc)
{
  _freeflyCamera.ToProtocol(&_settings.camera);
  SaveSettings(_settings, inc);
}
#endif

//------------------------------------------------------------------------------
void Landscape::Reset()
{
  InitBoids();
}

//------------------------------------------------------------------------------
bool Landscape::Close()
{
  return true;
}

//------------------------------------------------------------------------------
BaseEffect* Landscape::Create(const char* name, const char* config, u32 id)
{
  return new Landscape(name, config, id);
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

//------------------------------------------------------------------------------
void Landscape::FlockCamera::Update(const FixedUpdateState& state)
{
  Vector3 targetPos = ToVector3(flock->seek->target);
  //Vector3 curPos = flock->boids._center;
  Vector3 curPos = _pos;

  Vector3 dir = Normalize(targetPos - curPos);
  Vector3 targetVel = 10 * dir;

  V3 cc(curPos.x, curPos.y, curPos.z);
  V3 vv(targetVel.x, targetVel.y, targetVel.z);
  V3 tt(targetPos.x, targetPos.y, targetPos.z);

  SmoothDriver::DriveCubic(&cc, &vv, &tt, &vv, 50, state.delta);

  _pos = ToVector3(cc);
  _dir = ToVector3(Normalize(vv));

  //_pos = flock->boids._center;
  //Vector3 target = flock->seek->target;
  //_dir = Normalize(target - _pos);
  Camera::Update(state);
}
