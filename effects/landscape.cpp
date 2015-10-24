#include "landscape.hpp"
#include "../tano.hpp"
#include "../graphics.hpp"
#include "../graphics_context.hpp"
#include "../demo_engine.hpp"
#include "../resource_manager.hpp"
#include "../init_sequence.hpp"
#include "../generated/demo.parse.hpp"
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
#include "../vertex_types.hpp"

using namespace tano;
using namespace tano::scheduler;
using namespace bristol;
using namespace DirectX;

static const vec3 ZERO3(0, 0, 0);
static const float GRID_SIZE = 5;
static const int MAX_CHUNKS = 750;
static const float NOISE_HEIGHT = 50;
static const float NOISE_SCALE_X = 0.01f;
static const float NOISE_SCALE_Z = 0.01f;

static const float SPLINE_RADIUS = 500;

struct FlockTiming
{
  float time;
  int idx;
};

static const float FLOCK_FADE = 1.0f;

static const float START_TIME = 24.0f;
vector<FlockTiming> FLOCK_TIMING = {
    {0.0f, 2},
    {27.5f - START_TIME, 6},
    {39.0f - START_TIME, 5},
    {43.0f - START_TIME, 2},
    {1000, 0},
};

#define DEBUG_DRAW_PATH 0
#define PROFILE_UPDATES 0

int Landscape::Chunk::nextId = 1;

static RandomInt RANDOM_INT;

//------------------------------------------------------------------------------
void BehaviorSpacing::Update(const ParticleKinematics::UpdateParams& params)
{
  vec3* pos = params.bodies->pos;
  vec3* vel = params.bodies->vel;
  vec3* force = params.bodies->forces[forceIdx];
  int numBodies = params.bodies->numBodies;
  float maxSpeed = params.p->_maxSpeed;

  float spacing = g_Blackboard->GetFloatVar("landscape.spacing");
  float f = g_Blackboard->GetFloatVar("landscape.spacingForce");

  // pick a random number of points, and try to adjust their spacing
  for (int i = 0; i < numBodies; ++i)
  {
    int a = RANDOM_INT.Next() % numBodies;
    int b = RANDOM_INT.Next() % numBodies;

    float dist = Distance(pos[a], pos[b]);
    if (dist > 0.f)
    {
      vec3 dir = ((dist - spacing) / dist) * (pos[b] - pos[a]);
      force[a] += f * 0.5f * dir;
      force[b] -= f * 0.5f * dir;
    }
  }
}

//------------------------------------------------------------------------------
BehaviorPathFollow::BehaviorPathFollow(const CardinalSpline& spline) : _spline(spline)
{
}

//------------------------------------------------------------------------------
void BehaviorPathFollow::Update(const UpdateParams& params)
{
  if (_splineOffset.empty())
  {
    _splineOffset.resize(params.bodies->numBodies);
    for (int i = 0; i < params.bodies->numBodies; ++i)
    {
      float t = 0;
      float dt = 1 / 10.f;
      float end = (float)_spline._controlPoints.size();

      float closestDist = FLT_MAX;
      float closestT = 0;
      while (t < end)
      {
        float cand = Distance(params.bodies->pos[i], _spline.Interpolate(t));
        if (cand < closestDist)
        {
          closestDist = cand;
          closestT = t;
        }

        t += dt;
      }

      _splineOffset[i] = closestT;
    }
  }

  vec3* pos = params.bodies->pos;
  vec3* vel = params.bodies->vel;
  vec3* force = params.bodies->forces[forceIdx];
  int numBodies = params.bodies->numBodies;
  float maxSpeed = params.p->_maxSpeed;

  for (int i = params.start; i < params.end; ++i)
  {
    float t = _splineOffset[i];
    vec3 desiredVel = maxSpeed * Normalize(_spline.Interpolate(t) - pos[i]);
    //force[i] += params.weight * ClampVector(desiredVel - vel[i], maxForce);
    force[i] += params.weight * (desiredVel - vel[i]);

    _splineOffset[i] += 0.005f;
  }
}

//------------------------------------------------------------------------------
float NoiseAtPoint(const vec3& v)
{
  return NOISE_HEIGHT * Perlin2D::Value(NOISE_SCALE_X * v.x, NOISE_SCALE_Z * v.z);
}

//------------------------------------------------------------------------------
Landscape::Flock::Flock(const BoidSettings& settings, const CardinalSpline& spline)
{
  boids.Init(settings.boids_per_flock, settings.max_speed, settings.max_force);
  seek = new BehaviorSeek();
  follow = new BehaviorPathFollow(spline);
}

//------------------------------------------------------------------------------
Landscape::Flock::~Flock()
{
  SAFE_DELETE(seek);
}

//------------------------------------------------------------------------------
Landscape::Landscape(const string& name, const string& config, u32 id)
    : BaseEffect(name, config, id)
{
}

//------------------------------------------------------------------------------
Landscape::~Landscape()
{
}

//------------------------------------------------------------------------------
bool Landscape::OnConfigChanged(const vector<char>& buf)
{
  _settings = ParseLandscapeSettings(InputBuffer(buf));
  return true;
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
  LOG_INFO("Elapsed: ", elapsed);`
  return false;
#endif

  BEGIN_INIT_SEQUENCE();

  _freeflyCamera.FromProtocol(_settings.camera);

  // clang-format off

  vector<u32> lowerIndices, upperIndices;
  for (int i = 0; i < MAX_CHUNKS; ++i)
  {
    GeneratePlaneIndices(
      NUM_CHUNK_VERTS, NUM_CHUNK_VERTS, i * NUM_CHUNK_VERTS * NUM_CHUNK_VERTS, &lowerIndices);

    GeneratePlaneIndices(
      UPPER_NUM_CHUNK_VERTS, UPPER_NUM_CHUNK_VERTS, i * UPPER_NUM_CHUNK_VERTS * UPPER_NUM_CHUNK_VERTS, &upperIndices);
  }

  // Blend desc that doesn't write to the emissive channel
  CD3D11_BLEND_DESC blendDescAlphaNoEmissive = blendDescBlendSrcAlpha;
  blendDescAlphaNoEmissive.IndependentBlendEnable = TRUE;
  blendDescAlphaNoEmissive.RenderTarget[1].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALPHA;

  CD3D11_BLEND_DESC blendDescNoEmissive = CD3D11_BLEND_DESC(CD3D11_DEFAULT());
  blendDescNoEmissive.IndependentBlendEnable = TRUE;
  blendDescNoEmissive.RenderTarget[1].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALPHA;

  INIT(_landscapeLowerBundle.Create(BundleOptions()
    .DynamicVb(MAX_CHUNKS * Chunk::LOWER_VERTS, sizeof(vec3))
    .StaticIb(lowerIndices)
    .VertexShader("shaders/out/landscape.lower", "VsLandscape")
    .GeometryShader("shaders/out/landscape.lower", "GsLandscape")
    .PixelShader("shaders/out/landscape.lower", "PsLandscape")
    .InputElement(CD3D11_INPUT_ELEMENT_DESC("POSITION", DXGI_FORMAT_R32G32B32_FLOAT))
    .BlendDesc(blendDescNoEmissive)));

  INIT(_landscapeUpperBundle.Create(BundleOptions()
    .DynamicVb(MAX_CHUNKS * Chunk::UPPER_VERTS, sizeof(vec3))
    .StaticIb(upperIndices)
    .VertexShader("shaders/out/landscape.landscape", "VsLandscape")
    .GeometryShader("shaders/out/landscape.landscape", "GsLandscape")
    .PixelShader("shaders/out/landscape.landscape", "PsLandscape")
    .InputElement(CD3D11_INPUT_ELEMENT_DESC("POSITION", DXGI_FORMAT_R32G32B32_FLOAT))
    .BlendDesc(blendDescAlphaNoEmissive)));

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

  INIT(_particleBundle.Create(BundleOptions()
    .DynamicVb(1024 * 1024, sizeof(vec3))
    .VertexShader("shaders/out/landscape.particle", "VsParticle")
    .GeometryShader("shaders/out/landscape.particle", "GsParticle")
    .PixelShader("shaders/out/landscape.particle", "PsParticle")
    .InputElement(CD3D11_INPUT_ELEMENT_DESC("POSITION", DXGI_FORMAT_R32G32B32_FLOAT))
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_POINTLIST)
    .DepthStencilDesc(depthDescDepthWriteDisabled)
    .BlendDesc(blendDescBlendOneOne)));

  INIT(_boidsBundle.Create(BundleOptions()
    .DynamicVb(1024 * 1024, sizeof(vec3))
    .VertexShader("shaders/out/landscape.boids", "VsParticle")
    .GeometryShader("shaders/out/landscape.boids", "GsParticle")
    .PixelShader("shaders/out/landscape.boids", "PsParticle")
    .InputElement(CD3D11_INPUT_ELEMENT_DESC("POSITION", DXGI_FORMAT_R32G32B32_FLOAT))
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_POINTLIST)
    .DepthStencilDesc(depthDescDepthWriteDisabled)
    .BlendDesc(blendDescBlendOneOne)
    .RasterizerDesc(rasterizeDescCullNone)));
  // clang-format on

  INIT_FATAL(_cbLensFlare.Create());
  INIT_FATAL(_cbComposite.Create());
  INIT_FATAL(_cbSky.Create());
  INIT_FATAL(_cbLandscape.Create());
  INIT_FATAL(_cbParticle.Create());

  INIT_RESOURCE_FATAL(_particleTexture, RESOURCE_MANAGER.LoadTexture("gfx/landscape_particle.png"));
  INIT_RESOURCE_FATAL(_boidsTexture, RESOURCE_MANAGER.LoadTexture("gfx/landscape_boids.png"));

  Reset();

  const BoidSettings& b = _settings.boids;

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
  SAFE_DELETE(_behaviorLandscapeFollow);

  const BoidSettings& b = _settings.boids;
  _behaviorSeparataion = new BehaviorSeparataion(b.separation_distance);
  _behaviorCohesion = new BehaviorCohesion(b.cohesion_distance);
  _behaviorLandscapeFollow = new BehaviorLandscapeFollow();
  _behaviorSpacing = new BehaviorSpacing();

  float clearance = g_Blackboard->GetFloatVar("landscape.clearance");

  vector<vec3> controlPoints;
  int numPts = 100;
  float angleInc = 2 * XM_PI / numPts;
  float angle = 0;
  for (int i = 0; i < numPts; ++i)
  {
    vec3 pt;
    pt.x = SPLINE_RADIUS * sin(angle);
    pt.z = SPLINE_RADIUS * cos(angle);
    float h = NoiseAtPoint(pt);
    pt.y = max(h, h/2) + clearance;
    controlPoints.push_back(pt);
    angle += angleInc;
  }

  _spline.Create(controlPoints.data(), (int)controlPoints.size());

  for (int i = 0; i < _settings.boids.num_flocks; ++i)
  {
    Flock* flock = new Flock(_settings.boids, _spline);
    flock->boids._maxSpeed = b.max_speed;

    float sum =
        b.wander_scale + /*b.separation_scale + */b.cohesion_scale + /*b.alignment_scale + */b.follow_scale;
    // Each flock gets its own seek behavior, because they need per flock information
    //flock->boids.AddKinematics(flock->seek, _settings.boids.wander_scale / sum);
    //flock->boids.AddKinematics(_behaviorSeparataion, _settings.boids.separation_scale / sum);
    //flock->boids.AddKinematics(_behaviorCohesion, _settings.boids.cohesion_scale / sum);

    flock->boids.AddKinematics(_behaviorLandscapeFollow, _settings.boids.follow_scale / sum);
    flock->boids.AddKinematics(flock->follow, b.wander_scale / sum);
    flock->boids.AddKinematics(_behaviorSpacing, b.cohesion_scale / sum);

    int pointIdx = _randomInt.Next() % _spline._controlPoints.size();

    vec3 center = vec3(_spline._controlPoints[pointIdx].x,
        _spline._controlPoints[pointIdx].y,
        _spline._controlPoints[pointIdx].z);

    // Init the boids
    vec3* pos = flock->boids._bodies.pos;
    //vec3* force = flock->boids._bodies.force;
    for (int i = 0; i < flock->boids._bodies.numBodies; ++i)
    {
      vec3 pp(_random.Next(-20.f, 20.f), 0, _random.Next(-20.f, 20.f));
      float h = NoiseAtPoint(pp);
      pp.y = max(h, h / 2) + clearance;
      pos[i] = center + vec3(pp.x, h, pp.z);
      //force[i] =
      //    vec3(_random.Next(-20.f, 20.f), _random.Next(-20.f, 20.f), _random.Next(-20.f, 20.f));
    }

    _flocks.Append(flock);
  }
}

//------------------------------------------------------------------------------
void BehaviorLandscapeFollow::Update(const ParticleKinematics::UpdateParams& params)
{
  float clearance = g_Blackboard->GetFloatVar("landscape.clearance");
  vec3* pos = params.bodies->pos;
  vec3* acc = params.bodies->acc;
  vec3* vel = params.bodies->vel;
  vec3* force = params.bodies->forces[forceIdx];
  int numBodies = params.bodies->numBodies;
  float maxSpeed = params.p->_maxSpeed;

  // NOTE! This is called from the schedular threads, so setting namespace
  // on the blackboard will probably break :)
  float ff = g_Blackboard->GetFloatVar("landscape.pushForce");
  float slowingDistance = g_Blackboard->GetFloatVar("landscape.slowingDistance");

  for (int i = params.start; i < params.end; ++i)
  {
    vec3 curPos = pos[i];
    vec3 curVel = vel[i];
    vec3 target = curPos + curVel;

    float h = NoiseAtPoint(target);
    target.y = max(h, h / 2) + clearance;
    
    float dist = Distance(curPos, target);
    float speed = min(maxSpeed, maxSpeed * dist / slowingDistance);
    vec3 desiredVel = speed * Normalize(target - curPos);
    force[i] += params.weight * (desiredVel - vel[i]);
  }
}

//------------------------------------------------------------------------------
void Landscape::UpdateFlock(const scheduler::TaskData& data)
{
  FlockKernelData* flockData = (FlockKernelData*)data.kernelData.data;
  Flock* flock = flockData->flock;

  vec3 pp = flockData->target;
  flock->seek->target = pp; // XMLoadFloat3(&XMFLOAT3(pp.x, pp.y, pp.z));

  flock->boids.Update(flockData->deltaTime, false);
}

//------------------------------------------------------------------------------
void Landscape::UpdateBoids(const FixedUpdateState& state)
{
  rmt_ScopedCPUSample(Boids_Update);

  float dt = state.delta;

  SimpleAppendBuffer<TaskId, 2048> chunkTasks;

  vec3 splineTarget =
      _spline.Interpolate(state.localTime.TotalSecondsAsFloat() * _settings.spline_speed);
  for (Flock* flock : _flocks)
  {
    FlockKernelData* data = (FlockKernelData*)g_ScratchMemory.Alloc(sizeof(FlockKernelData));
    *data = FlockKernelData{flock, splineTarget, _settings.boids.waypoint_radius, state.delta};
    KernelData kd;
    kd.data = data;
    kd.size = sizeof(FlockKernelData);
    chunkTasks.Append(g_Scheduler->AddTask(kd, UpdateFlock));
  }

  for (const TaskId& taskId : chunkTasks)
    g_Scheduler->Wait(taskId);
}

//------------------------------------------------------------------------------
bool Landscape::Update(const UpdateState& state)
{
  float t = state.localTime.TotalSecondsAsFloat();
  vec3 pp = _spline.Interpolate(t * _settings.spline_speed);

  vec3 sunDir = Normalize(g_Blackboard->GetVec3Var("landscape.sunDir"));
  _cbLandscape.ps0.sunDir = sunDir;
  _cbParticle.ps0.sunDir = sunDir;
  _cbSky.ps0.sunDir = sunDir;

  _cbComposite.ps0.time =
      vec2(state.localTime.TotalSecondsAsFloat(), state.globalTime.TotalSecondsAsFloat());

  float ss = g_Blackboard->GetFloatVar("landscape.sepScale");
  const BoidSettings& b = _settings.boids;

  for (int i = 0; i < _flocks.Size(); ++i)
  {
    Flock* f = _flocks[i];

    float ks = b.separation_scale * ss * sinf(state.localTime.TotalSecondsAsFloat() + i * 1.0f);
    float kc = b.cohesion_scale * ss * cosf(1.5f * state.localTime.TotalSecondsAsFloat() + i * 1.5f);

    float sum = b.wander_scale + ks + kc + b.alignment_scale + b.follow_scale;

    f->boids.UpdateWeight(_behaviorSeparataion, ks / sum);
    f->boids.UpdateWeight(_behaviorCohesion, kc / sum);
    f->boids.UpdateWeight(_behaviorLandscapeFollow, b.follow_scale / sum);
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
        _flockCamera._pos = _flockCamera.flock->boids._center;
        _flockCamera._pos.x += _random.Next(-20, 20);
        _flockCamera._pos.y += _random.Next(5, 10);
        _flockCamera._pos.z += _random.Next(5, 20);
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

  _curCamera->Update(state.delta);
  return true;
}

//------------------------------------------------------------------------------
void Landscape::UpdateCameraMatrix(const UpdateState& state)
{
  const IoState& ioState = TANO.GetIoState();

#if WITH_IMGUI
  static bool drawForces = false;
  if (g_KeyUpTrigger.IsTriggered('6'))
    drawForces = !drawForces;

  if (drawForces)
    _flocks[0]->boids.DrawForcePlot();
#endif

  if (g_KeyUpTrigger.IsTriggered('7'))
    _drawFlags ^= DrawUpper;

  if (g_KeyUpTrigger.IsTriggered('8'))
    _drawFlags ^= DrawLower;

  if (g_KeyUpTrigger.IsTriggered('9'))
    _drawFlags ^= DrawParticles;

  Matrix view = _curCamera->_view;
  Matrix proj = _curCamera->_proj;
  Matrix viewProj = view * proj;

  RenderTargetDesc desc = g_Graphics->GetBackBufferDesc();
  vec4 dim((float)desc.width, (float)desc.height, 0, 0);
  _cbSky.ps0.dim = dim;
  _cbSky.ps0.cameraPos = _curCamera->_pos;
  _cbSky.ps0.cameraLookAt = _curCamera->_pos + _curCamera->_dir;

  _cbLandscape.vs0.world = Matrix::Identity();
  _cbLandscape.vs0.viewProj = viewProj.Transpose();
  _cbLandscape.vs0.cameraPos = _curCamera->_pos;

  _cbLandscape.gs0.dim = dim;

  // The depth value written to the z-buffer is after the w-divide,
  // but the z-value we compare against is still in proj-space, so
  // we'll need to do the backward transform:
  // f*(z-n) / (f-n)*z = zbuf => z = f*n / (f-zbuf(f-n))
  float n = _curCamera->_nearPlane;
  float f = _curCamera->_farPlane;

  _cbParticle.ps0.nearFar = vec4(n, f, f * n, f - n);
  _cbParticle.gs0.world = Matrix::Identity();
  _cbParticle.gs0.viewProj = viewProj.Transpose();
  _cbParticle.gs0.cameraPos = _curCamera->_pos;
  _cbParticle.gs0.time =
      vec4(state.localTime.TotalSecondsAsFloat(), state.globalTime.TotalSecondsAsFloat(), 0, 0);

#if DEBUG_DRAW_PATH
  DEBUG_API.SetTransform(Matrix::Identity(), viewProj);
  float t = state.localTime.TotalSecondsAsFloat();
  DEBUG_API.AddDebugSphere(_spline.Interpolate(t * _settings.spline_speed), 10, Color(1, 1, 1));
#endif
}

//------------------------------------------------------------------------------
inline void Vector3ToFloat(float* buf, const vec3& v)
{
  buf[0] = v.x;
  buf[1] = v.y;
  buf[2] = v.z;
}

//------------------------------------------------------------------------------
inline void CopyPosNormal(float* buf, const vec3& v, const vec3& n)
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
  chunk->center = vec3(x + ofs, 0, y - ofs);
  chunk->lastAccessed = timestamp;
  _chunkLookup[make_pair(x, y)] = chunk;
  return chunk;
}
//------------------------------------------------------------------------------
void Landscape::CopyOutTask(const scheduler::TaskData& data)
{
  CopyKernelData* kernelData = (CopyKernelData*)data.kernelData.data;
  const Chunk* chunk = kernelData->chunk;

  memcpy(kernelData->lowerBuf, chunk->lowerData, Chunk::LOWER_VERTS * sizeof(vec3));
  memcpy(kernelData->upperBuf, chunk->upperData, Chunk::UPPER_VERTS * sizeof(vec3));
  memcpy(kernelData->particleBuf, chunk->upperData, Chunk::UPPER_VERTS * sizeof(vec3));
}

//------------------------------------------------------------------------------
void Landscape::FillChunk(const TaskData& data)
{
  ChunkKernelData* chunkData = (ChunkKernelData*)data.kernelData.data;
  Chunk* chunk = chunkData->chunk;
  float x = chunkData->x;
  float z = chunkData->z;

  vec3 v0, v1, v2, v3;
  vec3 n0, n1;

  // first compute the noise values
  vec3* noise = chunk->noiseValues;
  for (int i = 0; i < NUM_CHUNK_VERTS; ++i)
  {
    for (int j = 0; j < NUM_CHUNK_VERTS; ++j)
    {
      float xx0 = x + (j + 0) * GRID_SIZE;
      float zz0 = z + (i - 1) * GRID_SIZE;
      noise->x = xx0;
      noise->y = NoiseAtPoint(vec3{xx0, 0, zz0});
      noise->z = zz0;
      ++noise;
    }
  }

  int layerIncr[] = {1, 2};
  vec3* layerDest[] = {chunk->lowerData, chunk->upperData};
  float layerScale[] = {0.5f, 1.0f};
  for (int layer = 0; layer < 2; ++layer)
  {
    int incr = layerIncr[layer];
    vec3* dest = layerDest[layer];
    float scale = layerScale[layer];

    for (int i = 0; i < NUM_CHUNK_VERTS; i += incr)
    {
      for (int j = 0; j < NUM_CHUNK_VERTS; j += incr)
      {
        vec3 v = chunk->noiseValues[i * NUM_CHUNK_VERTS + j];
        v.y *= scale;
        *dest++ = v;
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
  vec3 c = _curCamera->_pos;
  c.y = NOISE_HEIGHT;
  vec3 buf0[16] = {c + vec3(-ofs, 0, +ofs),
      c + vec3(+ofs, 0, +ofs),
      c + vec3(+ofs, 0, -ofs),
      c + vec3(-ofs, 0, -ofs)};
  vec3 buf1[16];

  int numVerts = 4;
  for (int i = 0; i < 6; ++i)
  {
    numVerts = ClipPolygonAgainstPlane(numVerts, buf0, planes[i], buf1);
    if (numVerts == 0)
      return;
    memcpy(buf0, buf1, numVerts * sizeof(vec3));
  }

  vec3 minPos(buf0[0]);
  vec3 maxPos(buf0[0]);

  for (int i = 1; i < numVerts; ++i)
  {
    minPos = Min(minPos, buf0[i]);
    maxPos = Max(maxPos, buf0[i]);
  }

  // create a AABB for the clipped polygon
  float s = GRID_SIZE * NUM_CHUNK_QUADS;
  vec3 topLeft(SnapDown(minPos.x, s), 0, SnapUp(maxPos.z, s));
  vec3 topRight(SnapUp(maxPos.x, s), 0, SnapUp(maxPos.z, s));
  vec3 bottomLeft(SnapDown(minPos.x, s), 0, SnapDown(minPos.z, s));
  vec3 bottomRight(SnapUp(maxPos.x, s), 0, SnapDown(minPos.z, s));

  float x = topLeft.x;
  float z = topLeft.z;
  int chunkHits = 0;
  int chunkMisses = 0;

  vec3 v0, v1, v2, v3;
  vec3 n0, n1;

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
        chunkTasks.Append(g_Scheduler->AddTask(kd, FillChunk));
      }

      chunks.Append(chunk);
    }
  }

  for (const TaskId& taskId : chunkTasks)
    g_Scheduler->Wait(taskId);

  // sort the chunks by distance to camera (furthest first)
  vec3 camPos = _curCamera->_pos;
  for (Chunk* chunk : chunks)
    chunk->dist = DistanceSquared(camPos, chunk->center);

  sort(chunks.begin(),
      chunks.end(),
      [&](const Chunk* a, const Chunk* b)
      {
        return a->dist > b->dist;
      });

  // copy all the chunk data into the vertex buffer
  vec3* lowerBuf = _ctx->MapWriteDiscard<vec3>(_landscapeLowerBundle.objects._vb);
  vec3* upperBuf = _ctx->MapWriteDiscard<vec3>(_landscapeUpperBundle.objects._vb);
  vec3* particleBuf = _ctx->MapWriteDiscard<vec3>(_particleBundle.objects._vb);

  // upper chunks, and particles
  SimpleAppendBuffer<TaskId, 2048> copyTasks;

  for (const Chunk* chunk : chunks)
  {
    CopyKernelData* data = (CopyKernelData*)g_ScratchMemory.Alloc(sizeof(CopyKernelData));
    *data = CopyKernelData{ chunk, lowerBuf, upperBuf, particleBuf };
    KernelData kd;
    kd.data = data;
    kd.size = sizeof(ChunkKernelData);
    chunkTasks.Append(g_Scheduler->AddTask(kd, CopyOutTask));

    lowerBuf += Chunk::LOWER_VERTS;
    upperBuf += Chunk::UPPER_VERTS;
    particleBuf += Chunk::UPPER_VERTS;
  }

  for (const TaskId& taskId : copyTasks)
    g_Scheduler->Wait(taskId);

  _ctx->Unmap(_particleBundle.objects._vb);
  _ctx->Unmap(_landscapeLowerBundle.objects._vb);
  _ctx->Unmap(_landscapeUpperBundle.objects._vb);

  u32 numChunks = (u32)chunks.Size();
  assert(numChunks < MAX_CHUNKS);
  _numChunks = numChunks;
  _numLowerIndices = numChunks * Chunk::LOWER_INDICES;
  _numUpperIndices = numChunks * Chunk::UPPER_INDICES;
  _numParticles = numChunks * Chunk::UPPER_VERTS;

#if WITH_IMGUI
  TANO.AddPerfCallback([=]()
      {
        ImGui::Text("# particles: %d", _numParticles);
        ImGui::Text("# chunks: %d", _numChunks);
      });
#endif
}

//------------------------------------------------------------------------------
void Landscape::RenderBoids(const ObjectHandle* renderTargets, ObjectHandle dsHandle)
{
  vec3* boidPos = _ctx->MapWriteDiscard<vec3>(_boidsBundle.objects._vb);

  int numBoids = 0;
  for (const Flock* flock : _flocks)
  {
    vec3* pos = flock->boids._bodies.pos;
    memcpy(boidPos, pos, flock->boids._bodies.numBodies * sizeof(vec3));
    boidPos += flock->boids._bodies.numBodies;
    numBoids += flock->boids._bodies.numBodies;
  }

  _ctx->Unmap(_boidsBundle.objects._vb);

  _cbParticle.Set(_ctx, 0);
  _ctx->SetBundleWithSamplers(_boidsBundle, ShaderType::PixelShader);

  // Unset the DSV, as we want to use it as a texture resource
  _ctx->SetRenderTargets(renderTargets, 1, ObjectHandle(), nullptr);
  ObjectHandle srv[] = {_boidsTexture, dsHandle};
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
  FullscreenEffect* fullscreen = g_Graphics->GetFullscreenEffect();

  ScopedRenderTargetFull rtColor(
      DXGI_FORMAT_R16G16B16A16_FLOAT, BufferFlag::CreateSrv, BufferFlag::CreateSrv);
  ScopedRenderTarget rtBloomEmissive(DXGI_FORMAT_R16G16B16A16_FLOAT);

  _cbComposite.ps0.tonemap =
      vec4(_settings.tonemap.exposure, _settings.tonemap.min_white, _flockFade, 0);

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

    _cbLandscape.Set(_ctx, 0);

    if (_drawFlags & DrawLower)
    {
      _ctx->SetBundle(_landscapeLowerBundle);
      _ctx->DrawIndexed(_numLowerIndices, 0, 0);
    }

    if (_drawFlags & DrawUpper)
    {
      _ctx->SetBundle(_landscapeUpperBundle);
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
  fullscreen->Blur(rtColor, rtColorBlurred, rtColorBlurred._desc, 10, 2);

  ScopedRenderTarget rtEmissiveBlurred(
      rtColor._desc, BufferFlag::CreateSrv | BufferFlag::CreateUav);
  fullscreen->Blur(rtBloomEmissive, rtEmissiveBlurred, rtEmissiveBlurred._desc, 10, 2);

  RenderTargetDesc halfSize(
      rtColor._desc.width / 2, rtColor._desc.height / 2, DXGI_FORMAT_R16G16B16A16_FLOAT);
  ScopedRenderTarget rtScaleBias(halfSize);
  ScopedRenderTarget rtLensFlare(halfSize);

  {
    const LensFlareSettings& s = _settings.lens_flare;

    // lens flare
    fullscreen->ScaleBias(
        rtBloomEmissive, rtScaleBias, rtScaleBias._desc, s.scale_bias.scale, s.scale_bias.bias);

    _cbLensFlare.ps0.params = vec4(s.dispersion, (float)s.num_ghosts, s.halo_width, s.strength);
    _cbLensFlare.Set(_ctx, 0);

    fullscreen->Execute(rtScaleBias,
        rtLensFlare,
        rtLensFlare._desc,
        ObjectHandle(),
        _lensFlareBundle.objects._ps,
        false);
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
          g_Graphics->GetBackBuffer(),
          g_Graphics->GetBackBufferDesc(),
          g_Graphics->GetDepthStencil(),
          _compositeBundle.objects._ps,
          false);
    }
    else if (showBuffer == 1)
    {
      fullscreen->Copy(
          rtColorBlurred, g_Graphics->GetBackBuffer(), g_Graphics->GetBackBufferDesc(), false);
    }
    else if (showBuffer == 2)
    {
      fullscreen->Copy(
          rtEmissiveBlurred, g_Graphics->GetBackBuffer(), g_Graphics->GetBackBufferDesc(), false);
    }
    else
    {
      fullscreen->Copy(rtLensFlare, g_Graphics->GetBackBuffer(), g_Graphics->GetBackBufferDesc(), false);
   } 
  }

#if DEBUG_DRAW_PATH
  for (int i = 0; i < _spline._controlPoints.size() - 1; ++i)
  {
    vec3 p0(_spline._controlPoints[i].x, _spline._controlPoints[i].y, _spline._controlPoints[i].z);
    vec3 p1(_spline._controlPoints[i + 1].x,
        _spline._controlPoints[i + 1].y,
        _spline._controlPoints[i + 1].z);
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
    ImGui::InputInt("NumFlocks", &_settings.boids.num_flocks);
    ImGui::InputInt("BoidsPerFlock", &_settings.boids.boids_per_flock);
    bool newWeights = false;
    newWeights |= ImGui::SliderFloat("Separation", &_settings.boids.separation_scale, 0, 100);
    newWeights |= ImGui::SliderFloat("Cohension", &_settings.boids.cohesion_scale, 0, 100);
    newWeights |= ImGui::SliderFloat("Alignment", &_settings.boids.alignment_scale, 0, 100);
    newWeights |= ImGui::SliderFloat("Seek", &_settings.boids.wander_scale, 0, 100);
    newWeights |= ImGui::SliderFloat("Landscape Follow", &_settings.boids.follow_scale, 0, 100);

    if (newWeights)
    {
      BoidSettings& b = _settings.boids;
      float sum = b.wander_scale + b.separation_scale + b.cohesion_scale + b.alignment_scale
                  + b.follow_scale;
      UpdateWeight(_behaviorSeparataion, _settings.boids.separation_scale / sum);
      UpdateWeight(_behaviorCohesion, _settings.boids.cohesion_scale / sum);
      for (Flock* flock : _flocks)
        UpdateWeight(flock->seek, _settings.boids.wander_scale / sum);
      UpdateWeight(_behaviorLandscapeFollow, _settings.boids.follow_scale / sum);
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
  g_DemoEngine->RegisterFactory(Name(), Landscape::Create);
}

//------------------------------------------------------------------------------
void Landscape::FlockCamera::Update(float deltaTime)
{
  //vec3 targetPos = flock->seek->target;
  vec3 targetPos = flock->boids._center;
  vec3 curPos = _pos;

  vec3 dir = Normalize(targetPos - curPos);
  vec3 targetVel = 10 * dir;

  vec3 cc(curPos.x, curPos.y, curPos.z);
  vec3 vv(targetVel.x, targetVel.y, targetVel.z);
  vec3 tt(targetPos.x, targetPos.y, targetPos.z);

  SmoothDriver::DriveCubic(&cc, &vv, &tt, &vv, 50, deltaTime);

  _pos = cc;
  _dir = Normalize(vv);

  Camera::Update(deltaTime);
}
