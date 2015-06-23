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

using namespace tano;
using namespace tano::scheduler;
using namespace bristol;

static const Vector3 ZERO3(0,0,0);
static const float GRID_SIZE = 10;
static const float NOISE_HEIGHT = 50;
static const float NOISE_SCALE_X = 0.01f;
static const float NOISE_SCALE_Z = 0.01f;

int Landscape::Chunk::nextId = 1;

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
  _freeflyCamera._pitch = _settings.camera.pitch;
  _freeflyCamera._yaw = _settings.camera.yaw;
  _freeflyCamera._roll = _settings.camera.roll;
  _freeflyCamera._pos = _settings.camera.pos;

  {
    // Landscape
    u32 vertexFlags = VF_POS | VF_NORMAL;
    u32 vertexSize = sizeof(PosNormal);
    INIT(_landscapeGpuObjects.CreateDynamicVb(1024 * 1024 * 6 * vertexSize, vertexSize));

    u32 ibSize;
    u32 maxQuads = 1024 * 1024;
    u32* indices = GenerateQuadIndices(maxQuads, &ibSize);
    INIT(_landscapeGpuObjects.CreateIndexBuffer(ibSize, DXGI_FORMAT_R32_UINT, indices));

    INIT(_landscapeGpuObjects.LoadVertexShader("shaders/out/landscape", "VsLandscape", vertexFlags));
    INIT(_landscapeGpuObjects.LoadGeometryShader("shaders/out/landscape", "GsLandscape"));
    INIT(_landscapeGpuObjects.LoadPixelShader("shaders/out/landscape", "PsLandscape"));

    INIT(_landscapeState.Create(nullptr, &blendDescBlendSrcAlpha, &rasterizeDescCullNone));
    INIT(_landscapeLowerState.Create());
  }

  INIT(_skyBundle.Create(BundleOptions()
    .DepthStencilDesc(depthDescDepthDisabled)
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/landscape", "PsSky")));

  INIT(_compositeBundle.Create(BundleOptions()
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/landscape", "PsComposite")));

  INIT(_luminanceBundle.Create(BundleOptions()
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/landscape", "PsHighPassFilter")));

  INIT(_lensFlareBundle.Create(BundleOptions()
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/landscape", "PsLensFlare")));

  INIT(_cbLensFlare.Create());

  // Particles
  INIT_RESOURCE(_particleTexture, RESOURCE_MANAGER.LoadTexture(_settings.particle_texture.c_str()));

  INIT(_particleBundle.Create(BundleOptions()
    .DynamicVb(1024 * 1024 * 6, sizeof(Vector3))
    .VertexShader("shaders/out/landscape", "VsParticle")
    .GeometryShader("shaders/out/landscape", "GsParticle")
    .PixelShader("shaders/out/landscape", "PsParticle")
    .VertexFlags(VF_POS)
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_POINTLIST)
    .DepthStencilDesc(depthDescDepthWriteDisabled)
    .BlendDesc(blendDescBlendOneOne)
    .RasterizerDesc(rasterizeDescCullNone)));

  INIT(_boidsBundle.Create(BundleOptions()
    .DynamicVb(1024 * 1024 * 6, sizeof(Vector3))
    .VertexShader("shaders/out/landscape", "VsParticle")
    .GeometryShader("shaders/out/landscape", "GsParticle")
    .PixelShader("shaders/out/landscape", "PsParticle")
    .VertexFlags(VF_POS)
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_POINTLIST)
    .DepthStencilDesc(depthDescDepthWriteDisabled)
    .BlendDesc(blendDescBlendOneOne)
    .RasterizerDesc(rasterizeDescCullNone)));

  Reset();

  INIT(_cbPerFrame.Create());

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

  for (int i = 0; i < _settings.boids.num_flocks; ++i)
  {
    Flock* flock = new Flock(_settings.boids);
    flock->boids._maxSpeed = b.max_speed;

    float sum = 
      b.wander_scale + b.separation_scale + b.cohesion_scale + b.alignment_scale + b.follow_scale;
    // Each flock gets its own seek behavior, because they need per flock information
    flock->boids.AddKinematics(flock->seek, _settings.boids.wander_scale / sum);
    flock->boids.AddKinematics(_behaviorSeparataion, _settings.boids.separation_scale / sum);
    flock->boids.AddKinematics(_behaviorCohesion, _settings.boids.cohesion_scale / sum);
    flock->boids.AddKinematics(_behaviorAlignment, _settings.boids.alignment_scale / sum);
    flock->boids.AddKinematics(_landscapeFollow, _settings.boids.follow_scale / sum);

    float s = 200.f;
    V3 center(randf(-s, s), 0, randf(-s, s));

    // Create a waypoint for the flock
    float angle = randf(-XM_PI, XM_PI);
    float dist = randf(100.f, 200.f);
    flock->nextWaypoint = center + V3(dist * cos(angle), 0, dist * sin(angle));
    flock->nextWaypoint.y = 20 + NoiseAtPoint(flock->nextWaypoint);
    flock->wanderAngle = angle;

    flock->seek->target = flock->nextWaypoint;

    // Init the boids
    V3* pos = flock->boids._bodies.pos;
    V3* force = flock->boids._bodies.force;
    for (int i = 0; i < flock->boids._bodies.numBodies; ++i)
    {
      //DynParticles::Body& b = flock->boids._bodies[i];
      pos[i] = center + V3(randf(-20.f, 20.f), 0, randf(-20.f, 20.f));
      pos[i].y = 20 + NoiseAtPoint(pos[i]);
      force[i] = 10 * V3(randf(-20.f, 20.f), 0, randf(-20.f, 20.f));
    }

    _flocks.Append(flock);
  }
}

//------------------------------------------------------------------------------
void BehaviorLandscapeFollow::Update(
  DynParticles::Bodies* bodies, float weight, const UpdateState& state)
{
  V3* pos = bodies->pos;
  V3* acc = bodies->acc;
  V3* vel = bodies->vel;
  V3* force = bodies->force;
  DynParticles::DistMatrix* dm = bodies->distMatrix;
  int numBodies = bodies->numBodies;

  for (int i = 0; i < numBodies; ++i)
  {
    // return a force to keep the boid above the ground

    V3 probe = pos[i] + Normalize(vel[i]) * maxSpeed;
    V3 d = probe;
    d.y = 20 + NoiseAtPoint(probe);

    // if the probe is above the terrain height, move towards the average
    if (probe.y > d.y)
      d = 0.5f * (probe + d);

    V3 desiredVel = Normalize(d - probe) * maxSpeed;
    force[i] += ClampVector(desiredVel - vel[i], maxForce);
  }
}

//------------------------------------------------------------------------------
void Landscape::UpdateFlock(const scheduler::TaskData& data)
{
  FlockKernelData* flockData = (FlockKernelData*)data.kernelData.data;
  Flock* flock = flockData->flock;
  float radius = flockData->waypointRadius;

  V3* pos = flockData->flock->boids._bodies.pos;
  int numBodies = flockData->flock->boids._bodies.numBodies;

  // check if the flock has reached its waypoint
  float closestDist = FLT_MAX;
  Vector3 center(0, 0, 0);
  for (int i = 0; i < numBodies; ++i)
  {
    if (Distance(pos[i], flock->nextWaypoint) < radius)
    {
      float angle = flock->wanderAngle + randf(-XM_PI / 2, XM_PI / 2);
      float dist = randf(300.f, 400.f);
      flock->nextWaypoint += V3(dist * cos(angle), 0, dist * sin(angle));
      flock->nextWaypoint.y = 20 + NoiseAtPoint(flock->nextWaypoint);
      flock->wanderAngle = angle;
      break;
    }
  }

  flock->seek->target = flock->nextWaypoint;
  flock->boids.Update(flockData->updateState);
}

//------------------------------------------------------------------------------
void Landscape::UpdateBoids(const UpdateState& state)
{
  rmt_ScopedCPUSample(Boids_Update);
  
  static AvgStopWatch stopWatch;
  stopWatch.Start();

  float dt = 1.0f / state.frequency;

  SimpleAppendBuffer<TaskId, 2048> chunkTasks;

  for (Flock* flock : _flocks)
  {
    FlockKernelData* data = (FlockKernelData*)ARENA.Alloc(sizeof(FlockKernelData));
    *data = FlockKernelData{ flock, _settings.boids.waypoint_radius, state };
    KernelData kd;
    kd.data = data;
    kd.size = sizeof(FlockKernelData);
    chunkTasks.Append(SCHEDULER.AddTask(kd, UpdateFlock));
  }

  for (const TaskId& taskId : chunkTasks)
    SCHEDULER.Wait(taskId);

  double avg = stopWatch.Stop();
  TANO.AddPerfCallback([=]() {
    ImGui::Text("Update time: %.3fms", 1000 * avg);
  });

}

//------------------------------------------------------------------------------
bool Landscape::Update(const UpdateState& state)
{
  _cbPerFrame.time.x = (float)(state.localTime.TotalMilliseconds() / 1e6);
  _cbPerFrame.time.y = _flocks[0]->boids._center.x;
  _cbPerFrame.time.z = _flocks[0]->boids._center.y;
  _cbPerFrame.time.w = _flocks[0]->boids._center.z;
  UpdateBoids(state);
  UpdateCameraMatrix(state);
  return true;
}

//------------------------------------------------------------------------------
void Landscape::UpdateCameraMatrix(const UpdateState& state)
{
  const IoState& ioState = TANO.GetIoState();

  if (!_flocks.Empty())
  {
    if (g_KeyUpTrigger.IsTriggered('1'))
      _followFlock = (_followFlock + 1) % _flocks.Size();

    if (g_KeyUpTrigger.IsTriggered('2'))
      _followFlock = (_followFlock - 1) % _flocks.Size();

    if (_followFlock != -1 && _followFlock < _flocks.Size())
    {
      _followCamera.SetFollowTarget(_flocks[_followFlock]->boids._center);
    }
  }

  if (g_KeyUpTrigger.IsTriggered('7'))
    _drawFlags ^= 0x1;

  if (g_KeyUpTrigger.IsTriggered('8'))
    _drawFlags ^= 0x2;

  if (g_KeyUpTrigger.IsTriggered('9'))
    _drawFlags ^= 0x4;

  if (_useFreeFlyCamera || _flocks.Empty())
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

  // The depth value written to the z-buffer is after the w-divide,
  // but the z-value we compare against is still in proj-space, so
  // we'll need to do the backward transform:
  // f*(z-n) / (f-n)*z = zbuf => z = f*n / (f-zbuf(f-n))
  float n = _curCamera->_nearPlane;
  float f = _curCamera->_farPlane;
  _cbPerFrame.nearFar = Vector4(n, f, f*n, f-n);

  DEBUG_API.SetTransform(Matrix::Identity(), viewProj);
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

  int layerIncr[] = { 1, 2 };
  float* layerDest[] = { chunk->lowerData, chunk->upperData };
  float layerScale[] = { 0.5f, 1.0f };
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

        v0 = chunk->noiseValues[(i + 0)     * (CHUNK_SIZE + 1) + (j + 0)];
        v1 = chunk->noiseValues[(i + incr)  * (CHUNK_SIZE + 1) + (j + 0)];
        v2 = chunk->noiseValues[(i + incr)  * (CHUNK_SIZE + 1) + (j + incr)];
        v3 = chunk->noiseValues[(i + 0)     * (CHUNK_SIZE + 1) + (j + incr)];

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
        ChunkKernelData* data = (ChunkKernelData*)ARENA.Alloc(sizeof(ChunkKernelData));
        *data = ChunkKernelData{ chunk, x, z };
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

  sort(chunks.begin(), chunks.end(), [&](const Chunk* a, const Chunk* b) {
    return a->dist > b->dist;
  });

  // copy all the chunk data into the vertex buffer
  float* landscapeBuf = _ctx->MapWriteDiscard<float>(_landscapeGpuObjects._vb);
  V3* particleBuf = _ctx->MapWriteDiscard<V3>(_particleBundle.objects._vb);

  static int chunkNum = -1;
#if 0
  if (g_KeyUpTrigger.IsTriggered('4'))
  {
    if (chunkNum > -1)
      --chunkNum;
  }

  if (g_KeyUpTrigger.IsTriggered('5'))
    ++chunkNum;
#endif

  u32 numParticles = 0;
  int idx = 0;
  for (const Chunk* chunk : chunks)
  {
    if (chunkNum != -1)
    {
      idx++;
      if (idx != chunkNum)
        continue;
    }
    idx++;

    memcpy(landscapeBuf, chunk->upperData, Chunk::UPPER_DATA_SIZE * sizeof(float));
    landscapeBuf += Chunk::UPPER_DATA_SIZE;

    for (int i = 0; i < CHUNK_SIZE; i += 2)
    {
      for (int j = 0; j < CHUNK_SIZE; j += 2)
      {
        *particleBuf++ = chunk->noiseValues[i*(CHUNK_SIZE+1)+j];
        numParticles++;
      }
    }
  }

  u32 numChunks = chunkNum == -1 ? (u32)chunks.Size() : 1;

  _numUpperIndices = numChunks * Chunk::UPPER_INDICES;
  _numParticles = numParticles;

  for (const Chunk* chunk : chunks)
  {
    memcpy(landscapeBuf, chunk->lowerData, Chunk::LOWER_DATA_SIZE * sizeof(float));
    landscapeBuf += Chunk::LOWER_DATA_SIZE;
  }
  _numLowerIndices = numChunks * Chunk::LOWER_INDICES;

  TANO.AddPerfCallback([=]() {
    ImGui::Text("# particles: %d", numParticles);
    ImGui::Text("# chunks: %d", numChunks);
  });

  _ctx->Unmap(_particleBundle.objects._vb);
  _ctx->Unmap(_landscapeGpuObjects._vb);
}

//------------------------------------------------------------------------------
void Landscape::RenderBoids(const ObjectHandle* renderTargets, ObjectHandle dsHandle)
{
  V3* boidPos = _ctx->MapWriteDiscard<V3>(_boidsBundle.objects._vb);

  int numBoids = 0;
  for (const Flock* flock : _flocks)
  {
    //DEBUG_API.AddDebugLine(flock->boids._center, flock->nextWaypoint, Color(1, 1, 1));
    //DEBUG_API.AddDebugSphere(flock->nextWaypoint, 10, Color(1, 1, 1));

    V3* pos = flock->boids._bodies.pos;
    int numBodies = flock->boids._bodies.numBodies;

    for (int i = 0; i < numBodies; ++i)
    {
      *boidPos++ = pos[i];
      numBoids++;
    }
  }

  _ctx->Unmap(_boidsBundle.objects._vb);

  _ctx->SetBundleWithSamplers(_boidsBundle, ShaderType::PixelShader);

  // Unset the DSV, as we want to use it as a texture resource
  _ctx->SetRenderTargets(renderTargets, 2, ObjectHandle(), nullptr);
  ObjectHandle srv[] = { _particleTexture, dsHandle };
  _ctx->SetShaderResources(srv, 2, ShaderType::PixelShader);
  _ctx->Draw(numBoids, 0);
  _ctx->UnsetShaderResources(0, 2, ShaderType::PixelShader);
}

//------------------------------------------------------------------------------
bool Landscape::Render()
{
  rmt_ScopedCPUSample(Landscape_Render);
  static Color clearColor(0, 0, 0, 0);
  static const Color* clearColors[] = { &clearColor, &clearColor};
  FullscreenEffect* fullscreen = GRAPHICS.GetFullscreenEffect();

  ScopedRenderTargetFull rtColor(DXGI_FORMAT_R16G16B16A16_FLOAT, BufferFlag::CreateSrv, BufferFlag::CreateSrv);
  ScopedRenderTarget rtBloom(DXGI_FORMAT_R16G16B16A16_FLOAT);

  _cbPerFrame.toneMappingParams = Vector4(_settings.tonemap.shoulder, _settings.tonemap.max_white, 0, 0);
  _cbPerFrame.world = Matrix::Identity();
  _cbPerFrame.dim = Vector4((float)rtColor._desc.width, (float)rtColor._desc.height, 0, 0);
  u32 flags = ShaderType::VertexShader | ShaderType::GeometryShader | ShaderType::PixelShader;
  _ctx->SetConstantBuffer(_cbPerFrame, flags, 0);

  // We're using 2 render targets here. One for color, and one for bloom/emissive
  ObjectHandle renderTargets[] = {rtColor, rtBloom};
  _ctx->SetRenderTargets(renderTargets, 2, rtColor._dsHandle, &clearColors[0]);

  // Render the sky
  _ctx->SetBundle(_skyBundle);
  _ctx->Draw(3, 0);

  if (_renderLandscape)
  {
    RasterizeLandscape();

    _ctx->SetGpuObjects(_landscapeGpuObjects);

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

    if (_drawFlags & DrawParticles)
    {
      _ctx->SetBundleWithSamplers(_particleBundle, ShaderType::PixelShader);

      // Unset the DSV, as we want to use it as a texture resource
      _ctx->SetRenderTargets(renderTargets, 2, ObjectHandle(), nullptr);
      ObjectHandle srv[] = { _particleTexture, rtColor._dsHandle };
      _ctx->SetShaderResources(srv, 2, ShaderType::PixelShader);
      _ctx->Draw(_numParticles, 0);
      _ctx->UnsetShaderResources(0, 2, ShaderType::PixelShader);
    }
  }

  if (_renderBoids)
  {
    RenderBoids(renderTargets, rtColor._dsHandle);
  }

  _ctx->UnsetRenderTargets(0, 2);

  RenderTargetDesc halfSize(rtColor._desc.width / 2, rtColor._desc.height / 2, DXGI_FORMAT_R16G16B16A16_FLOAT);

  ScopedRenderTarget rtBloomBlurred(rtColor._desc, BufferFlag::CreateSrv | BufferFlag::CreateUav);
  fullscreen->Blur(rtBloom, rtBloomBlurred, 10);

  ScopedRenderTarget rtScaleBias(halfSize);

  fullscreen->ScaleBiasSecondary(
    rtColor,
    rtBloom,
    rtScaleBias,
    rtScaleBias._desc,
    _settings.lens_flare.scale_bias.scale,
    _settings.lens_flare.scale_bias.bias);

  ScopedRenderTarget rtLensFlare(halfSize);

  const LensFlareSettings& s = _settings.lens_flare;
  _cbLensFlare.params = Vector4(s.dispersion, (float)s.num_ghosts, s.halo_width, s.strength);
  _ctx->SetConstantBuffer(_cbLensFlare, ShaderType::PixelShader, 1);

  fullscreen->Execute(
    rtScaleBias,
    rtLensFlare,
    rtLensFlare._desc,
    ObjectHandle(),
    _lensFlareBundle.objects._ps,
    false);

  static int showBuffer = 0;
  if (g_KeyUpTrigger.IsTriggered('B'))
    showBuffer = (showBuffer + 1) % 3;

  if (showBuffer == 0)
  {
    ObjectHandle inputs[] = { rtColor, rtBloomBlurred, rtLensFlare };
    fullscreen->Execute(
      inputs,
      3,
      GRAPHICS.GetBackBuffer(),
      GRAPHICS.GetBackBufferDesc(),
      GRAPHICS.GetDepthStencil(),
      _compositeBundle.objects._ps,
      false);
  }
  else if (showBuffer == 1)
  {
    fullscreen->Copy(rtScaleBias, GRAPHICS.GetBackBuffer(), GRAPHICS.GetBackBufferDesc(), false);
  }
  else
  {
    fullscreen->Copy(rtLensFlare, GRAPHICS.GetBackBuffer(), GRAPHICS.GetBackBufferDesc(), false);
  }

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

  ImGui::SliderFloat("dispersion", &_settings.lens_flare.dispersion, 0, 2);
  ImGui::SliderInt("Num ghosts", &_settings.lens_flare.num_ghosts, 1, 10);
  ImGui::SliderFloat("Halo width", &_settings.lens_flare.halo_width, 0, 3);
  ImGui::SliderFloat("Strength", &_settings.lens_flare.strength, 0, 1);

  ImGui::SliderFloat("scale", &_settings.lens_flare.scale_bias.scale, 0, 3);
  ImGui::SliderFloat("bias", &_settings.lens_flare.scale_bias.bias, 0, 1);
  ImGui::Separator();

  ImGui::SliderFloat("Shoulder", &_settings.tonemap.shoulder, 0, 1);
  ImGui::SliderFloat("Max White", &_settings.tonemap.max_white, 0.5f, 10);
  ImGui::Separator();

  ImGui::Checkbox("Render landscape", &_renderLandscape);
  ImGui::Checkbox("Render boids", &_renderBoids);
  ImGui::InputInt("NumVerts", (int*)&_numUpperIndices);
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
  _freeflyCamera._pos = Vector3(0.f, 10.f, 30.f);
  _freeflyCamera._pitch = 0.f;
  _freeflyCamera._yaw = 0.f;
  _freeflyCamera._roll = 0.f;
  //_freeflyCamera._yaw = XM_PI;

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
