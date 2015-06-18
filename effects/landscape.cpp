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
#include "../scheduler.hpp"
#include "../arena_allocator.hpp"
#include "../perlin2d.hpp"

using namespace tano;
using namespace tano::scheduler;
using namespace bristol;

static const Vector3 ZERO3(0,0,0);
static const float GRID_SIZE = 10;

//------------------------------------------------------------------------------
float NoiseAtPoint(const Vector3& v)
{
  static const Vector3 NOISE_SCALE(10.f, 30.f, 10.f);
  static const Vector3 size(10 * 1024, 10 * 1024, 10 * 1024);
  return NOISE_SCALE.y * Perlin2D::Value(256 * v.x / size.x, 256 * v.z / size.z);
}

//------------------------------------------------------------------------------
LandscapeOverlay::LandscapeOverlay()
{
  memset(data, 0, sizeof(data));
}

//------------------------------------------------------------------------------
void LandscapeOverlay::Create(int x, int y, float amp, int size)
{
  for (int i = 0; i < size; ++i)
  {
    for (int j = 0; j < size; ++j)
    {
      int xOfs = x - size/2 + j;
      int yOfs = y - size/2 + i;
      if (xOfs < 0 || xOfs >= SIZE || yOfs < 0 || yOfs >= SIZE)
        continue;

      float dx = (float)(x - xOfs);
      float dy = (float)(y - yOfs);
      float r = Clamp(1.f, 10.f, sqrtf(dx*dx+dy*dy));
      data[0][yOfs*SIZE+xOfs] += amp / (r*r);
    }
  }
}

//------------------------------------------------------------------------------
void LandscapeOverlay::BlurLine(float* x, float scale, int m, float alpha, float* y)
{
  float* buffers[] = { x, scratch, scratch, x, x, y };

  for (int b = 0; b < 3; ++b)
  {
    float* src = buffers[b*2+0];
    float* dst = buffers[b*2+1];

    // set up initial pixel
    float sum = src[0];
    for (int i = 0; i < m; ++i)
      sum += src[i];
    sum += alpha * src[m];

    // note, the final pass is written transposed
    int dstInc = b < 2 ? 1 : SIZE;
    for (int i = 0; i < SIZE; ++i)
    {
      *dst = scale * sum;
      dst += dstInc;
      if (i + m + 2 < SIZE)
        sum += lerp(src[i + m + 1], src[i + m + 2], alpha);

      if (i - m - 1 > 0)
        sum -= lerp(src[i - m], src[i - m - 1], alpha);
    }
  }
}

//------------------------------------------------------------------------------
void LandscapeOverlay::Update()
{
  // Blur the data
  float r = 5;
  float scale = 1.f / (2.0f * r + 1.f);
  int m = (int)r;
  float alpha = r - m;

  // horizontal pass
  for (int i = 0; i < SIZE; ++i)
  {
    BlurLine(&data[0][i*SIZE], scale, m, alpha, &data[1][i]);
  }

  // vertical pass
  for (int i = 0; i < SIZE; ++i)
  {
    BlurLine(&data[1][i*SIZE], scale, m, alpha, &data[0][i]);
  }
}

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

    INIT(_landscapeGpuObjects.LoadShadersFromFile("shaders/out/landscape",
      "VsLandscape", "GsLandscape", "PsLandscape", vertexFlags));

    INIT(_landscapeState.Create(nullptr, &blendDescBlendSrcAlpha, &rasterizeDescCullNone));
    INIT(_landscapeLowerState.Create());
  }

  INIT(_skyBundle.Create(BundleOptions()
    .DepthStencilDesc(depthDescDepthDisabled)
    .ShaderFile("shaders/out/landscape")
    .VsEntry("VsQuad")
    .PsEntry("PsSky")));

  INIT(_copyBundle.Create(BundleOptions()
    .ShaderFile("shaders/out/landscape")
    .VsEntry("VsQuad")
    .PsEntry("PsCopy")));

  INIT(_addBundle.Create(BundleOptions()
    .ShaderFile("shaders/out/landscape")
    .VsEntry("VsQuad")
    .PsEntry("PsAdd")));

  INIT(_compositeBundle.Create(BundleOptions()
    .ShaderFile("shaders/out/landscape")
    .VsEntry("VsQuad")
    .PsEntry("PsComposite")));

  INIT(_luminanceBundle.Create(BundleOptions()
    .ShaderFile("shaders/out/landscape")
    .VsEntry("VsQuad")
    .PsEntry("PsHighPassFilter")));

  // Particles
  INIT_RESOURCE(_particleTexture, RESOURCE_MANAGER.LoadTexture(_settings.particle_texture.c_str()));

  INIT(_particleBundle.Create(BundleOptions()
    .DynamicVb(1024 * 1024 * 6, sizeof(Vector3))
    .ShaderFile("shaders/out/landscape")
    .VsEntry("VsParticle")
    .GsEntry("GsParticle")
    .PsEntry("PsParticle")
    .VertexFlags(VF_POS)
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_POINTLIST)
    .DepthStencilDesc(depthDescDepthWriteDisabled)
    .BlendDesc(blendDescBlendOneOne)
    .RasterizerDesc(rasterizeDescCullNone)));

  INIT(_blur.Init(_ctx, 10));

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

  if (!_flocks.empty())
  {
    if (g_KeyUpTrigger.IsTriggered('1'))
      _followFlock = (_followFlock + 1) % _flocks.size();

    if (g_KeyUpTrigger.IsTriggered('2'))
      _followFlock = (_followFlock - 1) % _flocks.size();

    if (_followFlock != -1 && _followFlock < _flocks.size())
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
  float scaleX = 256.f / (10 * 1024);
  float scaleY = 20;
  float scaleZ = 256.f / (10 * 1024);

  // first compute the noise values
  V3* noise = chunk->noiseValues;
  for (int i = 0; i <= CHUNK_SIZE; ++i)
  {
    for (int j = 0; j <= CHUNK_SIZE; ++j)
    {
      float xx0 = x + (j + 0) * GRID_SIZE;
      float zz0 = z + (i - 1) * GRID_SIZE;
      noise->x = xx0;
      noise->y = scaleY * Perlin2D::Value(scaleX * xx0, scaleZ * zz0);
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
  c.y = 30;
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

  vector<TaskId> chunkTasks;
  vector<Chunk*> chunks;
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
        chunkTasks.push_back(SCHEDULER.AddTask(kd, FillChunk));
      }

      chunks.push_back(chunk);
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

  u32 numChunks = chunkNum == -1 ? (u32)chunks.size() : 1;

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
void Landscape::RenderBoids()
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

//------------------------------------------------------------------------------
bool Landscape::Render()
{
  rmt_ScopedCPUSample(Landscape_Render);
  static Color clearColor(0, 0, 0, 0);
  static const Color* clearColors[] = { &clearColor, &clearColor};
  PostProcess* postProcess = GRAPHICS.GetPostProcess();

  ScopedRenderTargetFull rtColor(
    DXGI_FORMAT_R16G16B16A16_FLOAT,
    BufferFlags(BufferFlag::CreateSrv),
    BufferFlags(BufferFlag::CreateSrv));

  ScopedRenderTarget rtBloom(DXGI_FORMAT_R16G16B16A16_FLOAT);

  _cbPerFrame.world = Matrix::Identity();

  _cbPerFrame.dim = Vector4((float)rtColor._width, (float)rtColor._height, 0, 0);
  _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::VertexShader, 0);
  _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::GeometryShader, 0);
  _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::PixelShader, 0);

  // We're using 2 render targets here. One for color, and one for depth/bloom

  // Render the sky
  ObjectHandle renderTargets[] = {rtColor._rtHandle, rtBloom._rtHandle};
  _ctx->SetRenderTargets(renderTargets, 2, rtColor._dsHandle, &clearColors[0]);

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
  else
  {
    _ctx->SetGpuState(_landscapeState);
  }

  if (_renderBoids)
  {
    RenderBoids();
  }

  _ctx->UnsetRenderTargets(0, 2);

  ScopedRenderTarget rtBloomBlurred(DXGI_FORMAT_R16G16B16A16_FLOAT,
    BufferFlags(BufferFlag::CreateSrv | BufferFlag::CreateUav));

  _blur.Apply(rtBloom, rtBloomBlurred, 10);

  //ScopedRenderTarget rtColorAndBloom(DXGI_FORMAT_R16G16B16A16_FLOAT);
  //{
  //  ObjectHandle inputs[] = { rtColor, rtBloomBlurred };
  //  postProcess->Execute(inputs, 2, rtColorAndBloom, ObjectHandle(), _addBundle.objects._ps);
  //}

  //ScopedRenderTarget rtColorAndBloomBlurred(DXGI_FORMAT_R16G16B16A16_FLOAT,
  //  BufferFlags(BufferFlag::CreateSrv | BufferFlag::CreateUav));

  //_blur.Apply(rtColorAndBloom, rtColorAndBloomBlurred, 10);

  static bool showBlurred = false;
  if (g_KeyUpTrigger.IsTriggered('B'))
    showBlurred = !showBlurred;

  if (showBlurred)
  {
    postProcess->Execute(
      rtBloomBlurred,
      GRAPHICS.GetBackBuffer(),
      ObjectHandle(),
      _copyBundle.objects._ps,
      false);
  }
  else
  {
    //ObjectHandle inputs[] = { rtColorAndBloom, rtColorAndBloomBlurred, rtColor._dsHandle };
    ObjectHandle inputs[] = { rtColor, rtBloomBlurred };
    postProcess->Execute(
      inputs,
      2,
      GRAPHICS.GetBackBuffer(),
      GRAPHICS.GetDepthStencil(),
      _compositeBundle.objects._ps,
      false);
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
