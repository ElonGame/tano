#include "particle_tunnel.hpp"
#include "../tano.hpp"
#include "../graphics.hpp"
#include "../graphics_extra.hpp"
#include "../graphics_context.hpp"
#include "../demo_engine.hpp"
#include "../resource_manager.hpp"
#include "../init_sequence.hpp"
#include "../generated/demo.parse.hpp"
#include "../generated/input_buffer.hpp"
#include "../generated/output_buffer.hpp"
#include "../mesh_loader.hpp"
#include "../fullscreen_effect.hpp"
#include "../scheduler.hpp"
#include "../arena_allocator.hpp"
#include "../stop_watch.hpp"

using namespace tano;
using namespace tano::scheduler;
using namespace bristol;

namespace
{
  bool wireframe = false;
  float angle = 0;
  float height = 0;
  float distance = 1300;
  bool extended = false;
}

//------------------------------------------------------------------------------
void ParticleTunnel::ParticleEmitter::Create(const V3& center, int numParticles)
{
  _center = center;
  _numParticles = numParticles;
  x = new float[_numParticles];
  y = new float[_numParticles];
  z = new float[_numParticles];

  vx = new float[_numParticles];
  vy = new float[_numParticles];
  vz = new float[_numParticles];

  _lifetime = new Lifetime[_numParticles];
  scale = new float[_numParticles];
  _deadParticles = new int[_numParticles];

  float s = 100;
  for (int i = 0; i < _numParticles; ++i)
  {
    CreateParticle(i, s);
  }

  for (int i = 0; i < 1000; ++i)
  {
    Update(0.01f);
  }
}

//------------------------------------------------------------------------------
void ParticleTunnel::ParticleEmitter::CreateParticle(int idx, float s)
{
  x[idx] = _center.x + randf(-s, s);
  y[idx] = _center.y + randf(-s, s);
  z[idx] = _center.z + randf(1500.f, 2000.f);

  vx[idx] = randf(-s, s);
  vy[idx] = randf(-s, s);
  vz[idx] = -randf(10.f, 200.f);

  int ll = randf(2000, 3000);
  _lifetime[idx].left = ll;
  _lifetime[idx].total = ll;
  scale[idx] = randf(1.f, 5.f);
}

//------------------------------------------------------------------------------
void ParticleTunnel::ParticleEmitter::Destroy()
{
  SAFE_ADELETE(x);  SAFE_ADELETE(y);  SAFE_ADELETE(z);
  SAFE_ADELETE(vx); SAFE_ADELETE(vy); SAFE_ADELETE(vz);
  SAFE_ADELETE(_lifetime);
  SAFE_ADELETE(scale);
  SAFE_ADELETE(_deadParticles);
}

//------------------------------------------------------------------------------
void ParticleTunnel::ParticleEmitter::Update(float dt)
{
  float* xx = x;
  float* yy = y;
  float* zz = z;

  float *vvx = vx;
  float *vvy = vy;
  float *vvz = vz;

  Lifetime* ll = _lifetime;

  int numDead = 0;
  int* dead = _deadParticles;

  float s = 100;
  for (int i = 0, e = _numParticles; i < e; ++i)
  {
    *xx += dt * *vvx;
    *yy += dt * *vvy;
    *zz += dt * *vvz;
    --ll->left;

    if (*zz < -1500)
    {
      *dead++ = i;
    }

    xx++; yy++; zz++;
    vvx++; vvy++; vvz++;
    ll++;
  }

  for (int i = 0; i < numDead; ++i)
  {
    CreateParticle(_deadParticles[i], s);
  }
}

void ParticleTunnel::ParticleEmitter::CopyToBuffer(ParticleType* vtx)
{
  float* xx = x;
  float* yy = y;
  float* zz = z;

  float* ss = scale;
  ParticleEmitter::Lifetime* ll = _lifetime;

  for (int i = 0, e = _numParticles; i < e; ++i)
  {
    float s = 10.f * ss[i];

    float lifetime = (float)ll[i].left / ll[i].total;
    vtx->pos = V4{xx[i], yy[i], zz[i], 0};
    vtx->data = V4{lifetime, 0, 0, 0};
    vtx++;
  }
}

//------------------------------------------------------------------------------
void ParticleTunnel::TextParticles::Create(
    const vector<V3>& verts,
    float targetTime)
{
  numParticles = (int)verts.size();
  if (!curX)
  {
    curX = new float[numParticles];
    curY = new float[numParticles];
    curZ = new float[numParticles];

    startX = new float[numParticles]; 
    startY = new float[numParticles];
    startZ = new float[numParticles];

    endX = new float[numParticles];
    endY = new float[numParticles];
    endZ = new float[numParticles];
  }

  // note, everything is done on a per triangle basis, and not per vertex
  int numTris = (int)verts.size() / 3;
  int idx = 0;
  for (int i = 0; i < numTris; ++i)
  {
    //float dist = randf(settings.text_min_dist, settings.text_max_dist);

    for (int j = 0; j < 3; ++j)
    {
      float x = verts[idx].x;
      float y = verts[idx].y;
      float s = (x + y) / 50;

      float dist = settings.text_max_dist * (2 + cos(s) * sin(s));

      //    Vector3 dir(randf(-1.f, 1.f), randf(-1.f, 1.f), randf(0.f, 1.f));
      // project each vertex outwards along the hemisphere in the z-plane
      Vector3 dir((float)(0.1 * sin(s)), (float)(0.1 * cos(s)), 1);
      dir.Normalize();

      startX[idx] = verts[idx].x - dir.x * dist;
      startY[idx] = verts[idx].y - dir.y * dist;
      startZ[idx] = verts[idx].z - dir.z * dist;

      endX[idx] = verts[idx].x;
      endY[idx] = verts[idx].y;
      endZ[idx] = verts[idx].z;

      ++idx;
    }
  }

  selectedTris.clear();

  selectedTris.reserve(numTris);
  for (int i = 0; i < numTris; ++i)
  {
    float p = randf(0.f, 1.f);
    if (p > settings.text_triangle_prob)
      selectedTris.push_back(i);
  }
}

//------------------------------------------------------------------------------
void ParticleTunnel::TextParticles::Destroy()
{
  SAFE_ADELETE(curX); SAFE_ADELETE(curY); SAFE_ADELETE(curZ);
  SAFE_ADELETE(startX);  SAFE_ADELETE(startY);  SAFE_ADELETE(startZ);
  SAFE_ADELETE(endX); SAFE_ADELETE(endY); SAFE_ADELETE(endZ);
}

//------------------------------------------------------------------------------
void ParticleTunnel::TextParticles::Update(float seconds, float start, float end)
{
  float* xx = curX;
  float* yy = curY;
  float* zz = curZ;

  float duration = end - start;
  float s = Clamp(0.f, 1.f, seconds / duration);

  for (int i = 0, e = numParticles; i < e; ++i)
  {
    xx[i] = lerp(startX[i], endX[i], s);
    yy[i] = lerp(startY[i], endY[i], s);
    zz[i] = lerp(startZ[i], endZ[i], s);
  }
}

//------------------------------------------------------------------------------
ParticleTunnel::ParticleTunnel(const string &name, u32 id)
  : BaseEffect(name, id)
  , _neuroticaParticles(_settings)
  , _radioSilenceParticles(_settings)
  , _partyParticles(_settings)
  , _beatTrack("beat")
{
#if WITH_IMGUI
  PROPERTIES.Register("particle tunnel", 
    bind(&ParticleTunnel::RenderParameterSet, this),
    bind(&ParticleTunnel::SaveParameterSet, this));
#endif
}

//------------------------------------------------------------------------------
ParticleTunnel::~ParticleTunnel()
{
  for (int i = 0; i < _particleEmitters.Size(); ++i)
  {
    _particleEmitters[i].Destroy();
  }
}

//------------------------------------------------------------------------------
bool ParticleTunnel::Init(const char* configFile)
{
  BEGIN_INIT_SEQUENCE();

  _configName = configFile;
  AddFileWatchResult res = RESOURCE_MANAGER.AddFileWatch(configFile, true, [this](const string& filename, void*)
  {
    vector<char> buf;
    if (!RESOURCE_MANAGER.LoadFile(filename.c_str(), &buf))
      return false;

    return ParseParticleTunnelSettings(InputBuffer(buf), &_settings);
  });

  // Background state setup
  INIT(_backgroundBundle.Create(BundleOptions()
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/particle_tunnel", "PsBackground")));

  // Particle state setup
  vector<D3D11_INPUT_ELEMENT_DESC> inputs = {
    CD3D11_INPUT_ELEMENT_DESC("POSITION", DXGI_FORMAT_R32G32B32_FLOAT),
    CD3D11_INPUT_ELEMENT_DESC("TEXCOORD", DXGI_FORMAT_R32G32B32_FLOAT) };

  INIT(_particleBundle.Create(BundleOptions()
    .VertexShader("shaders/out/particle_tunnel", "VsParticle")
    .GeometryShader("shaders/out/particle_tunnel", "GsParticle")
    .PixelShader("shaders/out/particle_tunnel", "PsParticle")
    .InputElements(inputs)
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_POINTLIST)
    .DynamicVb(24 * _settings.num_particles, sizeof(ParticleType))
    .DepthStencilDesc(depthDescDepthDisabled)
    .BlendDesc(blendDescPreMultipliedAlpha)
    .RasterizerDesc(rasterizeDescCullNone)));

  INIT_RESOURCE(_particleTexture, RESOURCE_MANAGER.LoadTexture(_settings.texture.c_str()));
  
  // Create default emitter
  for (int i = 0; i < 10; ++i)
  {
    _particleEmitters.Append(ParticleEmitter()).Create(V3(0, 0, 0), _settings.num_particles);
  }
  
  // Composite state setup
  INIT(_compositeBundle.Create(BundleOptions()
    .VertexShader("shaders/out/common", "VsQuad")
    .PixelShader("shaders/out/particle_tunnel", "PsComposite")));

  // Text setup
  INIT(_textWriter.Init("gfx/text1.boba"));
  _textWriter.GenerateTris("neurotica efs", &_neuroticaTris, &_neuroticaCapTris);
  _textWriter.GenerateTris("radio\nsilence", &_radioSilenceTris, &_radioSilenceCapTris);
  _textWriter.GenerateTris("solskogen", &_partyTris, &_partyCapTris);

  _neuroticaParticles.Create(_neuroticaTris, 5.f);
  _radioSilenceParticles.Create(_radioSilenceTris, 5.f);
  _partyParticles.Create(_partyTris, 5.f);

  _curParticles = &_neuroticaParticles;

#if WITH_TEXT
  {
    CD3D11_RASTERIZER_DESC rssDesc = CD3D11_RASTERIZER_DESC(CD3D11_DEFAULT());
    rssDesc.CullMode = D3D11_CULL_NONE;

    CD3D11_DEPTH_STENCIL_DESC dsDesc = CD3D11_DEPTH_STENCIL_DESC(CD3D11_DEFAULT());
    dsDesc.DepthEnable = FALSE;

    // pre-multiplied alpha
    CD3D11_BLEND_DESC blendDesc = CD3D11_BLEND_DESC(CD3D11_DEFAULT());
    D3D11_RENDER_TARGET_BLEND_DESC& b = blendDesc.RenderTarget[0];
    b.BlendEnable = TRUE;
    b.DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
    b.DestBlendAlpha = D3D11_BLEND_INV_SRC_ALPHA;

    INIT(_textState.Create(&dsDesc, &blendDesc, &rssDesc));
  }

  INIT(_textGpuObjects.CreateDynamicVb((u32)_neuroticaTris.size() * sizeof(Vector3), sizeof(Vector3)));
  INIT(_textGpuObjects.LoadShadersFromFile("shaders/out/particle_tunnel", "VsText", nullptr, "PsText", VertexFlags::VF_POS));
#endif

  INIT_RESOURCE(_lineTexture, RESOURCE_MANAGER.LoadTexture("gfx/line.png"));

  INIT(_lineBundle.Create(BundleOptions()
    .RasterizerDesc(rasterizeDescCullNone)
    .DepthStencilDesc(depthDescDepthDisabled)
    .BlendDesc(blendDescPreMultipliedAlpha)
    .DynamicVb((u32)_neuroticaTris.size() * 3 * 2, sizeof(Vector3))
    .VertexShader("shaders/out/particle_tunnel", "VsLines")
    .VertexFlags(VertexFlags::VF_POS)
    .GeometryShader("shaders/out/particle_tunnel", "GsLines")
    .PixelShader("shaders/out/particle_tunnel", "PsLines")
    .Topology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST)));

  // Generic setup
  INIT(_cbPerFrame.Create());
  UpdateCameraMatrix();
  _cbPerFrame.tint = _settings.tint;
  _cbPerFrame.inner = _settings.inner_color;
  _cbPerFrame.outer = _settings.outer_color;

  int w, h;
  GRAPHICS.GetBackBufferSize(&w, &h);
  _cbPerFrame.dim.x = (float)w;
  _cbPerFrame.dim.y = (float)h;

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
void ParticleTunnel::UpdateCameraMatrix()
{
  float x = distance * sin(angle);
  float z = -distance * cos(angle);
  Vector3 pos = Vector3(x, height, z);
  Vector3 target = Vector3(0, 0, 0);
  Vector3 dir = target - pos;
  dir.Normalize();

  Matrix view = Matrix::CreateLookAt(pos, Vector3(0, 0, 0), Vector3(0, 1, 0));
  Matrix proj = Matrix::CreatePerspectiveFieldOfView(XMConvertToRadians(45), 16/10.f, 0.1f, 3000.f);
  Matrix viewProj = view * proj;

  _cbPerFrame.world = Matrix::Identity();
  _cbPerFrame.view = view.Transpose();
  _cbPerFrame.proj = proj.Transpose();
  _cbPerFrame.viewProj = viewProj.Transpose();
  _cbPerFrame.viewDir = Vector4(dir.x, dir.y, dir.z, 1);
  _cbPerFrame.camPos = Vector4(pos.x, pos.y, pos.z, 1);
}

//------------------------------------------------------------------------------
void ParticleTunnel::UpdateEmitter(const TaskData& data)
{
  EmitterKernelData* emitterData = (EmitterKernelData*)data.kernelData.data;
  ParticleEmitter* emitter = emitterData->emitter;

  for (int i = 0; i < emitterData->ticks; ++i)
  {
    emitter->Update(emitterData->dt);
  }

  emitter->CopyToBuffer(emitterData->vtx);
}

//------------------------------------------------------------------------------
bool ParticleTunnel::Update(const UpdateState& state)
{
  UpdateCameraMatrix();

  if (g_KeyUpTrigger.IsTriggered('1'))
  {
    float s = 500;
    V3 center(randf(-s, s), randf(-s, s), 0);
    _particleEmitters.Append(ParticleEmitter()).Create(center, _settings.num_particles);
  }


  float ms = state.localTime.TotalMilliseconds() / 1000.f;
  _cbPerFrame.time.x = ms;
  _cbPerFrame.time.w = _beatTrack;

  // check which text segment we're in
  float partLength = _settings.text_length + _settings.fade_delay + _settings.fade_out;
  int idx = (int)((ms - _settings.text_start) / partLength);
  TextParticles* particles[] = { &_neuroticaParticles, &_radioSilenceParticles, &_partyParticles };
  if (ms >= _settings.text_start && idx < ELEMS_IN_ARRAY(particles))
  {
    _curParticles = particles[idx];
    float ofs = ms - _settings.text_start - idx * partLength;
    // check if we're fading out
    if (ofs < _settings.text_length + _settings.fade_delay)
    {
      _cbPerFrame.time.y = 0;
      _cbPerFrame.time.z = Clamp(0.f, 1.f, 1.f - ofs / _settings.text_length);
    }
    else
    {
      _cbPerFrame.time.y = (ofs - _settings.text_length - _settings.fade_delay) / _settings.fade_out;
      _cbPerFrame.time.z = 0;
    }
    _particlesStart = _settings.text_start + idx * partLength;
    _particlesEnd = _particlesStart + _settings.text_length;
  }
  else
  {
    _curParticles = nullptr;
  }

  rmt_ScopedCPUSample(ParticleTunnel_Update);

  rmt_ScopedCPUSample(MapWriteDiscard);


  {
    static AvgStopWatch stopWatch;
    stopWatch.Start();

    rmt_ScopedCPUSample(Particles_Update);
    float dt = 1.f / state.frequency;

    ObjectHandle vb = _particleBundle.objects._vb;
    ParticleType* vtx = _ctx->MapWriteDiscard<ParticleType>(_particleBundle.objects._vb);

#if 0

    for (int j = 0; j < state.numTicks; ++j)
    {
      for (int i = 0; i < _particleEmitters.Size(); ++i)
      {
        _particleEmitters[i].Update(dt);
      }
    }

    for (int i = 0; i < _particleEmitters.Size(); ++i)
    {
      _particleEmitters[i].CopyToBuffer(vtx + i * 6 * _settings.num_particles);
    }
#else
    SimpleAppendBuffer<TaskId, 32> tasks;

    for (int i = 0; i < _particleEmitters.Size(); ++i)
    {
      EmitterKernelData* data = g_ScratchMemory.Alloc<EmitterKernelData>(1);
      *data = EmitterKernelData{ &_particleEmitters[i], dt, state.numTicks, vtx + i * _settings.num_particles };
      KernelData kd;
      kd.data = data;
      kd.size = sizeof(EmitterKernelData);
      tasks.Append(SCHEDULER.AddTask(kd, UpdateEmitter));
    }

    for (const TaskId& taskId : tasks)
      SCHEDULER.Wait(taskId);
#endif

    _ctx->Unmap(vb);

    double avg = stopWatch.Stop();
    TANO.AddPerfCallback([=]() {
      ImGui::Text("Update time: %.3fms", 1000 * avg);
    });

  }

  {
    rmt_ScopedCPUSample(TextParticles_Update);
    if (_curParticles)
      _curParticles->Update(ms - _particlesStart, _particlesStart, _particlesEnd);
  }

#if WITH_TEXT
  {
    // Blit text
    rmt_ScopedCPUSample(TextParticle_Map);

    float* xx = _curParticles->curX;
    float* yy = _curParticles->curY;
    float* zz = _curParticles->curZ;

    Vector3* vtx = _ctx->MapWriteDiscard<Vector3>(_textGpuObjects._vb);

    for (int i = 0, e = (int)_curParticles->selectedTris.size(); i < e; ++i)
    {
      int triIdx = _curParticles->selectedTris[i];
      for (int j = 0; j < 3; ++j)
      {
        vtx->x = xx[triIdx + j];
        vtx->y = yy[triIdx + j];
        vtx->z = zz[triIdx + j];

        vtx++;
      }
    }
    _ctx->Unmap(_textGpuObjects._vb);
  }
#endif

  // Blit lines
  if (_curParticles)
  {
    float* xx = _curParticles->curX;
    float* yy = _curParticles->curY;
    float* zz = _curParticles->curZ;

    Vector3* vtx = _ctx->MapWriteDiscard<Vector3>(_lineBundle.objects._vb);

#if 0
    _numLinesPoints = 2;

    float angle = 0;
    float angleInc = 2 * 3.1415f / 20;
    float r = 20;
    Vector3 prev(r * cos(angle - angleInc), r * sin(angle - angleInc), 1);
    for (int i = 0; i < 20; ++i)
    {
      Vector3 cur(r * cos(angle), r * sin(angle), 1);
      vtx[i * 2 + 0] = prev;
      vtx[i * 2 + 1] = cur;
      prev = cur;
      angle += angleInc;
    }

    _numLinesPoints = 2 * 20;

    //vtx[0] = Vector3(-50, 0, 0);
    //vtx[1] = Vector3(+50, 0, 0);
#else
    int numTris = (int)_curParticles->selectedTris.size();
    _numLinesPoints = 6 * numTris;

    for (int i = 0; i < numTris; ++i)
    {
      int triIdx = _curParticles->selectedTris[i];
      // Barrett enumeration :)
      for (int j = 2, k = 0; k < 3; j = k++)
      {
        int idx0 = triIdx * 3 + j;
        int idx1 = triIdx * 3 + k;

        vtx->x = xx[idx0];
        vtx->y = yy[idx0];
        vtx->z = zz[idx0];
        vtx++;

        vtx->x = xx[idx1];
        vtx->y = yy[idx1];
        vtx->z = zz[idx1];
        vtx++;
      }
    }

#endif

    _ctx->Unmap(_lineBundle.objects._vb);
  }

  return true;
}

//------------------------------------------------------------------------------
bool ParticleTunnel::Render()
{
  static Color black(0, 0, 0, 0);

  ScopedRenderTarget rt(DXGI_FORMAT_R16G16B16A16_FLOAT);

  u32 flags = ShaderType::VertexShader | ShaderType::GeometryShader | ShaderType::PixelShader;
  _ctx->SetConstantBuffer(_cbPerFrame, flags, 0);

  // Render the background
  _ctx->SetRenderTarget(rt._rtHandle, GRAPHICS.GetDepthStencil(), &black);
  _ctx->SetBundle(_backgroundBundle);
  _ctx->Draw(3, 0);

  // Render particles
  _ctx->SetBundleWithSamplers(_particleBundle, PixelShader);
  _ctx->SetShaderResource(_particleTexture);
  _ctx->Draw(_settings.num_particles * _particleEmitters.Size(), 0);

#if WITH_TEXT
  // text
  _ctx->SetGpuObjects(_textGpuObjects);
  _ctx->SetGpuState(_textState);
  _ctx->Draw((u32)_curParticles->selectedTris.size(), 0);
#endif

  // lines
  ScopedRenderTarget rtLines(DXGI_FORMAT_R16G16B16A16_FLOAT);
  _ctx->SetRenderTarget(rtLines._rtHandle, GRAPHICS.GetDepthStencil(), &black);
  if (_curParticles)
  {
    _ctx->SetBundleWithSamplers(_lineBundle, PixelShader);
    //_ctx->SetGpuObjects(_linesGpuObjects);
    //_ctx->SetGpuState(_linesState);
    //_ctx->SetSamplerState(_linesState._samplers[GpuState::Linear]);
    _ctx->SetShaderResource(_lineTexture);
    _ctx->Draw(_numLinesPoints, 0);
  }
  _ctx->UnsetRenderTargets(0, 1);

  ScopedRenderTarget rtBlur(DXGI_FORMAT_R16G16B16A16_FLOAT, BufferFlags(BufferFlag::CreateSrv | BufferFlag::CreateUav));

  FullscreenEffect* fullscreen = GRAPHICS.GetFullscreenEffect();
  fullscreen->Blur(rtLines._rtHandle, rtBlur._rtHandle, _settings.blur_radius);

  // compose final image on default swap chain

  _ctx->SetConstantBuffer(_cbPerFrame, ShaderType::PixelShader, 0);

  ObjectHandle inputs[] = { rt, rtLines, rtBlur };
  fullscreen->Execute(
    inputs, 3,
    GRAPHICS.GetBackBuffer(), GRAPHICS.GetBackBufferDesc(),
    GRAPHICS.GetDepthStencil(),
    _compositeBundle.objects._ps,
    false);

  return true;
}

//------------------------------------------------------------------------------
#if WITH_IMGUI
void ParticleTunnel::RenderParameterSet()
{
  ImGui::Checkbox("extended", &extended);
  if (extended)
  {
    if (ImGui::ColorEdit4("Tint", &_settings.tint.x)) _cbPerFrame.tint = _settings.tint;
    if (ImGui::ColorEdit4("Inner", &_settings.inner_color.x)) _cbPerFrame.inner = _settings.inner_color;
    if (ImGui::ColorEdit4("Outer", &_settings.outer_color.x)) _cbPerFrame.outer = _settings.outer_color;
    ImGui::Separator();
    ImGui::InputInt("# particles", &_settings.num_particles, 25, 100);

    ImGui::SliderFloat("blur", &_settings.blur_radius, 1, 100);

    //if (ImGui::Checkbox("wireframe", &wireframe))
    //{
    //  CD3D11_RASTERIZER_DESC rssDesc = CD3D11_RASTERIZER_DESC(CD3D11_DEFAULT());
    //  rssDesc.CullMode = D3D11_CULL_NONE;
    //  if (wireframe)
    //    rssDesc.FillMode = D3D11_FILL_WIREFRAME;

    //  CD3D11_DEPTH_STENCIL_DESC dsDesc = CD3D11_DEPTH_STENCIL_DESC(CD3D11_DEFAULT());
    //  dsDesc.DepthEnable = FALSE;

    //  CD3D11_BLEND_DESC blendDesc = CD3D11_BLEND_DESC(CD3D11_DEFAULT());

    //  // pre-multiplied alpha
    //  D3D11_RENDER_TARGET_BLEND_DESC& b = blendDesc.RenderTarget[0];
    //  b.BlendEnable = TRUE;
    //  b.DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
    //  b.DestBlendAlpha = D3D11_BLEND_INV_SRC_ALPHA;

    //  _linesState.Create(&dsDesc, &blendDesc, &rssDesc);
    //}

    if (ImGui::SliderFloat("min dist", &_settings.text_min_dist, 1, 2000)) Reset();
    if (ImGui::SliderFloat("max dist", &_settings.text_max_dist, 1, 2000)) Reset();
    if (ImGui::SliderFloat("triangle prob", &_settings.text_triangle_prob, 0.f, 1.f)) Reset();
  }
  else
  {
    ImGui::SliderAngle("camera xz-plane", &angle);
    ImGui::SliderFloat("camera distance", &distance, 1, 2000);
    ImGui::SliderFloat("camera height", &height, -100, 100);
  }

  if (ImGui::Button("Reset"))
    Reset();
}
#endif

//------------------------------------------------------------------------------
#if WITH_IMGUI
void ParticleTunnel::SaveParameterSet()
{
  OutputBuffer buf;
  Serialize(buf, _settings);
  if (FILE* f = fopen(_configName.c_str(), "wt"))
  {
    fwrite(buf._buf.data(), 1, buf._ofs, f);
    fclose(f);
  }
}
#endif

//------------------------------------------------------------------------------
void ParticleTunnel::Reset()
{
  _curParticles->Create(_neuroticaTris, _settings.text_length);
}

//------------------------------------------------------------------------------
bool ParticleTunnel::Close()
{
  return true;
}

//------------------------------------------------------------------------------
BaseEffect* ParticleTunnel::Create(const char* name, u32 id)
{
  return new ParticleTunnel(name, id);
}

//------------------------------------------------------------------------------
const char* ParticleTunnel::Name()
{
  return "particle_tunnel";
}

//------------------------------------------------------------------------------
void ParticleTunnel::Register()
{
  DEMO_ENGINE.RegisterFactory(Name(), ParticleTunnel::Create);
}
