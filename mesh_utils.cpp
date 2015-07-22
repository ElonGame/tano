#include "mesh_utils.hpp"
#include "mesh_loader.hpp"
#include "gpu_objects.hpp"
#include "init_sequence.hpp"
#include "arena_allocator.hpp"
#include "generated/demo.types.hpp"

using namespace tano;
using namespace bristol;

namespace
{
  // vertex info per vertex format
  struct BufferInfo
  {
    BufferInfo() : vertexCount(0), indexCount(0) {}
    int vertexCount;
    int indexCount;

    vector<float> verts;
    vector<u32> indices;

    struct VtxInfo
    {
      scene::Mesh* mesh;
      u32 vertexStart;
      u32 indexStart;
    };

    vector<VtxInfo> vtxInfo;
  };
}

//------------------------------------------------------------------------------
void InitMatrix4x3(const float* m, Matrix* mtx)
{
  // clang-format off
  *mtx = Matrix(
    *(m + 0),   *(m + 1),   *(m + 2),   0, 
    *(m + 3),   *(m + 4),   *(m + 5),   0, 
    *(m + 6),   *(m + 7),   *(m + 8),   0, 
    *(m + 9),   *(m + 10),  *(m + 11),  1);
  // clang-format on
}

//------------------------------------------------------------------------------
bool tano::CreateScene(const MeshLoader& loader, u32 userDataSize, scene::Scene* scene)
{
  BEGIN_INIT_SEQUENCE();

  bool toWorldSpace = true;

  u32 numObjects = (u32)(
      loader.meshes.size() + loader.nullObjects.size() + loader.cameras.size() + loader.lights.size() + 1);
  scene->baseObjects.resize(numObjects, nullptr);

  if (userDataSize > 0)
    scene->userData = new u8[loader.meshes.size() * userDataSize];

  // vertex info per vertex format
  unordered_map<u32, BufferInfo> bufferInfo;

  for (size_t i = 0; i < loader.meshes.size(); ++i)
  {
    const protocol::MeshBlob* meshBlob = loader.meshes[i];
    u32 fmt = MeshLoader::GetVertexFormat(*meshBlob);
    BufferInfo& info = bufferInfo[fmt];
    info.indexCount += meshBlob->numIndices;
    info.vertexCount += meshBlob->numVerts;
  }

  V3 minVerts(+FLT_MAX, +FLT_MAX, +FLT_MAX);
  V3 maxVerts(-FLT_MAX, -FLT_MAX, -FLT_MAX);

  for (size_t m = 0; m < loader.meshes.size(); ++m)
  {
    const protocol::MeshBlob* meshBlob = loader.meshes[m];
    scene::Mesh* mesh = new scene::Mesh(meshBlob->name, meshBlob->id, meshBlob->parentId);
    if (userDataSize)
      mesh->userData = &scene->userData[m * userDataSize];

    u32 vertexFormat = MeshLoader::GetVertexFormat(*meshBlob);
    u32 vertexSize = VertexSizeFromFlags(vertexFormat);
    u32 floatsPerElem = vertexSize / sizeof(float);
    BufferInfo& info = bufferInfo[vertexFormat];

    vector<BufferInfo::VtxInfo>& vtxInfo = info.vtxInfo;
    u32 prevFloatCount = (u32)info.verts.size();
    u32 prevVertexCount = prevFloatCount / floatsPerElem;
    u32 prevIndexCount = (u32)info.indices.size();

    vtxInfo.push_back(BufferInfo::VtxInfo{mesh, prevVertexCount, prevIndexCount});

    scene->meshes.push_back(mesh);
    scene->baseObjects[meshBlob->id] = mesh;

    mesh->vertexFormat = vertexFormat;

    Matrix mtxLocal, mtxGlobal;
    InitMatrix4x3(meshBlob->mtxLocal, &mtxLocal);
    InitMatrix4x3(meshBlob->mtxGlobal, &mtxGlobal);

    if (toWorldSpace)
    {
      // mesh->mtxLocal = Matrix::Identity();
      // mesh->mtxGlobal = Matrix::Identity();

      mesh->mtxLocal = mtxLocal;
      mesh->mtxGlobal = mtxGlobal;

      mesh->mtxInvLocal = mtxLocal.Invert();
      mesh->mtxInvGlobal = mtxGlobal.Invert();
    }
    else
    {
      mesh->mtxLocal = mtxLocal;
      mesh->mtxGlobal = mtxGlobal;
    }

    u32 numVerts = meshBlob->numVerts;
    u32 numIndices = meshBlob->numIndices;
    info.verts.resize(prevFloatCount + floatsPerElem * numVerts);
    info.indices.resize(prevIndexCount + numIndices);
    float* verts = &info.verts[prevFloatCount];
    u32* indices = &info.indices[prevIndexCount];

    // De-interleave the vertex data into a single array
    for (u32 i = 0; i < numVerts; ++i)
    {
      const auto& fnCopy = [&verts, i, meshBlob](float* src, int n)
      {
        for (int j = 0; j < n; ++j)
          verts[j] = *(src + i * n + j);
        verts += n;
      };

      if (toWorldSpace)
      {
        Vector3 vv(meshBlob->verts[i * 3 + 0], meshBlob->verts[i * 3 + 1], meshBlob->verts[i * 3 + 2]);
        Vector3::Transform(vv, mtxLocal, vv);
        verts[0] = vv.x;
        verts[1] = vv.y;
        verts[2] = vv.z;
        verts += 3;
        // fnCopy(&vv.x, 3);
      }
      else
      {
        fnCopy(meshBlob->verts, 3);
      }

      V3 vv(verts[-3], verts[-2], verts[-1]);
      minVerts = Min(minVerts, vv);
      maxVerts = Max(maxVerts, vv);

      if (vertexFormat & VF_NORMAL)
        fnCopy(meshBlob->normals, 3);

      if (vertexFormat & VF_TEX2_0)
        fnCopy(meshBlob->uv, 2);
    }

    copy(meshBlob->indices, meshBlob->indices + meshBlob->numIndices, indices);

    // TODO: these are ignored for now..
    for (u32 i = 0; i < meshBlob->numMaterialGroups; ++i)
    {
      protocol::MeshBlob::MaterialGroup* mg = &meshBlob->materialGroups[i];
      mesh->materialGroups.push_back({mg->materialId, mg->startIndex, mg->numIndices});
      mesh->indexCount += mg->numIndices;
    }
  }

  // Create the mesh buffers
  for (const auto& kv : bufferInfo)
  {
    u32 fmt = kv.first;
    const BufferInfo& info = kv.second;

    u32 vertexSize = VertexSizeFromFlags(fmt);

    scene::MeshBuffer* buf = new scene::MeshBuffer();
    scene->meshBuffers.push_back(buf);
    ObjectHandle vb = GRAPHICS.CreateBuffer(
        D3D11_BIND_VERTEX_BUFFER, info.vertexCount * vertexSize, false, info.verts.data(), vertexSize);
    ObjectHandle ib = GRAPHICS.CreateBuffer(D3D11_BIND_INDEX_BUFFER, info.indexCount * sizeof(u32), false,
        info.indices.data(), DXGI_FORMAT_R32_UINT);

    buf->vb = vb;
    buf->ib = ib;
    for (const BufferInfo::VtxInfo& vtxInfo : info.vtxInfo)
    {
      buf->meshes.push_back(vtxInfo.mesh);
      vtxInfo.mesh->vb = vb;
      vtxInfo.mesh->ib = ib;
      vtxInfo.mesh->startIndexLocation = vtxInfo.indexStart;
      vtxInfo.mesh->baseVertexLocation = vtxInfo.vertexStart;
    }
  }

  for (const protocol::MaterialBlob* materialBlob : loader.materials)
  {
    if (scene->materials.find(materialBlob->materialId) == scene->materials.end())
      scene->materials[materialBlob->materialId] = new scene::Material();

    scene::Material* mat = scene->materials[materialBlob->materialId];
    mat->id = materialBlob->materialId;
    mat->flags = materialBlob->flags;
    mat->name = materialBlob->name;

    if (mat->flags & scene::Material::FLAG_COLOR)
    {
      mat->color.color = {materialBlob->color->r, materialBlob->color->g, materialBlob->color->b};
      mat->color.texture = materialBlob->color->texture;
      mat->color.brightness = materialBlob->color->brightness;
    }

    if (mat->flags & scene::Material::FLAG_LUMINANCE)
    {
      mat->luminance.color = {
          materialBlob->luminance->r, materialBlob->luminance->g, materialBlob->luminance->b};
      mat->luminance.texture = materialBlob->luminance->texture;
      mat->luminance.brightness = materialBlob->luminance->brightness;
    }

    if (mat->flags & scene::Material::FLAG_REFLECTION)
    {
      mat->reflection.color = {
          materialBlob->reflection->r, materialBlob->reflection->g, materialBlob->reflection->b};
      mat->reflection.texture = materialBlob->reflection->texture;
      mat->reflection.brightness = materialBlob->reflection->brightness;
    }
  }

  for (const protocol::CameraBlob* cameraBlob : loader.cameras)
  {
    scene->cameras.push_back(new scene::Camera(cameraBlob->name, cameraBlob->id, cameraBlob->parentId));
    scene->baseObjects[cameraBlob->id] = scene->cameras.back();

    scene::Camera* cam = scene->cameras.back();
    cam->verticalFov = cameraBlob->verticalFov;
    cam->nearPlane = cameraBlob->nearPlane;
    cam->farPlane = cameraBlob->farPlane;

    InitMatrix4x3(cameraBlob->mtxLocal, &cam->mtxLocal);
    InitMatrix4x3(cameraBlob->mtxGlobal, &cam->mtxGlobal);
  }

  for (const protocol::NullObjectBlob* nullBlob : loader.nullObjects)
  {
    scene->nullObjects.push_back(new scene::NullObject(nullBlob->name, nullBlob->id, nullBlob->parentId));
    scene->baseObjects[nullBlob->id] = scene->nullObjects.back();

    scene::NullObject* obj = scene->nullObjects.back();
    InitMatrix4x3(nullBlob->mtxLocal, &obj->mtxLocal);
    InitMatrix4x3(nullBlob->mtxGlobal, &obj->mtxGlobal);
  }

  scene->minVerts = minVerts;
  scene->maxVerts = maxVerts;

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool CreateBuffersFromMesh(const MeshLoader& loader, const char* name, u32* vertexFlags, GpuObjects* objects)
{
  for (const protocol::MeshBlob* mesh : loader.meshes)
  {
    // if a name is given, check against this
    if (name && strcmp(name, mesh->name) != 0)
      continue;

    // mesh found, allocate the vertex/index buffer
    u32 vertexFormat = MeshLoader::GetVertexFormat(*mesh);
    if (vertexFlags)
      *vertexFlags = vertexFormat;

    u32 vertexSize = VertexSizeFromFlags(vertexFormat);

    // create a temp array to interleave the vertex data
    u32 numVerts = mesh->numVerts;
    vector<float> buf(vertexSize * numVerts);
    float* dst = buf.data();
    for (u32 i = 0; i < numVerts; ++i)
    {
      // Copy a vertex at a time (dst[0], dst[1], dst[2] etc refers to x, y, z)
      dst[0] = mesh->verts[i * 3 + 0];
      dst[1] = mesh->verts[i * 3 + 1];
      dst[2] = mesh->verts[i * 3 + 2];
      dst += 3;

      if (vertexFormat & VF_NORMAL)
      {
        dst[0] = mesh->normals[i * 3 + 0];
        dst[1] = mesh->normals[i * 3 + 1];
        dst[2] = mesh->normals[i * 3 + 2];
        dst += 3;
      }

      if (vertexFormat & VF_TEX2_0)
      {
        dst[0] = mesh->uv[i * 2 + 0];
        dst[1] = mesh->uv[i * 2 + 1];
        dst += 2;
      }
    }

    objects->CreateVertexBuffer(mesh->numVerts * vertexSize, vertexSize, buf.data());
    objects->CreateIndexBuffer(mesh->numIndices * sizeof(u32), DXGI_FORMAT_R32_UINT, mesh->indices);
    return true;
  }
  return false;
}

//------------------------------------------------------------------------------
int tano::CalcPlexusGrouping(
    V3* vtx, const V3* points, int num, int* neighbours, int maxNeighbours, const PlexusGrouping& config)
{
  u8* connected = g_ScratchMemory.Alloc<u8>(num * num);
  memset(connected, 0, num * num);

  struct DistEntry
  {
    float dist;
    int idx;
  };
  DistEntry* dist = g_ScratchMemory.Alloc<DistEntry>(num);
  int* idx = g_ScratchMemory.Alloc<int>(num);
  u8* degree = g_ScratchMemory.Alloc<u8>(num);
  memset(degree, 0, num);

  V3* orgVtx = vtx;

  float minDist = config.min_dist;
  float maxDist = config.max_dist;
  int numNeighbours = config.num_neighbours;

  float eps = config.eps;

  auto fnSort = [&](int a, int b)
  {
    float da = dist[a].dist;
    float db = dist[b].dist;
    float d = da - db;
    if (d < 0)
      d *= -1;
    if (d < eps)
      return degree[a] < degree[b];
    return da < db;
  };

  // iterate over all the neighbours, and calc distance etc
  for (int i = 0; i < num; ++i)
    idx[i] = i;

  for (int i = 0; i < num; ++i)
  {
    int numValid = 0;
    for (int j = 0; j < numNeighbours; ++j)
    {
      int curIdx = neighbours[i * maxNeighbours + j];
      if (curIdx == -1)
        break;

      float d = Distance(points[i], points[curIdx]);

      if (curIdx == i || d < minDist || d > maxDist || connected[i * num + curIdx] ||
          connected[curIdx * num + i])
        continue;

      idx[numValid] = numValid;
      dist[numValid].dist = d;
      dist[numValid].idx = curIdx;
      numValid++;
    }

    sort(idx, idx + numValid, fnSort);

    int cnt = min(config.num_nearest, numValid);
    for (int j = 0; j < cnt; ++j)
    {
      int curIdx = dist[idx[j]].idx;

      vtx[0] = points[i];
      vtx[1] = points[curIdx];
      vtx += 2;

      connected[i * num + curIdx] = 1;
      degree[i]++;
      degree[curIdx]++;
    }
  }

  return (int)(vtx - orgVtx);
}
