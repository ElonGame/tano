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

namespace tano
{
  //------------------------------------------------------------------------------
  void InitMatrix4x3(const float* m, Matrix* mtx)
  {
    // clang-format off
    *mtx = Matrix(
      *(m + 0), *(m + 1), *(m + 2), 0,
      *(m + 3), *(m + 4), *(m + 5), 0,
      *(m + 6), *(m + 7), *(m + 8), 0,
      *(m + 9), *(m + 10), *(m + 11), 1);
    // clang-format on
  }

  //------------------------------------------------------------------------------
  SceneOptions& SceneOptions::TransformToWorldSpace()
  {
    flags.Set(OptionFlag::WorldSpace);
    return *this;
  }

  //------------------------------------------------------------------------------
  SceneOptions& SceneOptions::UseMaterials()
  {
    flags.Set(OptionFlag::UseMaterials);
    return *this;
  }

  //------------------------------------------------------------------------------
  SceneOptions& SceneOptions::UniqueBuffers()
  {
    flags.Set(OptionFlag::UniqueBuffers);
    return *this;
  }

  //------------------------------------------------------------------------------
  SceneOptions& SceneOptions::SetUserDataSize(ObjectType type, u32 size)
  {
    userDataSize[type] = size;
    return *this;
  }

  //------------------------------------------------------------------------------
  SceneOptions& SceneOptions::SetLoadFilter(ObjectType filter)
  {
    loadFilter = filter;
    return *this;
  }

  //------------------------------------------------------------------------------
  bool CreateScene(const MeshLoader& loader, const SceneOptions& options, scene::Scene* scene)
  {
    BEGIN_INIT_SEQUENCE();

    u32 numObjectsByType[SceneOptions::NUM_OBJECT_TYPES];
    numObjectsByType[SceneOptions::NullObject] = (int)loader.nullObjects.size();
    numObjectsByType[SceneOptions::Mesh] = (int)loader.meshes.size();
    numObjectsByType[SceneOptions::Camera] = (int)loader.cameras.size();
    numObjectsByType[SceneOptions::Light] = (int)loader.lights.size();
    numObjectsByType[SceneOptions::Material] = (int)loader.materials.size();

    u32 numObjects = 1 + numObjectsByType[SceneOptions::NullObject] + numObjectsByType[SceneOptions::Mesh] +
                     numObjectsByType[SceneOptions::Camera] + numObjectsByType[SceneOptions::Light];

    scene->baseObjects.resize(numObjects, nullptr);

    // calc total user data size
    u32 userDataSize = 0;
    for (int i = 0; i < SceneOptions::NUM_OBJECT_TYPES; ++i)
    {
      // only look at sizes for object types we're actually loading
      if (options.loadFilter & (1 << i))
      {
        userDataSize += numObjectsByType[i] * options.userDataSize[i];
      }
    }

    u8* userDataPtr = nullptr;
    if (userDataSize > 0)
    {
      scene->userData = new u8[userDataSize];
      userDataPtr = scene->userData;
    }

    if (options.loadFilter & SceneOptions::MeshFlag)
    {
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
        if (options.userDataSize[SceneOptions::Mesh])
        {
          mesh->userData = userDataPtr;
          userDataPtr += options.userDataSize[SceneOptions::Mesh];
        }

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

        bool toWorldSpace = options.flags.IsSet(SceneOptions::OptionFlag::WorldSpace);

        if (!toWorldSpace)
        {
          mesh->mtxLocal = mtxLocal;
          mesh->mtxGlobal = mtxGlobal;
          mesh->mtxInvLocal = mtxLocal.Invert();
          mesh->mtxInvGlobal = mtxGlobal.Invert();
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

        // if (options.flags.IsSet(SceneOptions::OptionFlag::UseMaterials))
        {
          for (u32 i = 0; i < meshBlob->numMaterialGroups; ++i)
          {
            protocol::MeshBlob::MaterialGroup* mg = &meshBlob->materialGroups[i];
            mesh->materialGroups.push_back({mg->materialId, mg->startIndex, mg->numIndices});
            mesh->indexCount += mg->numIndices;
          }
        }
      }

      scene->minVerts = minVerts;
      scene->maxVerts = maxVerts;

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
    }

    if (options.loadFilter & SceneOptions::MaterialMask)
    {
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
    }

    if (options.loadFilter & SceneOptions::CameraFlag)
    {
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
    }

    if (options.loadFilter & SceneOptions::NullObjectFlag)
    {
      for (const protocol::NullObjectBlob* nullBlob : loader.nullObjects)
      {
        scene->nullObjects.push_back(new scene::NullObject(nullBlob->name, nullBlob->id, nullBlob->parentId));
        scene->baseObjects[nullBlob->id] = scene->nullObjects.back();

        scene::NullObject* obj = scene->nullObjects.back();
        InitMatrix4x3(nullBlob->mtxLocal, &obj->mtxLocal);
        InitMatrix4x3(nullBlob->mtxGlobal, &obj->mtxGlobal);
      }
    }

    for (scene::BaseObject* baseObj : scene->baseObjects)
    {
      if (baseObj && baseObj->parentId != 0xffffffff)
      {
        baseObj->parentPtr = scene->baseObjects[baseObj->parentId];
      }
    }

    END_INIT_SEQUENCE();
  }

  //------------------------------------------------------------------------------
  SceneOptions::SceneOptions()
  {
    for (int i = 0; i < NUM_OBJECT_TYPES; ++i)
    {
      userDataSize[i] = 0;
    }
  }

  //------------------------------------------------------------------------------
  int CalcPlexusGrouping(
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

  //------------------------------------------------------------------------------
  vector<u32> GenerateQuadIndices(int numQuads)
  {
    vector<u32> indices(numQuads * 6);
    for (int i = 0; i < numQuads; ++i)
    {
      // 0, 1, 3
      indices[i * 6 + 0] = i * 4 + 0;
      indices[i * 6 + 1] = i * 4 + 1;
      indices[i * 6 + 2] = i * 4 + 3;

      // 3, 1, 2
      indices[i * 6 + 3] = i * 4 + 3;
      indices[i * 6 + 4] = i * 4 + 1;
      indices[i * 6 + 5] = i * 4 + 2;
    }

    return indices;
  }

  //------------------------------------------------------------------------------
  vector<u32> GenerateCubeIndices(int numCubes)
  {
    //    5----6
    //   /    /|
    //  1----2 |
    //  | 4--|-7
    //  |/   |/
    //  0----3

    vector<u32> indices(numCubes * 36);
    u32* ptr = indices.data();

    auto AddFace = [&](int i, int a, int b, int c){
      *ptr++ = i * 8 + a;
      *ptr++ = i * 8 + b;
      *ptr++ = i * 8 + c;
    };

    for (int i = 0; i < numCubes; ++i)
    {
      // Front face
      AddFace(i, 0, 1, 2);
      AddFace(i, 0, 2, 3);

      // Back face
      AddFace(i, 4, 6, 5);
      AddFace(i, 4, 7, 6);

      // Left face
      AddFace(i, 4, 5, 1);
      AddFace(i, 4, 1, 0);

      // Right face
      AddFace(i, 3, 2, 6);
      AddFace(i, 3, 6, 7);

      // Top face
      AddFace(i, 1, 5, 6);
      AddFace(i, 1, 6, 2);

      // Bottom face
      AddFace(i, 4, 0, 3);
      AddFace(i, 4, 3, 7);
    }

    return indices;
  }

  //------------------------------------------------------------------------------
  V3* AddCube(V3* buf, const V3& pos, float scale)
  {
    *buf++ = pos + V3(-scale, -scale, -scale);
    *buf++ = pos + V3(-scale, +scale, -scale);
    *buf++ = pos + V3(+scale, +scale, -scale);
    *buf++ = pos + V3(+scale, -scale, -scale);
    *buf++ = pos + V3(-scale, -scale, +scale);
    *buf++ = pos + V3(-scale, +scale, +scale);
    *buf++ = pos + V3(+scale, +scale, +scale);
    *buf++ = pos + V3(+scale, -scale, +scale);
    return buf;
  }

  //------------------------------------------------------------------------------
  vector<u32> GenerateCubeIndicesFaceted(int numCubes)
  {
    //    5----6
    //   /    /|
    //  1----2 |
    //  | 4--|-7
    //  |/   |/
    //  0----3

    vector<u32> indices(numCubes * 36);
    u32* ptr = indices.data();

    auto AddFace = [&](int i, int a, int b, int c){
      *ptr++ = i + a;
      *ptr++ = i + b;
      *ptr++ = i + c;
    };

    // In a faceted cube all vertices are unique, so it's basically just
    // repeating adding the first 2 faces 6 times
    for (int i = 0; i < numCubes; ++i)
    {
      for (int j = 0; j < 6; ++j)
      {
        AddFace(i * 24 + j * 4, 0, 1, 2);
        AddFace(i * 24 + j * 4, 0, 2, 3);
      }
    }

    return indices;
  }

  //------------------------------------------------------------------------------
  vector<u32> CreateCylinderIndices(int numRingSegments, int numHeightSegments)
  {
    vector<u32> indices;

    // 1--2
    // |  |
    // 0--3

    for (int i = 0; i < numHeightSegments-1; ++i)
    {
      for (int j = 0; j < numRingSegments; ++j)
      {
        u32 v0 = (i + 0) * numRingSegments + j;
        u32 v1 = (i + 1) * numRingSegments + j;
        u32 v2 = (i + 1) * numRingSegments + (j+1) % numRingSegments;
        u32 v3 = (i + 0) * numRingSegments + (j+1) % numRingSegments;

        indices.push_back(v0);
        indices.push_back(v1);
        indices.push_back(v2);

        indices.push_back(v0);
        indices.push_back(v2);
        indices.push_back(v3);
      }
    }

    return indices;
  }

  //------------------------------------------------------------------------------
  V3* AddCubeWithNormal(V3* buf, const V3& pos, float scale)
  {
    V3 verts[] = {
      {pos + V3(-scale, -scale, -scale)},
      {pos + V3(-scale, +scale, -scale)},
      {pos + V3(+scale, +scale, -scale)},
      {pos + V3(+scale, -scale, -scale)},
      {pos + V3(-scale, -scale, +scale)},
      {pos + V3(-scale, +scale, +scale)},
      {pos + V3(+scale, +scale, +scale)},
      {pos + V3(+scale, -scale, +scale)},
    };

    V3 n0(0, 0, -1);
    V3 n1(0, 0, +1);
    V3 n2(-1, 0, 0);
    V3 n3(+1, 0, 0);
    V3 n4(0, +1, 0);
    V3 n5(0, -1, 0);

    //    5----6
    //   /    /|
    //  1----2 |
    //  | 4--|-7
    //  |/   |/
    //  0----3

    auto AddFace = [&](int a, int b, int c, int d, const V3& n){
      *buf++ = verts[a]; *buf++ = n;
      *buf++ = verts[b]; *buf++ = n;
      *buf++ = verts[c]; *buf++ = n;
      *buf++ = verts[d]; *buf++ = n;
    };

    /// Front face
    AddFace(0, 1, 2, 3, n0);

    // Back face
    AddFace(7, 6, 5, 4, n1);

    // Left face
    AddFace(3, 2, 6, 7, n2);

    // Right face
    AddFace(4, 5, 1, 0, n3);

    // Top face
    AddFace(1, 5, 6, 2, n4);

    // Bottom face
    AddFace(4, 0, 3, 7, n5);

    return buf;
  }

  //------------------------------------------------------------------------------
  void GeneratePlaneIndices(int width, int height, vector<u32>* out)
  {
    out->resize((width - 1) * (height - 1) * 6);
    u32* res = out->data();

    // 1 2
    // 0 3

    for (int i = 1; i < height; ++i)
    {
      for (int j = 0; j < width - 1; ++j)
      {
        // 0, 1, 3
        res[0] = (i + 0) * width + (j + 0);
        res[1] = (i - 1) * width + (j + 0);
        res[2] = (i + 0) * width + (j + 1);

        // 3, 1, 2
        res[3] = (i + 0) * width + (j + 1);
        res[4] = (i - 1) * width + (j + 0);
        res[5] = (i - 1) * width + (j + 1);

        res += 6;
      }
    }
  }
}
