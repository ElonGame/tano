#include "mesh_utils.hpp"
#include "mesh_loader.hpp"
#include "gpu_objects.hpp"
#include "init_sequence.hpp"

using namespace tano;
using namespace bristol;

extern "C" float stb_perlin_noise3(float x, float y, float z, int x_wrap=0, int y_wrap=0, int z_wrap=0);

static DirectX::SimpleMath::Vector3 Cross(const DirectX::SimpleMath::Vector3& a, const DirectX::SimpleMath::Vector3& b)
{
  return DirectX::SimpleMath::Vector3(
    (a.y * b.z) - (a.z * b.y),
    (a.z * b.x) - (a.x * b.z),
    (a.x * b.y) - (a.y * b.x));
}

namespace tano
{
  //------------------------------------------------------------------------------
  bool CreateScene(const MeshLoader& loader, scene::Scene* scene)
  {
    BEGIN_INIT_SEQUENCE();

     u32 numObjects = (u32)(loader.meshes.size() + loader.nullObjects.size() + loader.cameras.size() + loader.lights.size() + 1);
     scene->baseObjects.resize(numObjects, nullptr);

    for (const protocol::MeshBlob* meshBlob : loader.meshes)
    {
      // mesh found, allocate the vertex/index buffer
      scene->meshes.push_back(new scene::Mesh(meshBlob->name, meshBlob->id, meshBlob->parentId));
      scene->baseObjects[meshBlob->id] = scene->meshes.back();

      scene::Mesh* mesh = scene->meshes.back();
      u32 vertexFormat = MeshLoader::GetVertexFormat(*meshBlob);
      u32 vertexSize = VertexSizeFromFlags(vertexFormat);
      mesh->vertexFormat = vertexFormat;
      mesh->mtxLocal = {
        meshBlob->mtxLocal[0],   meshBlob->mtxLocal[1],   meshBlob->mtxLocal[2],   0,
        meshBlob->mtxLocal[3],   meshBlob->mtxLocal[4],   meshBlob->mtxLocal[5],   0,
        meshBlob->mtxLocal[6],   meshBlob->mtxLocal[7],   meshBlob->mtxLocal[8],   0,
        meshBlob->mtxLocal[9],   meshBlob->mtxLocal[10],  meshBlob->mtxLocal[11],  1 };

      mesh->mtxGlobal = {
        meshBlob->mtxGlobal[0], meshBlob->mtxGlobal[1], meshBlob->mtxGlobal[2], 0,
        meshBlob->mtxGlobal[3], meshBlob->mtxGlobal[4], meshBlob->mtxGlobal[5], 0,
        meshBlob->mtxGlobal[6], meshBlob->mtxGlobal[7], meshBlob->mtxGlobal[8], 0,
        meshBlob->mtxGlobal[9], meshBlob->mtxGlobal[10], meshBlob->mtxGlobal[11], 1 };

      // create a temp array to interleave the vertex data
      u32 numVerts = meshBlob->numVerts;
      vector<float> buf(vertexSize * numVerts);
      float* dst = buf.data();
      for (u32 i = 0; i < numVerts; ++i)
      {
        const auto& fnCopy = [&dst, i, meshBlob](float* src, int n)
        {
          for (int j = 0; j < n; ++j)
            dst[j] = *(src + i*3 + j);
          dst += n;
        };

        fnCopy(meshBlob->verts, 3);

        if (vertexFormat & VF_NORMAL)
          fnCopy(meshBlob->normals, 3);

        if (vertexFormat & VF_TEX2_0)
          fnCopy(meshBlob->uv, 2);
      }

      for (u32 i = 0; i < meshBlob->numMaterialGroups; ++i)
      {
        protocol::MeshBlob::MaterialGroup* mg = &meshBlob->materialGroups[i];
        mesh->materialGroups.push_back({ mg->materialId, mg->startIndex, mg->numIndices });
      }

      INIT(mesh->gpuObjects.LoadVertexShader("shaders/out/blob", "VsMesh", vertexFormat));
      INIT(mesh->gpuObjects.LoadPixelShader("shaders/out/blob", "PsMesh"));

      INIT(mesh->gpuObjects.CreateVertexBuffer(meshBlob->numVerts * vertexSize, vertexSize, buf.data()));
      INIT(mesh->gpuObjects.CreateIndexBuffer(meshBlob->numIndices * sizeof(u32), DXGI_FORMAT_R32_UINT, meshBlob->indices));
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
        mat->color.color = { materialBlob->color->r, materialBlob->color->g, materialBlob->color->b };
        mat->color.texture = materialBlob->color->texture;
        mat->color.brightness = materialBlob->color->brightness;
      }

      if (mat->flags & scene::Material::FLAG_LUMINANCE)
      {
        mat->luminance.color = { materialBlob->luminance->r, materialBlob->luminance->g, materialBlob->luminance->b };
        mat->luminance.texture = materialBlob->luminance->texture;
        mat->luminance.brightness = materialBlob->luminance->brightness;
      }

      if (mat->flags & scene::Material::FLAG_REFLECTION)
      {
        mat->reflection.color = { materialBlob->reflection->r, materialBlob->reflection->g, materialBlob->reflection->b };
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

      cam->mtxLocal = {
        cameraBlob->mtxLocal[0], cameraBlob->mtxLocal[1], cameraBlob->mtxLocal[2], 0,
        cameraBlob->mtxLocal[3], cameraBlob->mtxLocal[4], cameraBlob->mtxLocal[5], 0,
        cameraBlob->mtxLocal[6], cameraBlob->mtxLocal[7], cameraBlob->mtxLocal[8], 0,
        cameraBlob->mtxLocal[9], cameraBlob->mtxLocal[10], cameraBlob->mtxLocal[11], 1 };

      cam->mtxGlobal = {
        cameraBlob->mtxGlobal[0], cameraBlob->mtxGlobal[1], cameraBlob->mtxGlobal[2], 0,
        cameraBlob->mtxGlobal[3], cameraBlob->mtxGlobal[4], cameraBlob->mtxGlobal[5], 0,
        cameraBlob->mtxGlobal[6], cameraBlob->mtxGlobal[7], cameraBlob->mtxGlobal[8], 0,
        cameraBlob->mtxGlobal[9], cameraBlob->mtxGlobal[10], cameraBlob->mtxGlobal[11], 1 };
    }

    for (const protocol::NullObjectBlob* nullBlob : loader.nullObjects)
    {
      scene->nullObjects.push_back(new scene::NullObject(nullBlob->name, nullBlob->id, nullBlob->parentId));
      scene->baseObjects[nullBlob->id] = scene->nullObjects.back();

      scene::NullObject* obj = scene->nullObjects.back();

      obj->mtxLocal = {
        nullBlob->mtxLocal[0], nullBlob->mtxLocal[1], nullBlob->mtxLocal[2], 0,
        nullBlob->mtxLocal[3], nullBlob->mtxLocal[4], nullBlob->mtxLocal[5], 0,
        nullBlob->mtxLocal[6], nullBlob->mtxLocal[7], nullBlob->mtxLocal[8], 0,
        nullBlob->mtxLocal[9], nullBlob->mtxLocal[10], nullBlob->mtxLocal[11], 1 };

      obj->mtxGlobal = {
        nullBlob->mtxGlobal[0], nullBlob->mtxGlobal[1], nullBlob->mtxGlobal[2], 0,
        nullBlob->mtxGlobal[3], nullBlob->mtxGlobal[4], nullBlob->mtxGlobal[5], 0,
        nullBlob->mtxGlobal[6], nullBlob->mtxGlobal[7], nullBlob->mtxGlobal[8], 0,
        nullBlob->mtxGlobal[9], nullBlob->mtxGlobal[10], nullBlob->mtxGlobal[11], 1 };
    }

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
        dst[0] = mesh->verts[i*3+0];
        dst[1] = mesh->verts[i*3+1];
        dst[2] = mesh->verts[i*3+2];
        dst += 3;

        if (vertexFormat & VF_NORMAL)
        {
          dst[0] = mesh->normals[i*3+0];
          dst[1] = mesh->normals[i*3+1];
          dst[2] = mesh->normals[i*3+2];
          dst += 3;
        }

        if (vertexFormat & VF_TEX2_0)
        {
          dst[0] = mesh->uv[i*2+0];
          dst[1] = mesh->uv[i*2+1];
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
  void ComputeNormal(
    float v0x, float v0y, float v0z,
    float v1x, float v1y, float v1z,
    float v2x, float v2y, float v2z,
    float* nx, float* ny, float* nz)
  {
    float e1x = v2x - v1x;
    float e1y = v2y - v1y;
    float e1z = v2z - v1z;

    float e2x = v0x - v1x;
    float e2y = v0y - v1y;
    float e2z = v0z - v1z;

    Cross(e1x, e1y, e1z, e2x, e2y, e2z, nx, ny, nz);
    Normalize(nx, ny, nz);
  }

  //------------------------------------------------------------------------------
  bool CreateBuffersFromMeshFaceted(const MeshLoader& loader, const char* name, u32* vertexFlags, GpuObjects* objects)
  {
    for (const protocol::MeshBlob* mesh : loader.meshes)
    {
      if (strcmp(name, mesh->name) != 0)
        continue;

      // mesh found, allocate the vertex/index buffer
      u32 vertexFormat = MeshLoader::GetVertexFormat(*mesh);
      // Note, normals are automatically added, so adjust vertexFlags to show this
      vertexFormat |= VF_NORMAL;

      if (vertexFlags)
        *vertexFlags = vertexFormat;

      u32 vertexSize = VertexSizeFromFlags(vertexFormat);

      // when building a facated mesh, we don't want vertices sharing a face, so num_vertices = num_indices
      u32 numVerts = mesh->numIndices;
      u32 numIndices = mesh->numIndices;

      // create a temp array to interleave the vertex data
      vector<float> buf(vertexSize * numVerts);
      float* dst = buf.data();

      // note, we're processing a face at a time
      for (u32 i = 0; i < numIndices / 3; ++i) {

        u32 a = mesh->indices[i*3+0];
        u32 b = mesh->indices[i*3+1];
        u32 c = mesh->indices[i*3+2];
        const u32 indices[] ={ a, b, c };

        Vector3 v0(mesh->verts[a*3+0], mesh->verts[a*3+1], mesh->verts[a*3+2]);
        Vector3 v1(mesh->verts[b*3+0], mesh->verts[b*3+1], mesh->verts[b*3+2]);
        Vector3 v2(mesh->verts[c*3+0], mesh->verts[c*3+1], mesh->verts[c*3+2]);
        const Vector3* verts[] ={ &v0, &v1, &v2 };

        // compute face normal
        Vector3 e1 = v1 - v0; e1.Normalize();
        Vector3 e2 = v2 - v0; e2.Normalize();
        Vector3 n = Cross(e1, e2); n.Normalize();

        // add the 3 vertices
        for (u32 j = 0; j < 3; ++j)
        {
          dst[0] = verts[j]->x;
          dst[1] = verts[j]->y;
          dst[2] = verts[j]->z;
          dst += 3;

          dst[0] = n.x;
          dst[1] = n.y;
          dst[2] = n.z;
          dst += 3;

          if (vertexFormat & VF_TEX2_0) {
            dst[0] = mesh->uv[indices[j]*2+0];
            dst[1] = mesh->uv[indices[j]*2+1];
            dst += 2;
          }
        }
      }

      objects->CreateVertexBuffer(numVerts * vertexSize, vertexSize, buf.data());
      objects->CreateIndexBuffer(mesh->numIndices * sizeof(u32), DXGI_FORMAT_R32_UINT, mesh->indices);
      return true;
    }
    return false;
  }

  //------------------------------------------------------------------------------
  float Luminosity(const Color& col)
  {
    return 0.21f * col.x + 0.72f * col.y + 0.07f * col.z;
  }
  //------------------------------------------------------------------------------
  Color ColorFromRgba(const u32* col)
  {
    u32 c = *col;
    return Color(
      ((c >> 24) & 0xff) / 255.f,
      ((c >> 16) & 0xff) / 255.f,
      ((c >> 8) & 0xff) / 255.f,
      ((c >> 0) & 0xff) / 255.f);
  }

  //------------------------------------------------------------------------------
  void Vector3ToFloat(float* buf, const Vector3& v)
  {
    buf[0] = v.x;
    buf[1] = v.y;
    buf[2] = v.z;
  }

  //------------------------------------------------------------------------------
  bool CreateBuffersFromBitmapFaceted(
    const u8* bitmap, 
    int width, 
    int height, 
    const Vector3& scale,
    u32* vertexFlags, 
    GpuObjects* objects)
  {

    Vector3 size(scale.x * ((float)width-1), 0, scale.z * ((float)height-1));
    Vector3 ofs(-size.x / 2, 0, -size.z/2);

    vector<float> verts;
    int numVerts = 3 * (width-1) * (height-1);
    verts.resize(6 * 2 * numVerts);

    int triIdx = 0;
    const u32* buf = (u32*)bitmap;
    int hOfs = height-1;

    float* ptr = verts.data();

    // create the vertices
    for (int i = 0; i < height-1; ++i) {
      for (int j = 0; j < width-1; ++j) {

        // 1--2
        // |  |
        // 0--3

        float xx0 = ofs.x + (float)(j+0) * scale.x;
        float xx1 = ofs.x + (float)(j+1) * scale.x;
        float zz0 = ofs.z + (float)(i+0) * scale.z;
        float zz1 = ofs.z + (float)(i+1) * scale.z;

#if 0
        float x0 = xx0; float x1 = xx0; float x2 = xx1; float x3 = xx1;
        float z0 = zz0; float z1 = zz1; float z2 = zz1; float z3 = zz0;

        float y0 = scale.y * stb_perlin_noise3(256 * x0 / size.x, 0, 256 * z0 / size.z);
        float y1 = scale.y * stb_perlin_noise3(256 * x0 / size.x, 0, 256 * z1 / size.z);
        float y2 = scale.y * stb_perlin_noise3(256 * x1 / size.x, 0, 256 * z1 / size.z);
        float y3 = scale.y * stb_perlin_noise3(256 * x1 / size.x, 0, 256 * z0 / size.z);

        float n0x, n0y, n0z;
        float n1x, n1y, n1z;
        ComputeNormal(x0, y0, z0, x1, y1, z1, x2, y2, z2, &n0x, &n0y, &n0z);
        ComputeNormal(x2, y2, z2, x3, y3, z3, x0, y0, z0, &n1x, &n1y, &n1z);

        // 0, 1, 3
        *ptr++ = x0; *ptr++ = y0; *ptr++ = z0;
        *ptr++ = n0x; *ptr++ = n0x; *ptr++ = n0x;

        *ptr++ = x1; *ptr++ = y1; *ptr++ = z1;
        *ptr++ = n0x; *ptr++ = n0x; *ptr++ = n0x;

        *ptr++ = x3; *ptr++ = y3; *ptr++ = z3;
        *ptr++ = n0x; *ptr++ = n0x; *ptr++ = n0x;

        // 3, 1, 2
        *ptr++ = x3; *ptr++ = y3; *ptr++ = z3;
        *ptr++ = n1x; *ptr++ = n1x; *ptr++ = n1x;

        *ptr++ = x1; *ptr++ = y1; *ptr++ = z1;
        *ptr++ = n1x; *ptr++ = n1x; *ptr++ = n1x;

        *ptr++ = x2; *ptr++ = y2; *ptr++ = z2;
        *ptr++ = n1x; *ptr++ = n1x; *ptr++ = n1x;
#else
        Color c0 = ColorFromRgba(&buf[(hOfs-(j+0))*width+(i+0)]);
        Color c1 = ColorFromRgba(&buf[(hOfs-(j+0))*width+(i+1)]);
        Color c2 = ColorFromRgba(&buf[(hOfs-(j+1))*width+(i+1)]);
        Color c3 = ColorFromRgba(&buf[(hOfs-(j+1))*width+(i+0)]);

        Vector3 v0(xx0, scale.y * (-0.5f + Luminosity(c0)), zz0);
        Vector3 v1(xx0, scale.y * (-0.5f + Luminosity(c1)), zz1);
        Vector3 v2(xx1, scale.y * (-0.5f + Luminosity(c2)), zz1);
        Vector3 v3(xx1, scale.y * (-0.5f + Luminosity(c3)), zz0);
        
        v0.y = scale.y * stb_perlin_noise3(256 * xx0 / size.x, 0, 256 * zz0 / size.z);
        v1.y = scale.y * stb_perlin_noise3(256 * xx0 / size.x, 0, 256 * zz1 / size.z);
        v2.y = scale.y * stb_perlin_noise3(256 * xx1 / size.x, 0, 256 * zz1 / size.z);
        v3.y = scale.y * stb_perlin_noise3(256 * xx1 / size.x, 0, 256 * zz0 / size.z);
        
        Vector3 e1, e2;
        e1 = v2 - v1;
        e2 = v0 - v1;
        Vector3 n0 = Cross(e1, e2);
        n0.Normalize();
        
        e1 = v0 - v3;
        e2 = v2 - v3;
        Vector3 n1 = Cross(e1, e2);
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
#endif
      }
    }

    int vertexSize = sizeof(PosNormal);
    objects->CreateVertexBuffer(numVerts * vertexSize, vertexSize, verts.data());
    return true;
  }

}
