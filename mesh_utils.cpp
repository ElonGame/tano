#include "mesh_utils.hpp"
#include "mesh_loader.hpp"
#include "gpu_objects.hpp"

using namespace tano;
using namespace bristol;

namespace tano
{
  //------------------------------------------------------------------------------
  bool CreateBuffersFromMesh(const MeshLoader& loader, const char* name, u32* vertexFlags, GpuObjects* objects)
  {
    for (const MeshLoader::MeshElement* mesh : loader.meshes)
    {
      if (strcmp(name, mesh->name) != 0)
        continue;

      // mesh found, allocate the vertex/index buffer
      u32 vertexFormat = mesh->GetVertexFormat();
      if (vertexFlags)
        *vertexFlags = vertexFormat;

      u32 vertexSize = VertexSizeFromFlags(vertexFormat);

      // create a temp array to interleave the vertex data
      u32 numVerts = mesh->numVerts;
      vector<float> buf(vertexSize * numVerts);
      float* dst = buf.data();
      for (u32 i = 0; i < numVerts; ++i)
      {
        dst[0] = mesh->verts[i*3+0];
        dst[1] = mesh->verts[i*3+1];
        dst[2] = mesh->verts[i*3+2];
        dst += 3;

        if (vertexFormat & VF_NORMAL) {
          dst[0] = mesh->normals[i*3+0];
          dst[1] = mesh->normals[i*3+1];
          dst[2] = mesh->normals[i*3+2];
          dst += 3;
        }

        if (vertexFormat & VF_TEX2_0) {
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

  inline Vector3 Cross(const Vector3& a, const Vector3& b)
  {
    using namespace DirectX;
    XMVECTOR v1 = XMLoadFloat3(&a);
    XMVECTOR v2 = XMLoadFloat3(&b);
    XMVECTOR r = XMVector3Cross(v1, v2);
    Vector3 tmp;
    XMStoreFloat3(&tmp, r);
    return tmp;
  }

  //------------------------------------------------------------------------------
  bool CreateBuffersFromMeshFaceted(const MeshLoader& loader, const char* name, u32* vertexFlags, GpuObjects* objects)
  {
    for (const MeshLoader::MeshElement* mesh : loader.meshes)
    {
      if (strcmp(name, mesh->name) != 0)
        continue;

      // mesh found, allocate the vertex/index buffer
      u32 vertexFormat = mesh->GetVertexFormat();
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

        u32 a = mesh->indices[i*3+2];
        u32 b = mesh->indices[i*3+1];
        u32 c = mesh->indices[i*3+0];
        const u32 indices[] ={ a, b, c };

        Vector3 v0(mesh->verts[a*3+0], mesh->verts[a*3+1], mesh->verts[a*3+2]);
        Vector3 v1(mesh->verts[b*3+0], mesh->verts[b*3+1], mesh->verts[b*3+2]);
        Vector3 v2(mesh->verts[c*3+0], mesh->verts[c*3+1], mesh->verts[c*3+2]);
        const Vector3* verts[] ={ &v0, &v1, &v2 };

        // compute face normal
        Vector3 e1 = v1 - v0; e1.Normalize();
        Vector3 e2 = v2 - v0; e2.Normalize();
        Vector3 n  = Cross(e1, e2);


        // add the 3 vertices
        for (u32 j = 0; j < 3; ++j)
        {
          dst[0] = verts[j]->x;
          dst[1] = verts[j]->y;
          dst[2] = verts[j]->z;
          dst += 3;

          if (vertexFormat & VF_NORMAL) {
            dst[0] = n.x;
            dst[1] = n.y;
            dst[2] = n.z;
            dst += 3;
          }

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

}
