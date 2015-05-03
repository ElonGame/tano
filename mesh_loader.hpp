#pragma once

namespace tano
{
  struct Mesh;

#pragma pack(push, 1)
  struct MeshLoader
  {
    struct SceneBlob
    {
      char id[4];
      u32 fixupOffset;
      u32 meshDataStart;
      u32 lightDataStart;
      u32 cameraDataStart;
      u32 materialDataStart;
      u32 numMeshes;
      u32 numLights;
      u32 numCameras;
      u32 numMaterials;
#pragma warning(suppress: 4200)
      char data[0];
    };

    struct MeshBlob
    {
      struct MaterialGroup
      {
        u32 materialId;
        u32 startIndex;
        u32 numIndices;
      };

      const char* name;
      u32 numVerts;
      u32 numIndices;
      u32 numMaterialGroups;
      MaterialGroup* materialGroups;
      float* verts;
      float* normals;
      float* uv;
      u32* indices;

      // bounding sphere
      float sx, sy, sz, r;
    };

    struct MaterialBlob
    {
      struct MaterialComponent
      {
        float r, g, b, a;
        const char* texture;
        float brightness;
      };

      u32 blobSize;
      const char* name;
      u32 materialId;
      u32 flags;
      MaterialComponent* color;
      MaterialComponent* luminance;
      MaterialComponent* reflection;
    };

    static u32 GetVertexFormat(const MeshBlob& mesh);

    bool Load(const char* filename);
    void ProcessFixups(u32 fixupOffset);

    vector<MeshBlob*> meshes;
    vector<MaterialBlob*> materials;
    vector<char> buf;
  };
#pragma pack(pop)

}
