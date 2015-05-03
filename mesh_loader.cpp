#include "mesh_loader.hpp"

using namespace tano;
using namespace bristol;

//------------------------------------------------------------------------------
bool MeshLoader::Load(const char* filename)
{
  if (!LoadFile(filename, &buf))
    return false;

  const SceneBlob* scene = (const SceneBlob*)&buf[0];

  if (strncmp(scene->id, "boba", 4) != 0)
    return false;

  ProcessFixups(scene->fixupOffset);

  // add meshes
  MeshBlob* meshBlob = (MeshBlob*)&buf[scene->meshDataStart];
  for (u32 i = 0; i < scene->numMeshes; ++i, ++meshBlob)
  {
    meshes.push_back(meshBlob);
  }

  char* ptr = &buf[scene->materialDataStart];
  for (u32 i = 0; i < scene->numMaterials; ++i)
  {
    MaterialBlob* materialBlob = (MaterialBlob*)ptr;
    materials.push_back(materialBlob);
    ptr += materialBlob->blobSize;
  }

  return true;
}

//------------------------------------------------------------------------------
void MeshLoader::ProcessFixups(u32 fixupOffset)
{
  // Process all the fixups. A list of locations that point to relative
  // data is stored (the fixup list), and for each of these locations, we
  // add the base of the file we loaded, converting the fixups to valid
  // memory locations

  // Note, on 64-bit, we are still limited to 32 bit file sizes and offsets, but
  // all the fixed up pointers are 64-bit.
  u32* fixupList = (u32*)&buf[fixupOffset];
  u32 numFixups = *fixupList++;
  intptr_t base = (intptr_t)&buf[0];
  u32* base32 = (u32*)base;
  for (u32 i = 0; i < numFixups; ++i)
  {
    // get the offset in the file that needs to be adjusted
    u32 src = fixupList[i];

    // adjust the offset from being a relativer pointer into the file
    // to being an absolute ptr into memory
    *(intptr_t*)(base + src) += base;
  }
}

//------------------------------------------------------------------------------
u32 MeshLoader::GetVertexFormat(const MeshBlob& mesh)
{
  return (mesh.verts ? VF_POS : 0) | (mesh.normals ? VF_NORMAL : 0) | (mesh.uv ? VF_TEX2_0 : 0);
}
