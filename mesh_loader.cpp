#include "mesh_loader.hpp"

using namespace tano;
using namespace bristol;

//------------------------------------------------------------------------------
bool MeshLoader::Load(const char* filename)
{
  if (!LoadFile(filename, &buf))
    return false;

  const Scene* scene = (const Scene*)&buf[0];

  if (strncmp(scene->id, "boba", 4) != 0)
    return false;

  ProcessFixups(scene->fixupOffset);

  // add meshes
  char* ptr = &buf[scene->elementOffset[(u32)SceneElement::Mesh]];
  u32 numMeshes = *(u32*)ptr;
  MeshElement* element = (MeshElement*)(ptr + 4);
  for (u32 i = 0; i < numMeshes; ++i, ++element)
  {
    meshes.push_back(element);
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
  for (u32 i = 0; i < numFixups; ++i)
  {
    u32 fixup = fixupList[i];

    // this is a bit cheesy, but the offsets are stored as 32 bits
    u32 ofs = *(u32*)(base + fixup);
    *(intptr_t*)(base + fixup) = base + ofs;
  }
}

//------------------------------------------------------------------------------
u32 MeshLoader::MeshElement::GetVertexFormat() const
{
  return (verts ? VF_POS : 0) | (normals ? VF_NORMAL : 0) | (uv ? VF_TEX2_0 : 0);
}
