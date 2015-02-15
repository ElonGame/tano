#pragma once

namespace tano
{
  struct GpuObjects;
  struct MeshLoader;

  bool CreateBuffersFromMesh(const MeshLoader& loader, const char* name, u32* vertexFlags, GpuObjects* objects);
  bool CreateBuffersFromMeshFaceted(const MeshLoader& loader, const char* name, u32* vertexFlags, GpuObjects* objects);
}