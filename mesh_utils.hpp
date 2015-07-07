#pragma once
#include "gpu_objects.hpp"
#include "scene.hpp"

namespace tano
{
  struct GpuObjects;
  struct MeshLoader;

  bool CreateScene(const MeshLoader& loader, bool createShaders, scene::Scene* scene);
  bool CreateBuffersFromMesh(const MeshLoader& loader, const char* name, u32* vertexFlags, GpuObjects* objects);
  bool CreateBuffersFromMeshFaceted(const MeshLoader& loader, const char* name, u32* vertexFlags, GpuObjects* objects);

  bool CreateBuffersFromBitmapFaceted(
    const u8* bitmap, 
    int width, 
    int height, 
    const Vector3& scale, 
    u32* vertexFlags, 
    GpuObjects* objects);

}