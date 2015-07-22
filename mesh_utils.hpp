#pragma once
#include "gpu_objects.hpp"
#include "scene.hpp"

namespace tano
{
  struct GpuObjects;
  struct MeshLoader;

  struct PlexusGrouping;
  int CalcPlexusGrouping(V3* vtx, const V3* points, int num, int* neighbours, int maxNeighbours, const PlexusGrouping& config);

  bool CreateScene(const MeshLoader& loader, u32 userDataSize, scene::Scene* scene);
  bool CreateBuffersFromMesh(
      const MeshLoader& loader, const char* name, u32* vertexFlags, GpuObjects* objects);
  bool CreateBuffersFromMeshFaceted(
      const MeshLoader& loader, const char* name, u32* vertexFlags, GpuObjects* objects);

  bool CreateBuffersFromBitmapFaceted(
      const u8* bitmap, int width, int height, const Vector3& scale, u32* vertexFlags, GpuObjects* objects);
}