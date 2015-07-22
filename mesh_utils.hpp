#pragma once
#include "gpu_objects.hpp"
#include "scene.hpp"

namespace tano
{
  struct GpuObjects;
  struct MeshLoader;

  struct PlexusGrouping;
  int CalcPlexusGrouping(
      V3* vtx, const V3* points, int num, int* neighbours, int maxNeighbours, const PlexusGrouping& config);

  bool CreateScene(const MeshLoader& loader, u32 userDataSize, scene::Scene* scene);
}