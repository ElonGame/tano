#pragma once
#include "gpu_objects.hpp"
#include "scene.hpp"

namespace tano
{
  struct GpuObjects;
  struct MeshLoader;

  struct PlexusGrouping;
  int CalcPlexusGrouping(
      vec3* vtx, const vec3* points, int num, int* neighbours, int maxNeighbours, const PlexusGrouping& config);

  struct SceneOptions
  {
    SceneOptions();

    enum ObjectType
    {
      NullObject,
      Mesh,
      Camera,
      Light,
      Material,

      NUM_OBJECT_TYPES,

      NullObjectFlag = 1 << NullObject,
      MeshFlag = 1 << Mesh,
      CameraFlag = 1 << Camera,
      LightFlag = 1 << Light,
      MaterialMask = 1 << Material,

      AllObjectTypes = NullObjectFlag | MeshFlag | CameraFlag | LightFlag | MaterialMask,
    };

    struct OptionFlag
    {
      enum Enum
      {
        WorldSpace = 1 << 0,
        UseMaterials = 1 << 1,
        UniqueBuffers = 1 << 2,
      };

      struct Bits
      {
        u32 worldSpace : 1;
        u32 useMaterials : 1;
        u32 uniqueBuffers : 1;
      };
    };

    SceneOptions& TransformToWorldSpace();
    SceneOptions& UseMaterials();
    SceneOptions& UniqueBuffers();
    SceneOptions& SetUserDataSize(ObjectType type, u32 size);
    SceneOptions& SetLoadFilter(ObjectType filter);

    typedef Flags<OptionFlag> OptionFlags;
    OptionFlags flags;

    string nameFilter;
    u32 loadFilter = AllObjectTypes;
    u32 userDataSize[NUM_OBJECT_TYPES];
  };

  //------------------------------------------------------------------------------
  bool CreateScene(const MeshLoader& loader, const SceneOptions& options, scene::Scene* scene);

  //------------------------------------------------------------------------------
  vector<u32> GenerateQuadIndices(int numQuads);
  vector<u32> GenerateCubeIndices(int numCubes);
  vector<u32> GenerateCubeIndicesFaceted(int numCubes);

  vector<u32> CreateCylinderIndices(int numRingSegments, int numHeightSegments, bool segmented = false);

  vec3* AddCube(vec3* buf, const vec3& pos, float scale);
  vec3* AddCubeWithNormal(vec3* buf, const vec3& pos, float scale, bool singleFace = false);

  void GeneratePlaneIndices(int width, int height, int ofs, vector<u32>* out);
}
