#pragma once
#include "gpu_objects.hpp"

namespace tano
{
  namespace scene
  {
    struct BaseObject
    {
      BaseObject(const string& name, u32 id, u32 parentId) : name(name), id(id), parentId(parentId) {}
      Matrix mtxLocal = Matrix::Identity();
      Matrix mtxGlobal = Matrix::Identity();
      BaseObject* parentPtr = nullptr;
      string name;
      u32 id;
      u32 parentId;
    };

    struct Mesh : BaseObject
    {
      Mesh(const string& name, u32 id, u32 parentId) : BaseObject(name, id, parentId) {}

      struct MaterialGroup
      {
        u32 materialId;
        u32 startIndex;
        u32 indexCount;
      };

      GpuObjects gpuObjects;
      u32 vertexFormat;
      vector<MaterialGroup> materialGroups;
    };

    struct Camera : BaseObject
    {
      Camera(const string& name, u32 id, u32 parentId) : BaseObject(name, id, parentId) {}

      float verticalFov;
      float nearPlane, farPlane;
    };

    struct NullObject : BaseObject
    {
      NullObject(const string& name, u32 id, u32 parentId) : BaseObject(name, id, parentId) {}
    };

    struct Material
    {
      struct MaterialComponent
      {
        Color color;
        string texture;
        float brightness;
      };

      enum Flags
      {
        FLAG_COLOR = 1 << 0,
        FLAG_LUMINANCE = 1 << 1,
        FLAG_REFLECTION = 1 << 2,
      };

      u32 id;
      u32 flags;
      string name;

      MaterialComponent color;
      MaterialComponent luminance;
      MaterialComponent reflection;
    };

    struct Scene
    {
      ~Scene();
      vector<BaseObject*> baseObjects;
      vector<Mesh*> meshes;
      vector<Camera*> cameras;
      vector<NullObject*> nullObjects;
      unordered_map<u32, Material*> materials;
    };
  }
}