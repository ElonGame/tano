#pragma once
#include "mesh_loader.hpp"

namespace tano
{
  struct TextWriter
  {
    bool Init(const char* filename);

    void GenerateTris(const char* str, vector<Vector3>* tris);

    struct Letter
    {
      MeshLoader::MeshElement* outline = nullptr;
      MeshLoader::MeshElement* cap1 = nullptr;
      MeshLoader::MeshElement* cap2 = nullptr;
    };

    vector<Letter> _letters;
    MeshLoader _loader;
  };

}
