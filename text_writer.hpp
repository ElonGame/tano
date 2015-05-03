#pragma once
#include "mesh_loader.hpp"

namespace tano
{
  struct TextWriter
  {
    bool Init(const char* filename);

    void GenerateTris(const char* str, vector<Vector3>* outlineLines, vector<Vector3>* capTris);

    struct Letter
    {
      void CalcBounds();
      float width = 0;
      float height = 0;
      MeshLoader::MeshBlob* outline = nullptr;
      MeshLoader::MeshBlob* cap1 = nullptr;
      MeshLoader::MeshBlob* cap2 = nullptr;
    };

    vector<Letter> _letters;
    MeshLoader _loader;
  };

}
