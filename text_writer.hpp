#pragma once
#include "mesh_loader.hpp"
#include "tano_math.hpp"

namespace tano
{
  struct TextWriter
  {
    bool Init(const char* filename);

    void GenerateTris(const char* str, vector<V3>* outlineLines, vector<V3>* capTris);
    void GenerateIndexedTris(const char* str, vector<V3>* outlineLines, vector<V3>* capTris);

    struct Letter
    {
      void CalcBounds();
      float width = 0;
      float height = 0;
      protocol::MeshBlob* outline = nullptr;
      protocol::MeshBlob* cap1 = nullptr;
      protocol::MeshBlob* cap2 = nullptr;
    };

    vector<Letter> _letters;
    MeshLoader _loader;
  };

}
