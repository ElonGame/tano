#pragma once
#include "mesh_loader.hpp"
#include "tano_math.hpp"

namespace tano
{
  struct TextWriter
  {
    enum TextSegment
    {
      TextOutline,
      TextCap1,
      TextCap2,
    };

    bool Init(const char* filename);

    void GenerateTris(const char* str, TextSegment segment, vector<vec3>* verts);
    void GenerateIndexedTris(const char* str,
        TextSegment segment,
        vector<vec3>* verts,
        vector<int>* indices,
        vector<u32>* edges = nullptr);

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
