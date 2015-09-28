#pragma once
#include "boba_scene_format.hpp"

namespace tano
{
  struct Mesh;

  struct MeshLoader
  {
    static u32 GetVertexFormat(const protocol::MeshBlob& mesh);

    bool Load(const char* filename);
    void ProcessFixups(u32 fixupOffset);

    vector<protocol::MeshBlob*> meshes;
    vector<protocol::NullObjectBlob*> nullObjects;
    vector<protocol::CameraBlob*> cameras;
    vector<protocol::LightBlob*> lights;
    vector<protocol::MaterialBlob*> materials;
    vector<protocol::SplineBlob*> splines;
    vector<char> buf;
  };

}
