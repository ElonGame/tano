#include "scene.hpp"

using namespace tano;
using namespace tano::scene;
using namespace bristol;

Scene::~Scene()
{
  SeqDelete(&meshes);
  SeqDelete(&cameras);
  SeqDelete(&nullObjects);

  AssocDelete(&materials);
}