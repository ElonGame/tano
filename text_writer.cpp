#include "text_writer.hpp"

//------------------------------------------------------------------------------
using namespace tano;

//------------------------------------------------------------------------------
void TextWriter::Letter::CalcBounds()
{
  u32 numIndices = cap1->numIndices;

  float minX = FLT_MAX; float maxX = -FLT_MAX;
  float minY = FLT_MAX; float maxY = -FLT_MAX;
  for (u32 j = 0; j < numIndices; ++j)
  {
    float x = cap1->verts[cap1->indices[j] * 3 + 0];
    float y = cap1->verts[cap1->indices[j] * 3 + 1];
    float z = cap1->verts[cap1->indices[j] * 3 + 2];
    minX = min(minX, x); maxX = max(maxX, x);
    minY = min(minY, y); maxY = max(maxY, y);
  }

  width = maxX - minX;
  height = maxY - minY;
}

//------------------------------------------------------------------------------
bool TextWriter::Init(const char* filename)
{
  if (!_loader.Load(filename))
    return false;

  u32 numLetters = 'Z' - 'A' + 1;
  _letters.resize(numLetters);

  Letter* curLetter = nullptr;

  for (u32 i = 0; i < _loader.meshes.size(); ++i)
  {
    // Letter mesh names are ['A'..'Z']
    MeshLoader::MeshElement* e = _loader.meshes[i];
    int t = (int)e->name[0] - 'A';
    if (strlen(e->name) == 1 && t >= 0 && t < (int)numLetters)
    {
      curLetter = &_letters[t];
      curLetter->outline = e;
    }
    else if (strcmp(e->name, "Cap 1") == 0)
    {
      if (curLetter)
        curLetter->cap1 = e;
      else
        LOG_WARN("Cap found without matching letter!");
    }
    else if (strcmp(e->name, "Cap 2") == 0)
    {
      if (curLetter)
        curLetter->cap2 = e;
      else
        LOG_WARN("Cap found without matching letter!");
    }
  }

  for (size_t i = 0; i < _letters.size(); ++i)
  {
    _letters[i].CalcBounds();
  }

  return true;
}

//------------------------------------------------------------------------------
void TextWriter::GenerateTris(const char* str, vector<Vector3>* tris)
{
  u32 len = (u32)strlen(str);
  float xOfs = 0;
  float yOfs = 0;
  float xMin = FLT_MAX, xMax = -FLT_MAX;
  float yMin = FLT_MAX, yMax = -FLT_MAX;
  u32 idx = 0;
  for (u32 i = 0; i < len; ++i)
  {
    char ch = toupper(str[i]);

    if (ch == ' ')
    {
      xOfs += 50;
      continue;
    }
    else if (ch < 'A' || ch > 'Z')
    {
      continue;
    }

    const TextWriter::Letter& letter = _letters[ch - 'A'];

    // Copy vertices to @tris
    // The actual mesh data uses indices, so we expand those here
    MeshLoader::MeshElement* elem = letter.outline;
    u32 numIndices = elem->numIndices;
    tris->resize(tris->size() + numIndices);
    for (u32 j = 0; j < numIndices; ++j)
    {
      float x = elem->verts[elem->indices[j]*3+0];
      float y = elem->verts[elem->indices[j]*3+1];
      float z = elem->verts[elem->indices[j]*3+2];
      
      xMin = min(xMin, x + xOfs); xMax = max(xMax, x + xOfs);
      yMin = min(yMin, y + yOfs); yMax = max(yMax, y + yOfs);
      (*tris)[idx + j] = Vector3(x + xOfs, y + yOfs, z);
    }

    xOfs += letter.width * 1.05f;
    idx += numIndices;
  }

  // loop over all the verts and center the text
  float xCenter = (xMax - xMin) / 2;
  float yCenter = (yMax - yMin) / 2;
  for (u32 i = 0, e = (u32)tris->size(); i < e; ++i)
  {
    (*tris)[i].x -= xCenter;
    (*tris)[i].y -= yCenter;
  }

}

