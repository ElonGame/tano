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
  // split text into rows
  vector<string> rows;
  u32 len = (u32)strlen(str);
  const char* start = str;
  const char* end = str + len;
  const char* cur = start;
  while (start != end)
  {
    cur = strchr(start, '\n');
    if (!cur)
      break;
    rows.push_back(string(start, cur - start));
    start = cur + 1;
  }

  if (start != end)
    rows.push_back(string(start, end - start));

  struct RowDim
  {
    u32 triStart, triEnd;
    float xMin, xMax;
    float yMin, yMax;
  };

  vector<RowDim> dims;

  float xMin = FLT_MAX, xMax = -FLT_MAX;
  float yMin = FLT_MAX, yMax = -FLT_MAX;
  u32 idx = 0;
  for (const string& str : rows)
  {
    float xOfs = 0;
    u32 curLen = (u32)str.size();
    u32 triStart = (u32)tris->size();
    for (u32 i = 0; i < curLen; ++i)
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
      //MeshLoader::MeshElement* elem = letter.cap1;
      MeshLoader::MeshElement* elem = letter.outline;
      u32 numIndices = elem->numIndices;
      tris->resize(tris->size() + numIndices);
      for (u32 j = 0; j < numIndices; ++j)
      {
        int vertIdx = elem->indices[j] * 3;
        float x = elem->verts[vertIdx + 0] + xOfs;
        float y = elem->verts[vertIdx + 1];
        float z = elem->verts[vertIdx + 2];

        xMin = min(xMin, x); xMax = max(xMax, x);
        yMin = min(yMin, y); yMax = max(yMax, y);
        (*tris)[idx + j] = Vector3(x, y, z);
      }

      xOfs += letter.width * 1.05f;
      idx += numIndices;
    }

    dims.push_back({triStart, (u32)tris->size(), xMin, xMax, yMin, yMax});
  }

  float xStart = dims[0].xMin;
  float xEnd = dims[0].xMax;
  float yStart = dims[0].yMin;
  float yEnd = dims[0].yMax;
  for (size_t i = 1; i < dims.size(); ++i)
  {
    xStart = min(xStart, dims[i].xMin);
    xEnd = max(xEnd, dims[i].xMax);
    yStart = min(yStart, dims[i].yMin);
    yEnd = max(yEnd, dims[i].yMax);
  }

  float sx = xEnd - xStart;
  float sy = yEnd - yStart;

  float y0 = dims[0].yMax - dims[0].yMin;
  float xCenter = sx / 2;
  float yCenter = sy / 2;
  float yOfs = yCenter;

  for (const RowDim& row : dims)
  {
    float rx = row.xMax - row.xMin;
    float ry = row.yMax - row.yMin;
    float xRowOfs = (sx - rx) / 2;
    float yRowOfs = (sy - ry) / 2;

    // loop over all the verts and center the text
    for (u32 i = row.triStart, e = row.triEnd; i < e; ++i)
    {
      (*tris)[i].x -= (xCenter - xRowOfs);
      //(*tris)[i].y -= (yCenter - yRowOfs + yOfs);
      (*tris)[i].y += yOfs;
    }

    yOfs -= ry;
  }
  
}

