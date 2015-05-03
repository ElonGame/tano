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
    MeshLoader::MeshBlob* e = _loader.meshes[i];
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
void TextWriter::GenerateTris(
    const char* str,
    vector<Vector3>* outlineTris,
    vector<Vector3>* capTris)
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
    u32 outlineTriStart, outlineTriEnd;
    u32 capTriStart, capTriEnd;
    float xMin, xMax;
    float yMin, yMax;
  };

  vector<RowDim> dims;

  Vector3 vMin(+FLT_MAX, +FLT_MAX, +FLT_MAX);
  Vector3 vMax(-FLT_MAX, -FLT_MAX, -FLT_MAX);
  u32 outlineIdx = 0;
  u32 capIdx = 0;
  for (const string& str : rows)
  {
    float xOfs = 0;
    u32 curLen = (u32)str.size();
    u32 outlineTriStart = (u32)outlineTris->size();
    u32 capTriStart = (u32)capTris->size();
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
      if (outlineTris)
      {
        MeshLoader::MeshBlob* elem = letter.outline;
        u32 numIndices = elem->numIndices;
        outlineTris->resize(outlineTris->size() + numIndices);
        for (u32 j = 0; j < numIndices; ++j)
        {
          int vertIdx = elem->indices[j] * 3;
          Vector3 v(elem->verts[vertIdx + 0] + xOfs, elem->verts[vertIdx + 1], elem->verts[vertIdx + 2]);
          vMin = Vector3::Min(v, vMin);
          vMax = Vector3::Max(v, vMax);
          (*outlineTris)[outlineIdx + j] = v;
        }

        outlineIdx += numIndices;
      }

      if (capTris)
      {
        MeshLoader::MeshBlob* elem = letter.cap1;
        u32 numIndices = elem->numIndices;
        capTris->resize(capTris->size() + numIndices);
        for (u32 j = 0; j < numIndices; ++j)
        {
          int vertIdx = elem->indices[j] * 3;
          Vector3 v(elem->verts[vertIdx + 0] + xOfs, elem->verts[vertIdx + 1], elem->verts[vertIdx + 2]);
          if (!outlineTris)
          {
            vMin = Vector3::Min(v, vMin);
            vMax = Vector3::Max(v, vMax);
          }
          (*capTris)[capIdx + j] = v;
        }

        capIdx += numIndices;
      }

      xOfs += letter.width * 1.05f;
    }

    dims.push_back({
      outlineTriStart, (u32)outlineTris->size(), capTriStart, (u32)capTris->size(),
      vMin.x, vMax.x, vMin.y, vMax.y});
  }

  float xStart = dims[0].xMin;
  float xEnd = dims[0].xMax;
  float ySize = 0;
  for (size_t i = 1; i < dims.size(); ++i)
  {
    xStart = min(xStart, dims[i].xMin);
    xEnd = max(xEnd, dims[i].xMax);
    ySize += dims[i].yMax - dims[i].yMin;
  }

  float sx = xEnd - xStart;
  float xCenter = sx / 2;
  float yOfs = ySize / 2;

  for (const RowDim& row : dims)
  {
    float rx = row.xMax - row.xMin;
    float ry = row.yMax - row.yMin;
    float xRowOfs = (sx - rx) / 2;

    // loop over all the verts and center the text
    if (outlineTris)
    {
      for (u32 i = row.outlineTriStart, e = row.outlineTriEnd; i < e; ++i)
      {
        (*outlineTris)[i].x -= (xCenter - xRowOfs);
        // we first center the text around its local origin, and then move it
        // to the correct global position
        (*outlineTris)[i].y += -ry / 2 + yOfs;
      }
    }

    if (capTris)
    {
      for (u32 i = row.capTriStart, e = row.capTriEnd; i < e; ++i)
      {
        (*capTris)[i].x -= (xCenter - xRowOfs);
        // we first center the text around its local origin, and then move it
        // to the correct global position
        (*capTris)[i].y += -ry / 2 + yOfs;
      }
    }

    yOfs -= ry;
  }
}

