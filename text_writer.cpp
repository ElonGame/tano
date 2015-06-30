#include "text_writer.hpp"

//------------------------------------------------------------------------------
using namespace tano;

//------------------------------------------------------------------------------
namespace
{
  struct RowDim
  {
    u32 triStart, triEnd;
    float xMin, xMax;
    float yMin, yMax;
  };

  void MakeRows(const char* str, vector<string>* rows)
  {
    // split text into rows
    u32 len = (u32)strlen(str);
    const char* start = str;
    const char* end = str + len;
    const char* cur = start;
    while (start != end)
    {
      cur = strchr(start, '\n');
      if (!cur)
        break;
      rows->push_back(string(start, cur - start));
      start = cur + 1;
    }

    if (start != end)
      rows->push_back(string(start, end - start));
  }
};

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
    protocol::MeshBlob* e = _loader.meshes[i];
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
void TextWriter::GenerateTris(const char* str, TextSegment segment, vector<V3>* verts)
{
  // split text into rows
  vector<string> rows;
  MakeRows(str, &rows);

  vector<RowDim> lineInfo;

  V3 vMin(+FLT_MAX, +FLT_MAX, +FLT_MAX);
  V3 vMax(-FLT_MAX, -FLT_MAX, -FLT_MAX);
  u32 vtxIdx = 0;
  for (const string& str : rows)
  {
    float xOfs = 0;
    u32 curLen = (u32)str.size();
    u32 triStart = (u32)verts->size();
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
      const protocol::MeshBlob* blobs[] = { letter.outline, letter.cap1, letter.cap2 };
      const protocol::MeshBlob* elem = blobs[(int)segment];
      u32 numIndices = elem->numIndices;
      verts->resize(verts->size() + numIndices);
      for (u32 j = 0; j < numIndices; ++j)
      {
        int vertIdx = elem->indices[j] * 3;
        V3 v(elem->verts[vertIdx + 0] + xOfs, elem->verts[vertIdx + 1], elem->verts[vertIdx + 2]);
        vMin = Min(v, vMin);
        vMax = Max(v, vMax);
        (*verts)[vtxIdx + j] = v;
      }

      vtxIdx += numIndices;

      xOfs += letter.width * 1.05f;
    }

    lineInfo.push_back({
      triStart, (u32)verts->size(),
      vMin.x, vMax.x, vMin.y, vMax.y});
  }

  float xStart = lineInfo[0].xMin;
  float xEnd = lineInfo[0].xMax;
  float ySize = 0;
  for (size_t i = 1; i < lineInfo.size(); ++i)
  {
    xStart = min(xStart, lineInfo[i].xMin);
    xEnd = max(xEnd, lineInfo[i].xMax);
    ySize += lineInfo[i].yMax - lineInfo[i].yMin;
  }

  float sx = xEnd - xStart;
  float xCenter = sx / 2;
  float yOfs = ySize / 2;

  for (const RowDim& row : lineInfo)
  {
    float rx = row.xMax - row.xMin;
    float ry = row.yMax - row.yMin;
    float xRowOfs = (sx - rx) / 2;

    // loop over all the verts and center the text
    if (verts)
    {
      for (u32 i = row.triStart, e = row.triEnd; i < e; ++i)
      {
        (*verts)[i].x -= (xCenter - xRowOfs);
        // we first center the text around its local origin, and then move it
        // to the correct global position
        (*verts)[i].y += -ry / 2 + yOfs;
      }
    }

    yOfs -= ry;
  }
}

//------------------------------------------------------------------------------
void TextWriter::GenerateIndexedTris(
    const char* str,
    TextSegment segment,
    vector<V3>* verts,
    vector<int>* indices)
{

  vector<string> rows;
  MakeRows(str, &rows);

  vector<RowDim> lineInfo;

  V3 vMin(+FLT_MAX, +FLT_MAX, +FLT_MAX);
  V3 vMax(-FLT_MAX, -FLT_MAX, -FLT_MAX);
  for (const string& str : rows)
  {
    float xOfs = 0;
    u32 curLen = (u32)str.size();
    u32 triStart = (u32)verts->size();
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
      const protocol::MeshBlob* blobs[] = { letter.outline, letter.cap1, letter.cap2 };
      const protocol::MeshBlob* elem = blobs[(int)segment];

      // copy verts/indices
      u32 numIndices = elem->numIndices;
      u32 prevIndices = (u32)indices->size();
      u32 numVerts = elem->numVerts;
      u32 prevVerts = (u32)verts->size();
      indices->resize(indices->size() + numIndices);
      verts->resize(verts->size() + numVerts);

      for (u32 j = 0; j < numIndices; ++j)
      {
        (*indices)[j+prevIndices] = elem->indices[j] + prevVerts;
      }

      //copy(elem->indices, elem->indices + numIndices, indices->begin() + prevIndices);

      // get max vertex dimensions
      for (u32 j = 0; j < numVerts; ++j)
      {
        V3 v(elem->verts[j*3 + 0] + xOfs, elem->verts[j*3 + 1], elem->verts[j*3 + 2]);
        (*verts)[j+prevVerts] = v;
        vMin = Min(v, vMin);
        vMax = Max(v, vMax);
      }

      xOfs += letter.width * 1.05f;
    }

    lineInfo.push_back({
      triStart, (u32)verts->size(),
      vMin.x, vMax.x, vMin.y, vMax.y });
  }

  float xStart = lineInfo[0].xMin;
  float xEnd = lineInfo[0].xMax;
  float ySize = 0;
  for (size_t i = 1; i < lineInfo.size(); ++i)
  {
    xStart = min(xStart, lineInfo[i].xMin);
    xEnd = max(xEnd, lineInfo[i].xMax);
    ySize += lineInfo[i].yMax - lineInfo[i].yMin;
  }

  float sx = xEnd - xStart;
  float xCenter = sx / 2;
  float yOfs = ySize / 2;

  for (const RowDim& row : lineInfo)
  {
    float rx = row.xMax - row.xMin;
    float ry = row.yMax - row.yMin;
    float xRowOfs = (sx - rx) / 2;

    // loop over all the verts and center the text
    for (u32 i = row.triStart, e = row.triEnd; i < e; ++i)
    {
      (*verts)[i].x -= (xCenter - xRowOfs);
      // we first center the text around its local origin, and then move it
      // to the correct global position
      (*verts)[i].y += -ry / 2 + yOfs;
    }

    yOfs -= ry;
  }
}
