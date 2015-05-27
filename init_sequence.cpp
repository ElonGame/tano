#include "init_sequence.hpp"

using namespace tano;
using namespace bristol;

namespace
{
  const int MAX_DEPTH = 32;
  tano::InitSequence gSequenceStack[MAX_DEPTH];
  int gCurDepth = -1;
}

//------------------------------------------------------------------------------
void InitSequence::Enter()
{
  assert(gCurDepth < MAX_DEPTH);
  gCurDepth++;

  // set the max depth for all lower levels
  for (int i = 0; i <= gCurDepth; ++i)
    gSequenceStack[i]._maxDepth = gCurDepth;
}

//------------------------------------------------------------------------------
bool InitSequence::Exit()
{
  InitSequence* cur = &gSequenceStack[gCurDepth--];

  bool success = true;
  vector<string> failures;

  string indent;
  for (int i = 0; i <= cur->_maxDepth; ++i)
  {
    InitSequence* seq = &gSequenceStack[i];
    success &= seq->_failures.empty();
    if (gCurDepth == -1)
    {
      for (const InitFailure& f : seq->_failures)
      {
        failures.push_back(ToString("%s%s(%d): %s", indent.c_str(), f.file, f.line, f.str.c_str()));
      }
    }
    indent += "  ";
  }

  if (!failures.empty())
  {
    LOG_WARN_NAKED("=========================== INIT FAILURES ===========================");
    for (const string& str : failures)
    {
      LOG_WARN_NAKED(str.c_str());
    }
    LOG_WARN_NAKED("=====================================================================");
  }

  return success;
}

//------------------------------------------------------------------------------
void InitSequence::AddFailure(const char* file, int line, const char* str, bool fatal)
{
  if (gCurDepth < 0)
    return;

  InitSequence* seq = &gSequenceStack[gCurDepth];
  seq->_failures.push_back({ file, line, str });
  seq->_fatal |= fatal;
}

