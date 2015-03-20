#include "init_sequence.hpp"

using namespace tano;
using namespace bristol;

namespace
{
  const int MAX_DEPTH = 32;
  tano::InitSequence SEQUENCE_STACK[MAX_DEPTH];
  int CUR_DEPTH = -1;
}

//------------------------------------------------------------------------------
void InitSequence::Enter()
{
  assert(CUR_DEPTH < MAX_DEPTH);
  CUR_DEPTH++;

  // set the max depth for all lower levels
  for (int i = 0; i <= CUR_DEPTH; ++i)
    SEQUENCE_STACK[i]._maxDepth = CUR_DEPTH;
}

//------------------------------------------------------------------------------
bool InitSequence::Exit()
{
  InitSequence* cur = &SEQUENCE_STACK[CUR_DEPTH--];

  bool success = true;
  vector<string> failures;

  string indent;
  for (int i = 0; i <= cur->_maxDepth; ++i)
  {
    InitSequence* seq = &SEQUENCE_STACK[i];
    success &= seq->_failures.empty();
    if (CUR_DEPTH == -1)
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
  InitSequence* seq = &SEQUENCE_STACK[CUR_DEPTH];
  seq->_failures.push_back({ file, line, str });
  seq->_fatal |= fatal;
}

