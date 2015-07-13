#include "init_sequence.hpp"

using namespace tano;
using namespace bristol;

namespace
{
  const int MAX_DEPTH = 32;
  tano::InitSequence g_SequenceStack[MAX_DEPTH];
  int g_CurDepth = -1;
}

//------------------------------------------------------------------------------
void InitSequence::Enter()
{
  assert(g_CurDepth < MAX_DEPTH);
  g_CurDepth++;

  // set the max depth for all lower levels
  for (int i = 0; i <= g_CurDepth; ++i)
    g_SequenceStack[i]._maxDepth = g_CurDepth;
}

//------------------------------------------------------------------------------
bool InitSequence::Exit()
{
  InitSequence* cur = &g_SequenceStack[g_CurDepth--];

  bool success = true;
  vector<string> messages;

  // Check if the current or lower level has failed. If at the bottom level, create the
  // failure string.
  string indent;
  for (int i = 0; i <= cur->_maxDepth; ++i)
  {
    InitSequence* seq = &g_SequenceStack[i];

    // Look for any failure messages
    success &= seq->_success;
    if (g_CurDepth == -1 && !success)
    {
      for (const InitMessage& f : seq->_messages)
      {
        if (!seq->_success)
        {
          if (f.flags & InitMessage::Failure)
          {
            messages.push_back(ToString("%s%s(%d): [XX] %s", indent.c_str(), f.file, f.line, f.str.c_str()));
          }
          else
          {
            messages.push_back(ToString("%s%s(%d): %s", indent.c_str(), f.file, f.line, f.str.c_str()));
          }
        }
      }
      indent += "  ";
    }
  }

  if (!messages.empty())
  {
    LOG_WARN_NAKED("=========================== INIT FAILURES ===========================");
    for (const string& str : messages)
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
  if (g_CurDepth < 0)
    return;

  InitSequence* seq = &g_SequenceStack[g_CurDepth];
  seq->_messages.push_back({ file, line, str, InitMessage::Failure });
  seq->_fatal |= fatal;
  seq->_success = false;
}

//------------------------------------------------------------------------------
void InitSequence::AddLog(const char* file, int line, const char* str)
{
  if (g_CurDepth < 0)
    return;

  InitSequence* seq = &g_SequenceStack[g_CurDepth];
  seq->_messages.push_back({ file, line, str, InitMessage::Log });
}

