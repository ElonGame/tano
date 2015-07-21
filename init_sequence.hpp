#pragma once

namespace tano
{
  struct InitSequence
  {
    static void Enter();
    static bool Exit();
    static void AddFailure(const char* file, int line, const char* str, bool fatal);
    static void AddLog(const char* file, int line, const char* str);

    struct InitMessage
    {
      enum Flag { Failure = 0x1, Log = 0x2 };
      const char* file;
      int line;
      string str;
      u32 flags;
    };

    vector<InitSequence> _children;
    vector<InitMessage> _messages;
    bool _success = true;
    bool _fatal = false;
    int _maxDepth = 0;
  };


#define BEGIN_INIT_SEQUENCE() InitSequence::Enter();
#define INIT(x) if (!(x)) { InitSequence::AddFailure(__FILE__, __LINE__, #x, false); }
#define INIT_HR(x) if (FAILED(x)) { InitSequence::AddFailure(__FILE__, __LINE__, #x, false); }
#define INIT_HR_FATAL(x) if (FAILED(x)) { InitSequence::AddFailure(__FILE__, __LINE__, #x, true); return InitSequence::Exit(); }
#define INJECT_ERROR(str) { InitSequence::AddFailure(__FILE__, __LINE__, str, false); }
#define INJECT_ERROR_FATAL(str) { InitSequence::AddFailure(__FILE__, __LINE__, str, true); }
#define INIT_RESOURCE(h, x) { h = (x); if (!(h).IsValid()) { InitSequence::AddFailure(__FILE__, __LINE__, #x, false); } }
#define INIT_FATAL_LOG(x, ...) if (!(x)) { InitSequence::AddFailure(__FILE__, __LINE__, #x, true); LOG_ERROR_NAKED(__VA_ARGS__); return InitSequence::Exit(); }
#define INIT_FATAL(x) if (!(x)) { InitSequence::AddFailure(__FILE__, __LINE__, #x, true); return InitSequence::Exit(); }
#define END_INIT_SEQUENCE() return InitSequence::Exit();

}