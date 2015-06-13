#pragma once

namespace tano
{
  // TODO: make a thread local stack of these guys, to make the output nicer on failures
  struct InitSequence
  {
    static void Enter();
    static bool Exit();
    static void AddFailure(const char* file, int line, const char* str, bool fatal);

    struct InitFailure
    {
      const char* file;
      int line;
      string str;
    };

    vector<InitSequence> _children;
    vector<InitFailure> _failures;
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
#define INIT_FATAL(x) if (!(x)) { InitSequence::AddFailure(__FILE__, __LINE__, #x, true); return InitSequence::Exit(); }
#define END_INIT_SEQUENCE() return InitSequence::Exit();

}