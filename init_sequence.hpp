#pragma once

namespace tano
{
  struct InitSequence
  {
    InitSequence();
    void AddFailure(const char* file, int line, const char* str, bool fatal);
    bool EndSequence();

    struct InitFailure
    {
      const char* file;
      int line;
      string str;
    };

    vector<InitFailure> _failures;
    bool _fatal;
  };

#define BEGIN_INIT_SEQUENCE() InitSequence __initSequence;
#define INIT(x) if (!__initSequence._fatal && !(x)) { __initSequence.AddFailure(__FILE__, __LINE__, #x, false); }
#define INJECT_ERROR(str) { __initSequence.AddFailure(__FILE__, __LINE__, str, false); }
#define INJECT_ERROR_FATAL(str) { __initSequence.AddFailure(__FILE__, __LINE__, str, true); }
#define INIT_RESOURCE(h, x) if (!__initSequence._fatal) { h = (x); if (!(h).IsValid()) { __initSequence.AddFailure(__FILE__, __LINE__, #x, false); } }
#define INIT_FATAL(x) if (!__initSequence._fatal && !(x)) { __initSequence.AddFailure(__FILE__, __LINE__, #x, true); }
#define END_INIT_SEQUENCE() return __initSequence.EndSequence();

}