#pragma once
#include <vector>
#include <deque>
namespace parser
{

  struct ParseExpcetion : std::exception
  {
    ParseExpcetion(const char* e) : std::exception(e) {}
  };

#define SET_PARSER_SUCCESS(s)                                                                      \
  do                                                                                               \
  {                                                                                                \
    if (s)                                                                                         \
    {                                                                                              \
      if (success)                                                                                 \
        *success = true;                                                                           \
    }                                                                                              \
    else                                                                                           \
    {                                                                                              \
      if (success)                                                                                 \
        *success = false;                                                                          \
      else                                                                                         \
        throw ParseExpcetion(__FUNCTION__);                                                        \
    }                                                                                              \
  } while (false);

#define CHECKED_OP(x)                                                                              \
  do                                                                                               \
  {                                                                                                \
    if (!(x))                                                                                      \
      return false;                                                                                \
  } while (false);

  struct InputBuffer
  {
    InputBuffer();
    InputBuffer(const char* buf, size_t len);
    InputBuffer(const std::vector<char>& buf);

    // The idea behind the functions is that everything where you expect to find something will return
    // an error, either in the success ptr, or by throwing an exception. Functions that are purely for
    // testing will return false on errors (EOF).
    char Peek(bool* success = nullptr);
    char Get(bool* success = nullptr);
    void Consume(bool* success = nullptr);

    bool Expect(char ch, bool* success = nullptr);
    void SkipUntil(char ch, bool consume, bool* success = nullptr);
    void SkipUntilOneOf(const char* str, size_t len, char* res, bool consume, bool* success = nullptr);
    std::string SubStr(size_t start, size_t len, bool* success = nullptr);

    void Rewind(size_t len);

    bool IsOneOf(const char* str, size_t len, char* res);
    bool IsOneOfIdx(const char* str, size_t len, int* res);
    void SkipWhile(const std::function<bool(char)>& fn);
    bool Satifies(const std::function<bool(char)>& fn, char* out);
    void SkipWhitespace();

    static bool IsDigit(char ch);
    static bool IsAlphaNum(char ch);

    bool ConsumeIf(char ch);
    bool Eof();

    bool InnerScope(const char* delim, InputBuffer* scope);

    void SaveState();
    void RestoreState(bool fullRestore=true);

    std::deque<size_t> _saveStack;

    const char* _buf;
    size_t _idx;
    size_t _len;
  };
}
