#include "input_buffer.hpp"

#define SET_SUCCESS(s)                                                                             \

using namespace std;

namespace parser
{
  //-----------------------------------------------------------------------------
  InputBuffer::InputBuffer() : _buf(nullptr), _idx(0), _len(0) {}

  //-----------------------------------------------------------------------------
  InputBuffer::InputBuffer(const char* buf, size_t len) : _buf(buf), _idx(0), _len(len) {}

  //-----------------------------------------------------------------------------
  InputBuffer::InputBuffer(const vector<char>& buf) : _buf(buf.data()), _idx(0), _len(buf.size()) {}

  //-----------------------------------------------------------------------------
  char InputBuffer::Peek(bool* success)
  {
    SET_PARSER_SUCCESS(!Eof());
    if (Eof())
      return 0;

    return _buf[_idx];
  }

  //-----------------------------------------------------------------------------
  void InputBuffer::Rewind(size_t len) { _idx = len < _idx ? _idx - len : 0; }

  //-----------------------------------------------------------------------------
  char InputBuffer::Get(bool* success)
  {
    SET_PARSER_SUCCESS(!Eof());
    if (Eof())
      return 0;

    return _buf[_idx++];
  }

  //-----------------------------------------------------------------------------
  void InputBuffer::Consume(bool* success)
  {
    SET_PARSER_SUCCESS(!Eof());
    if (Eof())
      return;

    ++_idx;
  }

  //-----------------------------------------------------------------------------
  bool InputBuffer::ConsumeIf(char ch)
  {    
    if (Eof())
      return false;

    bool consume = Peek() == ch;
    if (consume)
      Consume();
    return consume;
  }

  //-----------------------------------------------------------------------------
  bool InputBuffer::Eof() { return _idx == _len; }

  //-----------------------------------------------------------------------------
  bool InputBuffer::IsOneOf(const char* str, size_t len, char* res)
  {
    if (Eof())
      return false;

    *res = 0;
    int idx;
    if (!IsOneOfIdx(str, len, &idx))
      return false;

    *res = str[idx];
    return true;
  }

  //-----------------------------------------------------------------------------
  bool InputBuffer::IsOneOfIdx(const char* str, size_t len, int* res)
  {
    if (Eof())
      return false;

    *res = -1;
    char ch = Peek();

    for (size_t i = 0; i < len; ++i)
    {
      if (ch == str[i])
      {
        Consume();
        *res = (int)i;
        return true;
      }
    }
    return false;
  }

  //-----------------------------------------------------------------------------
  bool InputBuffer::Satifies(const function<bool(char)>& fn, char* out)
  {
    if (Eof())
      return false;

    char ch = Peek();
    bool satisfies = fn(ch);
    if (satisfies)
    {
      *out = ch;
      Consume();
    }
    return satisfies;
  }

  //-----------------------------------------------------------------------------
  void InputBuffer::SkipWhitespace()
  {
    while (true)
    {
      char ch = _buf[_idx];
      if (ch == ' ' || ch == '\t' || ch == '\r' || ch == '\n')
      {
        _idx++;
        if (_idx == _len)
          return;
      }
      else
      {
        return;
      }
    }
  }

  //-----------------------------------------------------------------------------
  bool InputBuffer::Expect(char ch, bool* success)
  {
    return Get(success) == ch;
  }

  //-----------------------------------------------------------------------------
  bool InputBuffer::IsDigit(char ch) { return !!isdigit(ch); }

  //-----------------------------------------------------------------------------
  bool InputBuffer::IsAlphaNum(char ch) { return !!isalnum(ch) || ch == '_'; }

  //-----------------------------------------------------------------------------
  void InputBuffer::SkipUntil(char ch, bool consume, bool* success)
  {
    SET_PARSER_SUCCESS(true);
    while (!Eof())
    {
      char tmp = Get();
      if (tmp == ch)
      {
        if (!consume)
          Rewind(1);
        return;
      }
    }

    SET_PARSER_SUCCESS(false);
  }

  //-----------------------------------------------------------------------------
  void InputBuffer::SkipUntilOneOf(const char* str, size_t len, char* res, bool consume, bool* success)
  {
    SET_PARSER_SUCCESS(true);
    while (!Eof())
    {
      char tmp = Get();
      for (size_t i = 0; i < len; ++i)
      {
        if (tmp == str[i])
        {
          if (res)
            *res = tmp;
          if (!consume)
            Rewind(1);
          return;
        }
      }
    }
    SET_PARSER_SUCCESS(false);
  }

  //-----------------------------------------------------------------------------
  void InputBuffer::SkipWhile(const function<bool(char)>& fn)
  {
    while (!Eof())
    {
      char ch = Get();
      if (!fn(ch))
      {
        Rewind(1);
        return;
      }
    }
  }

  //-----------------------------------------------------------------------------
  string InputBuffer::SubStr(size_t start, size_t len, bool* success)
  {
    bool valid = start + len <= _len;
    SET_PARSER_SUCCESS(valid);
    string res;
    if (valid)
    {
      res.assign(&_buf[start], len);
    }

    return res;
  }

  //-----------------------------------------------------------------------------
  bool InputBuffer::InnerScope(const char* delim, InputBuffer* scope)
  {
    // Note, the scope contains both the delimiters
    char open = delim[0];
    char close = delim[1];

    const char* start = &_buf[_idx];
    const char* end = &_buf[_idx + _len];
    const char* cur = start;

    // find opening delimiter
    while (cur < end)
    {
      if (*cur++ == open)
        break;
    }

    if (cur == end)
      return false;

    scope->_buf = cur - 1;
    scope->_idx = 0;

    // find closing
    int cnt = 0;
    while (cur < end)
    {
      char ch = *cur;
      if (ch == close)
      {
        if (cnt == 0)
        {
          // found the scope
          scope->_len = cur - scope->_buf + 1;
          return true;
        }
        else
        {
          --cnt;
        }
      }
      else if (ch == open)
      {
        ++cnt;
      }
      ++cur;
    }
    return false;
  }

  //-----------------------------------------------------------------------------
  void InputBuffer::SaveState()
  {
    _saveStack.push_back(_idx);
  }

  //-----------------------------------------------------------------------------
  void InputBuffer::RestoreState(bool fullRestore)
  {
    if (fullRestore)
    {
      _idx = _saveStack.back();
    }
    _saveStack.pop_back();
  }
}
