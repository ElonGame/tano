#include "input_buffer.hpp"

namespace test {

//-----------------------------------------------------------------------------
InputBuffer::InputBuffer()
  : _buf(nullptr)
  , _idx(0)
  , _len(0)
{
}

//-----------------------------------------------------------------------------
InputBuffer::InputBuffer(const char* buf, size_t len)
  : _buf(buf)
  , _idx(0)
  , _len(len)
{
}

//-----------------------------------------------------------------------------
bool InputBuffer::Peek(char* res)
{
  if (Eof())
    return false;

  *res = _buf[_idx];
  return true;
}

//-----------------------------------------------------------------------------
void InputBuffer::Rewind(size_t len)
{
  _idx = len < _idx ? _idx - len : 0;
}

//-----------------------------------------------------------------------------
bool InputBuffer::Get(char* res)
{
  if (Eof())
    return false;

  *res = _buf[_idx++];
  return _idx != _len;
}

//-----------------------------------------------------------------------------
bool InputBuffer::Consume()
{
  if (Eof())
    return false;

  ++_idx;
  return true;
}

//-----------------------------------------------------------------------------
bool InputBuffer::ConsumeIf(char ch, bool* res)
{
  char tmp;
  CHECKED_OP(Peek(&tmp));

  bool localRes = ch == tmp;
  if (res)
    *res = localRes;

  return localRes ? Consume() : true;
}

//-----------------------------------------------------------------------------
bool InputBuffer::Eof()
{
  return _idx == _len;
}

//-----------------------------------------------------------------------------
bool InputBuffer::OneOf(const char* str, size_t len, char* res)
{
  char ch;
  *res = 0;
  CHECKED_OP(Peek(&ch));

  *res = -1;
  for (size_t i = 0; i < len; ++i)
  {
    if (ch == str[i])
    {
      CHECKED_OP(Consume());
      *res = ch;
      break;
    }
  }

  return true;
}

//-----------------------------------------------------------------------------
bool InputBuffer::Satifies(const function<bool(char)>& fn, char* res)
{
  char ch;
  CHECKED_OP(Peek(&ch));

  if (res)
    *res = ch;

  bool tmp = fn(ch);
  if (!tmp)
    return false;

  return Consume();
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
bool InputBuffer::Expect(char ch)
{
  char tmp;
  return Get(&tmp) && tmp == ch;
}

//-----------------------------------------------------------------------------
bool InputBuffer::IsDigit(char ch)
{
  return !!isdigit(ch);
}

//-----------------------------------------------------------------------------
bool InputBuffer::IsAlphaNum(char ch)
{
  return !!isalnum(ch);
}

//-----------------------------------------------------------------------------
bool InputBuffer::SkipUntil(char ch, bool consume)
{
  char tmp;
  while (Get(&tmp))
  {
    if (tmp == ch)
    {
      if (!consume)
        Rewind(1);
      return true;
    }
  }
  return false;
}

//-----------------------------------------------------------------------------
bool InputBuffer::SkipWhile(const function<bool(char)>& fn, size_t* end)
{
  size_t start = _idx;
  char ch;
  while (Get(&ch))
  {
    if (!fn(ch))
    {
      Rewind(1);
      *end = _idx;
      return true;
    }
  }
  return false;
}

//-----------------------------------------------------------------------------
bool InputBuffer::SubStr(size_t start, size_t len, string* res)
{
  if (start >= _len || start + len > _len)
    return false;

  res->assign(&_buf[start], len);
  return true;
}

}
