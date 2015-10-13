#include <ctype.h>
#include <math.h>
#include "parse_base.hpp"
#include "input_buffer.hpp"
#include "output_buffer.hpp"

#pragma warning(disable : 4996)
using namespace std;

namespace parser
{
  struct ScopedRestore
  {
    ScopedRestore(InputBuffer* buf, bool* success) : buf(buf), success(success) { buf->SaveState(); }
    ~ScopedRestore()
    {
      // this relies on the fact the SET_PARSER_SUCCESS has been called, which will throw an
      // exception in the case of an error, so if success doesn't exist, then the parse must
      // have been successful.
      buf->RestoreState(success && !(*success));
    }
    InputBuffer* buf;
    bool* success;
  };

  //-----------------------------------------------------------------------------
  bool ParseBool(InputBuffer& buf, bool* success)
  {
    ScopedRestore r(&buf, success);
    SET_PARSER_SUCCESS(true);

    size_t start = buf._idx;
    buf.SkipWhile(InputBuffer::IsAlphaNum);
    size_t end = buf._idx;

    string str = buf.SubStr(start, end - start, success);
    if (str == "true")
      return true;

    if (str == "false")
      return false;

    SET_PARSER_SUCCESS(false);
    return false;
  }

  //-----------------------------------------------------------------------------
  float ParseFloat(InputBuffer& buf, bool* success)
  {
    ScopedRestore r(&buf, success);
    SET_PARSER_SUCCESS(true);

    char ch;
    buf.IsOneOf("-+", 2, &ch);
    bool neg = ch == '-';

    int numLeadingDigits = 0;
    int numTrailingDigits = 0;

    int whole = 0;
    // find whole part if one exists
    while (buf.Satifies(InputBuffer::IsDigit, &ch))
    {
      numLeadingDigits++;
      whole = whole * 10 + (ch - '0');
    }

    float res = (neg ? -1.f : 1.f) * whole;

    // look for a '.'
    if (buf.ConsumeIf('.'))
    {
      // fractional
      int frac = 0;
      int len = 0;
      while (buf.Satifies(InputBuffer::IsDigit, &ch))
      {
        ++len;
        frac = frac * 10 + (ch - '0');
        numTrailingDigits++;
      }

      if (len)
      {
        res += frac / powf(10.f, (float)len);
      }
    }

    bool validParse = numLeadingDigits > 0 || numTrailingDigits > 0;
    SET_PARSER_SUCCESS(validParse);
    return res;
  }

  //-----------------------------------------------------------------------------
  int ParseInt(InputBuffer& buf, bool* success)
  {
    ScopedRestore r(&buf, success);
    SET_PARSER_SUCCESS(true);

    char ch;
    buf.IsOneOf("-+", 2, &ch);
    bool neg = ch == '-';

    // read the first char, and make sure it's a digit
    if (!buf.Satifies(InputBuffer::IsDigit, &ch))
    {
      SET_PARSER_SUCCESS(false);
      return 0;
    }

    int val = ch - '0';
    while (buf.Satifies(InputBuffer::IsDigit, &ch))
    {
      val = val * 10 + (ch - '0');
    }

    return (neg ? -1 : 1) * val;
  }

  //-----------------------------------------------------------------------------
  template <int N>
  void ParseVec(InputBuffer& buf, float* res, bool* success)
  {
    ScopedRestore r(&buf, success);
    SET_PARSER_SUCCESS(true);

    // { x, y, z, w }
    buf.Expect('{', success);
    if (success && !(*success))
      return;

    for (int i = 0; i < N; ++i)
    {
      buf.SkipWhitespace();
      res[i] = ParseFloat(buf, success);
      if (success && !(*success))
        return;

      if (i != N - 1)
      {
        buf.SkipWhitespace();
        buf.Expect(',', success);
        if (success && !(*success))
          return;
      }
    }

    buf.SkipWhitespace();
    buf.Expect('}', success);
    if (success && !(*success))
      return;
  }

#if PARSER_WITH_VECTOR_TYPES
  //-----------------------------------------------------------------------------
  color ParseColor(InputBuffer& buf, bool* success)
  {
    ScopedRestore r(&buf, success);
    SET_PARSER_SUCCESS(true);

    float tmp[4];
    ParseVec<4>(buf, tmp, success);
    return color(tmp[0], tmp[1], tmp[2], tmp[3]);
  }

  //-----------------------------------------------------------------------------
  vec2 ParseVec2(InputBuffer& buf, bool* success)
  {
    ScopedRestore r(&buf, success);
    SET_PARSER_SUCCESS(true);

    float tmp[2];
    ParseVec<2>(buf, tmp, success);
    return vec2(tmp[0], tmp[1]);
  }

  //-----------------------------------------------------------------------------
  vec3 ParseVec3(InputBuffer& buf, bool* success)
  {
    ScopedRestore r(&buf, success);
    SET_PARSER_SUCCESS(true);

    float tmp[3];
    ParseVec<3>(buf, tmp, success);
    return vec3(tmp[0], tmp[1], tmp[2]);
  }

  //-----------------------------------------------------------------------------
  vec4 ParseVec4(InputBuffer& buf, bool* success)
  {
    ScopedRestore r(&buf, success);
    SET_PARSER_SUCCESS(true);

    float tmp[4];
    ParseVec<4>(buf, tmp, success);
    return vec4(tmp[0], tmp[1], tmp[2], tmp[3]);
  }
#endif

  //-----------------------------------------------------------------------------
  std::string ParseString(InputBuffer& buf, bool* success)
  {
    ScopedRestore r(&buf, success);
    SET_PARSER_SUCCESS(true);

    char ch;
    buf.SkipUntilOneOf("'\"", 2, &ch, true, success);
    if (success && !(*success))
      return string();

    size_t start = buf._idx;
    buf.SkipUntil(ch, true, success);
    if (success && !(*success))
      return string();

    size_t end = buf._idx;
    return buf.SubStr(start, end - start - 1, success);
  }

  //-----------------------------------------------------------------------------
  string ParseIdentifier(InputBuffer& buf, bool colon, bool* success)
  {
    // an identifier consists of 'id:', so we parse the id, and then find the trailing ':'
    size_t start = buf._idx;
    buf.SkipWhile(InputBuffer::IsAlphaNum);
    size_t end = buf._idx;

    if (colon)
    {
      // find the trailing ':'
      buf.SkipUntil(':', true, success);
      if (success && !(*success))
        return string();
    }

    return buf.SubStr(start, end - start, success);
  }

  //-----------------------------------------------------------------------------
  void Serialize(OutputBuffer& buf, bool value)
  {
    buf.EnsureCapacity(16);
    buf.Advance(sprintf(buf.Cur(), "%s", value ? "true" : "false"));
  }

  //-----------------------------------------------------------------------------
  void Serialize(OutputBuffer& buf, int value)
  {
    buf.EnsureCapacity(32);
    buf.Advance(sprintf(buf.Cur(), "%d", value));
  }

  //-----------------------------------------------------------------------------
  void Serialize(OutputBuffer& buf, float value)
  {
    buf.EnsureCapacity(32);
    buf.Advance(sprintf(buf.Cur(), "%f", value));
  }

  //-----------------------------------------------------------------------------
  void Serialize(OutputBuffer& buf, const string& value)
  {
    buf.EnsureCapacity(value.size() + 8);
    buf.Advance(sprintf(buf.Cur(), "'%s'", value.c_str()));
  }

#if PARSER_WITH_VECTOR_TYPES
  //-----------------------------------------------------------------------------
  void Serialize(OutputBuffer& buf, const color& value)
  {
    buf.EnsureCapacity(4 * 32);
    buf.Advance(sprintf(buf.Cur(), "{ %f, %f, %f, %f }", value.x, value.y, value.z, value.w));
  }

  //-----------------------------------------------------------------------------
  void Serialize(OutputBuffer& buf, const vec2& value)
  {
    buf.EnsureCapacity(2 * 32);
    buf.Advance(sprintf(buf.Cur(), "{ %f, %f }", value.x, value.y));
  }

  //-----------------------------------------------------------------------------
  void Serialize(OutputBuffer& buf, const vec3& value)
  {
    buf.EnsureCapacity(3 * 32);
    buf.Advance(sprintf(buf.Cur(), "{ %f, %f, %f }", value.x, value.y, value.z));
  }

  //-----------------------------------------------------------------------------
  void Serialize(OutputBuffer& buf, const vec4& value)
  {
    buf.EnsureCapacity(4 * 32);
    buf.Advance(sprintf(buf.Cur(), "{ %f, %f, %f, %f }", value.x, value.y, value.z, value.w));
  }
#endif
  //-----------------------------------------------------------------------------
  void Serialize(OutputBuffer& buf, int indent, const char* member, bool value)
  {
    int len = (int)strlen(member);
    buf.EnsureCapacity(len + indent + 16);
    buf.Advance(sprintf(buf.Cur(), "%*s: %s;\n", len + indent, member, value ? "true" : "false"));
  }

  //-----------------------------------------------------------------------------
  void Serialize(OutputBuffer& buf, int indent, const char* member, int value)
  {
    int len = (int)strlen(member);
    buf.EnsureCapacity(len + indent + 32);
    buf.Advance(sprintf(buf.Cur(), "%*s: %d;\n", len + indent, member, value));
  }

  //-----------------------------------------------------------------------------
  void Serialize(OutputBuffer& buf, int indent, const char* member, float value)
  {
    int len = (int)strlen(member);
    buf.EnsureCapacity(len + indent + 32);
    buf.Advance(sprintf(buf.Cur(), "%*s: %f;\n", len + indent, member, value));
  }

  //-----------------------------------------------------------------------------
  void Serialize(OutputBuffer& buf, int indent, const char* member, const string& value)
  {
    int len = (int)strlen(member);
    buf.EnsureCapacity(len + indent + value.size() + 32);
    buf.Advance(sprintf(buf.Cur(), "%*s: '%s';\n", len + indent, member, value.c_str()));
  }

#if PARSER_WITH_VECTOR_TYPES
  //-----------------------------------------------------------------------------
  void Serialize(OutputBuffer& buf, int indent, const char* member, const color& value)
  {
    int len = (int)strlen(member);
    buf.EnsureCapacity(len + indent + 4 * 32);
    buf.Advance(sprintf(
        buf.Cur(), "%*s: { %f, %f, %f, %f };\n", len + indent, member, value.x, value.y, value.z, value.w));
  }

  //-----------------------------------------------------------------------------
  void Serialize(OutputBuffer& buf, int indent, const char* member, const vec2& value)
  {
    int len = (int)strlen(member);
    buf.EnsureCapacity(len + indent + 2 * 32);
    buf.Advance(sprintf(buf.Cur(), "%*s: { %f, %f };\n", len + indent, member, value.x, value.y));
  }

  //-----------------------------------------------------------------------------
  void Serialize(OutputBuffer& buf, int indent, const char* member, const vec3& value)
  {
    int len = (int)strlen(member);
    buf.EnsureCapacity(len + indent + 3 * 32);
    buf.Advance(
        sprintf(buf.Cur(), "%*s: { %f, %f, %f };\n", len + indent, member, value.x, value.y, value.z));
  }

  //-----------------------------------------------------------------------------
  void Serialize(OutputBuffer& buf, int indent, const char* member, const vec4& value)
  {
    int len = (int)strlen(member);
    buf.EnsureCapacity(len + indent + 4 * 32);
    buf.Advance(sprintf(
        buf.Cur(), "%*s: { %f, %f, %f, %f };\n", len + indent, member, value.x, value.y, value.z, value.w));
  }
#endif
  //-----------------------------------------------------------------------------

}
