#pragma once
#pragma warning(disable : 4996)
#include "output_buffer.hpp"

#if PARSER_WITH_VECTOR_TYPES
#endif

#if PARSER_WITH_MATRIX_TYPES
#endif

namespace parser
{
  struct InputBuffer;

  // If a success pointer is passed in, then it's used to indicate the result, so the
  // functions can be used for probing. Otherwise an exception (ugh!!) is thrown if the
  // parse fails.
  bool ParseBool(InputBuffer& buf, bool* success = nullptr);
  float ParseFloat(InputBuffer& buf, bool* success = nullptr);
  int ParseInt(InputBuffer& buf, bool* success = nullptr);
  std::string ParseString(InputBuffer& buf, bool* success = nullptr);
  std::string ParseIdentifier(InputBuffer& buf, bool colon = true, bool* success = nullptr);

#if PARSER_WITH_VECTOR_TYPES
  color ParseColor(InputBuffer& buf, bool* success = nullptr);
  vec2 ParseVec2(InputBuffer& buf, bool* success = nullptr);
  vec3 ParseVec3(InputBuffer& buf, bool* success = nullptr);
  vec4 ParseVec4(InputBuffer& buf, bool* success = nullptr);
#endif

  void Serialize(OutputBuffer& buf, bool value);
  void Serialize(OutputBuffer& buf, int value);
  void Serialize(OutputBuffer& buf, float value);
  void Serialize(OutputBuffer& buf, const std::string& value);
#if PARSER_WITH_VECTOR_TYPES
  void Serialize(OutputBuffer& buf, const color& value);
  void Serialize(OutputBuffer& buf, const vec2& value);
  void Serialize(OutputBuffer& buf, const vec3& value);
  void Serialize(OutputBuffer& buf, const vec4& value);
#endif
  void Serialize(OutputBuffer& buf, int indent, const char* member, bool value);
  void Serialize(OutputBuffer& buf, int indent, const char* member, int value);
  void Serialize(OutputBuffer& buf, int indent, const char* member, float value);
  void Serialize(OutputBuffer& buf, int indent, const char* member, const std::string& value);
#if PARSER_WITH_VECTOR_TYPES
  void Serialize(OutputBuffer& buf, int indent, const char* member, const color& value);
  void Serialize(OutputBuffer& buf, int indent, const char* member, const vec2& value);
  void Serialize(OutputBuffer& buf, int indent, const char* member, const vec3& value);
  void Serialize(OutputBuffer& buf, int indent, const char* member, const vec4& value);
#endif

  template <typename T>
  void Serialize(OutputBuffer& buf, int indent, const char* member, const std::vector<T>& value)
  {
    int len = (int)strlen(member);
    buf.EnsureCapacity(len + indent + 16 + 8 * value.size());
    buf._ofs += sprintf(buf.Cur(), "%*s: [", len + indent, member);
    for (size_t i = 0, e = value.size(); i < e; ++i)
    {
      Serialize(buf, value[i]);
      if (i != e - 1)
      {
        buf._ofs += sprintf(buf.Cur(), ", ");
      }
    }
    buf._ofs += sprintf(buf.Cur(), "];\n");
  }
}
