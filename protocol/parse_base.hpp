#pragma once
#pragma warning(disable: 4996)
#include "output_buffer.hpp"

namespace test {

struct InputBuffer;
//struct OutputBuffer;

bool ParseBool(InputBuffer& buf, bool* res);
bool ParseFloat(InputBuffer& buf, float* res);
bool ParseInt(InputBuffer& buf, int* res);
bool ParseString(InputBuffer& buf, string* res);
bool ParseIdentifier(InputBuffer& buf, string* res);

void Serialize(OutputBuffer& buf, bool value);
void Serialize(OutputBuffer& buf, int value);
void Serialize(OutputBuffer& buf, float value);
void Serialize(OutputBuffer& buf, const string& value);
void Serialize(OutputBuffer& buf, int indent, const char* member, bool value);
void Serialize(OutputBuffer& buf, int indent, const char* member, int value);
void Serialize(OutputBuffer& buf, int indent, const char* member, float value);
void Serialize(OutputBuffer& buf, int indent, const char* member, const string& value);

template <typename T>
void Serialize(OutputBuffer& buf, int indent, const char* member, const vector<T>& value)
{
  int len = strlen(member);
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
