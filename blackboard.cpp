#include "blackboard.hpp"
#include "resource_manager.hpp"
#include "generated/input_buffer.hpp"
#include "generated/parse_base.hpp"

using namespace tano;
using namespace bristol;

Blackboard* Blackboard::_instance;

//------------------------------------------------------------------------------
bool Blackboard::Create(const char* filename)
{
  assert(!_instance);
  _instance = new Blackboard();
  return _instance->Init(filename);
}

//------------------------------------------------------------------------------
void Blackboard::Destory()
{
  delete exch_null(_instance);
}

//------------------------------------------------------------------------------
Blackboard& Blackboard::Instance()
{
  assert(_instance);
  return *_instance;
}

//------------------------------------------------------------------------------
bool Blackboard::Init(const char* filename)
{
  bool res;
  RESOURCE_MANAGER.AddFileWatch(filename, true, [&](const string& filename, void*)
  {
    Reset();
    vector<char> buf;
    RESOURCE_MANAGER.LoadFile(filename.c_str(), &buf);
    InputBuffer inputBuffer(buf);
    res = ParseBlackboard(inputBuffer);
    return res;
  });

  return res;
}

//------------------------------------------------------------------------------
void Blackboard::Reset()
{
  _floatVars.clear();
  _intVars.clear();
  _vec2Vars.clear();
  _vec3Vars.clear();
  _vec4Vars.clear();
}

//------------------------------------------------------------------------------
bool Blackboard::ParseBlackboard(InputBuffer& buf)
{
  static deque<string> namespaceStack;
  static auto fnFullName = [&](const string& str) {
    string res;
    for (auto it = namespaceStack.rbegin(); it != namespaceStack.rend(); ++it)
    {
      if (!res.empty())
        res += ".";
      res += *it;
    }

    if (!res.empty())
      res += ".";

    return res + str;
  };

  while (!buf.Eof())
  {
    string keyword;
    buf.SkipWhitespace();
    char peek;
    CHECKED_OP(buf.Peek(&peek));
    if (peek == '}')
      return true;

    CHECKED_OP(ParseIdentifier(buf, &keyword, false));
    buf.SkipWhitespace();

    string id;
    if (keyword == "namespace")
    {
      // get namespace, and add to namespace stack
      CHECKED_OP(ParseIdentifier(buf, &id, false));
      namespaceStack.push_back(id);

      buf.SkipWhitespace();
      CHECKED_OP(buf.Expect('{'));

      CHECKED_OP(ParseBlackboard(buf));

      buf.SkipWhitespace();
      CHECKED_OP(buf.Expect('}'));

      namespaceStack.pop_back();

    }
    else if (keyword == "int")
    {
      CHECKED_OP(ParseIdentifier(buf, &id, false));
      buf.SkipWhitespace();

      CHECKED_OP(buf.Expect('='));

      buf.SkipWhitespace();
      int value;
      ParseInt(buf, &value);
      AddIntVar(fnFullName(id), value);

      buf.SkipWhitespace();
      CHECKED_OP(buf.Expect(';'));
    }
    else if (keyword == "float")
    {
      CHECKED_OP(ParseIdentifier(buf, &id, false));
      buf.SkipWhitespace();

      CHECKED_OP(buf.Expect('='));

      buf.SkipWhitespace();
      float value;
      ParseFloat(buf, &value);
      AddFloatVar(fnFullName(id), value);

      buf.SkipWhitespace();
      CHECKED_OP(buf.Expect(';'));
    }
    else if (keyword == "vec2")
    {
      CHECKED_OP(ParseIdentifier(buf, &id, false));
      buf.SkipWhitespace();

      CHECKED_OP(buf.Expect('='));

      buf.SkipWhitespace();
      Vector2 value;
      ParseVec2(buf, &value);
      AddVec2Var(fnFullName(id), value);

      buf.SkipWhitespace();
      CHECKED_OP(buf.Expect(';'));
    }
    else if (keyword == "vec3")
    {
      CHECKED_OP(ParseIdentifier(buf, &id, false));
      buf.SkipWhitespace();

      CHECKED_OP(buf.Expect('='));

      buf.SkipWhitespace();
      Vector3 value;
      ParseVec3(buf, &value);
      AddVec3Var(fnFullName(id), value);

      buf.SkipWhitespace();
      CHECKED_OP(buf.Expect(';'));
    }
    else if (keyword == "vec4")
    {
      CHECKED_OP(ParseIdentifier(buf, &id, false));
      buf.SkipWhitespace();

      CHECKED_OP(buf.Expect('='));

      buf.SkipWhitespace();
      Vector4 value;
      ParseVec4(buf, &value);
      AddVec4Var(fnFullName(id), value);

      buf.SkipWhitespace();
      CHECKED_OP(buf.Expect(';'));
    }
    else
    {
      return false;
    }
  }

  return true;
}
