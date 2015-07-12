#include "blackboard.hpp"
#include "resource_manager.hpp"
#include "generated/input_buffer.hpp"
#include "generated/parse_base.hpp"
#include "init_sequence.hpp"

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
  RESOURCE_MANAGER.AddFileWatch(filename, true, [&](const string& filename)
  {
    vector<char> buf;
    RESOURCE_MANAGER.LoadFile(filename.c_str(), &buf);
    if (!buf.empty())
    {
      Reset();
      InputBuffer inputBuffer(buf);
      deque<string> namespaceStack;
      res = ParseBlackboard(inputBuffer, namespaceStack);
      return res;
    }
    else
    {
      return true;
    }
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
void Blackboard::SetNamespace(const string& ns)
{
  _curNamespace = ns;
}

//------------------------------------------------------------------------------
void Blackboard::ClearNamespace()
{
  _curNamespace.clear();
}

//------------------------------------------------------------------------------
void Blackboard::AddIntVar(const string& name, int value)
{
  _intVars[name] = value;
}

//------------------------------------------------------------------------------
void Blackboard::AddFloatVar(const string& name, float value)
{
  _floatVars[name] = value;
}

//------------------------------------------------------------------------------
void Blackboard::AddVec2Var(const string& name, const V2& value)
{
  _vec2Vars[name] = value;
}

//------------------------------------------------------------------------------
void Blackboard::AddVec3Var(const string& name, const V3& value)
{
  _vec3Vars[name] = value;
}

//------------------------------------------------------------------------------
void Blackboard::AddVec4Var(const string& name, const V4& value)
{
  _vec4Vars[name] = value;
}

//------------------------------------------------------------------------------
int Blackboard::GetIntVar(const string& name)
{
  return GetVar(name, _intVars);
}

//------------------------------------------------------------------------------
float Blackboard::GetFloatVar(const string& name)
{
  return GetVar(name, _floatVars);
}

//------------------------------------------------------------------------------
V2 Blackboard::GetVec2Var(const string& name)
{
  return GetVar(name, _vec2Vars);
}

//------------------------------------------------------------------------------
V3 Blackboard::GetVec3Var(const string& name)
{
  return GetVar(name, _vec3Vars);
}

//------------------------------------------------------------------------------
V4 Blackboard::GetVec4Var(const string& name)
{
  return GetVar(name, _vec4Vars);
}

//------------------------------------------------------------------------------
template<typename T>
T Blackboard::GetVar(const string& name, unordered_map<string, T>& vars)
{
  string fullname = _curNamespace.empty() ? name : _curNamespace + "." + name;
  auto it = vars.find(fullname);
  if (it == vars.end())
  {
    LOG_ERROR("Unknown variable: ", fullname);
    return T();
  }

  return it->second;
}

//------------------------------------------------------------------------------
bool Blackboard::ParseBlackboard(InputBuffer& buf, deque<string>& namespaceStack)
{
  BEGIN_INIT_SEQUENCE();
  auto fnFullName = [&](const string& str) {
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
    INIT_FATAL(buf.Peek(&peek));
    if (peek == '}')
      END_INIT_SEQUENCE();

    INIT_FATAL(ParseIdentifier(buf, &keyword, false));
    buf.SkipWhitespace();

    string id;
    if (keyword == "namespace")
    {
      // get namespace, and add to namespace stack
      INIT_FATAL(ParseIdentifier(buf, &id, false));
      namespaceStack.push_back(id);

      buf.SkipWhitespace();
      INIT_FATAL(buf.Expect('{'));

      INIT_FATAL(ParseBlackboard(buf, namespaceStack));

      buf.SkipWhitespace();
      INIT_FATAL(buf.Expect('}'));

      namespaceStack.pop_back();

    }
    else if (keyword == "int")
    {
      INIT_FATAL(ParseIdentifier(buf, &id, false));
      buf.SkipWhitespace();

      INIT_FATAL(buf.Expect('='));

      buf.SkipWhitespace();
      int value;
      ParseInt(buf, &value);
      AddIntVar(fnFullName(id), value);

      buf.SkipWhitespace();
      INIT_FATAL(buf.Expect(';'));
    }
    else if (keyword == "float")
    {
      INIT_FATAL(ParseIdentifier(buf, &id, false));
      buf.SkipWhitespace();

      INIT_FATAL(buf.Expect('='));

      buf.SkipWhitespace();
      float value;
      ParseFloat(buf, &value);
      AddFloatVar(fnFullName(id), value);

      buf.SkipWhitespace();
      INIT_FATAL(buf.Expect(';'));
    }
    else if (keyword == "vec2")
    {
      INIT_FATAL(ParseIdentifier(buf, &id, false));
      buf.SkipWhitespace();

      INIT_FATAL(buf.Expect('='));

      buf.SkipWhitespace();
      Vector2 tmp;
      ParseVec2(buf, &tmp);
      AddVec2Var(fnFullName(id), V2(tmp.x, tmp.y));

      buf.SkipWhitespace();
      INIT_FATAL(buf.Expect(';'));
    }
    else if (keyword == "vec3")
    {
      INIT_FATAL(ParseIdentifier(buf, &id, false));
      buf.SkipWhitespace();

      INIT_FATAL(buf.Expect('='));

      buf.SkipWhitespace();
      Vector3 tmp;
      ParseVec3(buf, &tmp);
      AddVec3Var(fnFullName(id), V3(tmp.x, tmp.y, tmp.z));

      buf.SkipWhitespace();
      INIT_FATAL(buf.Expect(';'));
    }
    else if (keyword == "vec4")
    {
      INIT_FATAL(ParseIdentifier(buf, &id, false));
      buf.SkipWhitespace();

      INIT_FATAL(buf.Expect('='));

      buf.SkipWhitespace();
      Vector4 tmp;
      ParseVec4(buf, &tmp);
      AddVec4Var(fnFullName(id), V4(tmp.x, tmp.y, tmp.z, tmp.w));

      buf.SkipWhitespace();
      INIT_FATAL(buf.Expect(';'));
    }
    else
    {
      INJECT_ERROR_FATAL("unknown keyword");
      END_INIT_SEQUENCE();
    }
  }

  END_INIT_SEQUENCE();
}
