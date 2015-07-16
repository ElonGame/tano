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
bool FindScope(const InputBuffer& buf, const char* delim, InputBuffer* scope)
{
  // Note, the scope contains both the delimiters
  char open = delim[0];
  char close = delim[1];

  const char* start = &buf._buf[buf._idx];
  const char* end = &buf._buf[buf._idx + buf._len];
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

//------------------------------------------------------------------------------
bool Blackboard::ParseBlackboard(InputBuffer& buf, deque<string>& namespaceStack)
{
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
    buf.SkipWhitespace();

    string keyword;
    CHECKED_OP(ParseIdentifier(buf, &keyword, false));
    buf.SkipWhitespace();

    string id;
    if (keyword == "namespace")
    {
      // get namespace, and add to namespace stack
      CHECKED_OP(ParseIdentifier(buf, &id, false));
      namespaceStack.push_back(id);

      buf.SkipWhitespace();

      // grab the inner scope, and recurse into it
      InputBuffer inner;
      CHECKED_OP(FindScope(buf, "{}", &inner));
      // create a tmp buffer, that doesn't include the delims
      InputBuffer tmp = inner;
      tmp._buf++;
      tmp._len -= 2;
      char cc = tmp._buf[tmp._len];
      CHECKED_OP(ParseBlackboard(tmp, namespaceStack));

      // update the current buffer to skip the inner one
      buf._idx += inner._len;

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
      Vector2 tmp;
      ParseVec2(buf, &tmp);
      AddVec2Var(fnFullName(id), V2(tmp.x, tmp.y));

      buf.SkipWhitespace();
      CHECKED_OP(buf.Expect(';'));
    }
    else if (keyword == "vec3")
    {
      CHECKED_OP(ParseIdentifier(buf, &id, false));
      buf.SkipWhitespace();

      CHECKED_OP(buf.Expect('='));

      buf.SkipWhitespace();
      Vector3 tmp;
      ParseVec3(buf, &tmp);
      AddVec3Var(fnFullName(id), V3(tmp.x, tmp.y, tmp.z));

      buf.SkipWhitespace();
      CHECKED_OP(buf.Expect(';'));
    }
    else if (keyword == "vec4")
    {
      CHECKED_OP(ParseIdentifier(buf, &id, false));
      buf.SkipWhitespace();

      CHECKED_OP(buf.Expect('='));

      buf.SkipWhitespace();
      Vector4 tmp;
      ParseVec4(buf, &tmp);
      AddVec4Var(fnFullName(id), V4(tmp.x, tmp.y, tmp.z, tmp.w));

      buf.SkipWhitespace();
      CHECKED_OP(buf.Expect(';'));
    }
    else
    {
      LOG_ERROR("unknown keyword");
      return false;
    }

    buf.SkipWhitespace();
  }

  return true;
}
