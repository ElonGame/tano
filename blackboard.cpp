#include "blackboard.hpp"
#include "resource_manager.hpp"
#include "init_sequence.hpp"
#if WITH_IMGUI
#include "imgui/imgui_internal.h"
#include "imgui_helpers.hpp"
#endif

using namespace tano;
using namespace bristol;

Blackboard* tano::g_Blackboard = nullptr;

//------------------------------------------------------------------------------
Blackboard::~Blackboard()
{
  Reset();
}

//------------------------------------------------------------------------------
bool Blackboard::Create(const char* filename, const char* datafile)
{
  assert(!g_Blackboard);
  g_Blackboard = new Blackboard();
  return g_Blackboard->Init(filename, datafile);
}

//------------------------------------------------------------------------------
void Blackboard::Destory()
{
#if WITH_BLACKBOARD_SAVE && WITH_BLACKBOARD_TCP && WITH_UNPACKED_RESOUCES
  _instance->SaveData();
#endif
  delete exch_null(g_Blackboard);
}

//------------------------------------------------------------------------------
bool Blackboard::Init(const char* filename, const char* datafile)
{
  _filename = filename;
  _datafile = datafile;

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
          if (res)
          {
            LoadData();
            _triggeredIds.clear();
          }
          _curExpr = min(_curExpr, (int)_expressionNames.size());
          return res;
        }
        else
        {
          return true;
        }
      });

#if WITH_BLACKBOARD_TCP
  WSAStartup(MAKEWORD(2, 2), &_wsaData);
  Connect("127.0.0.1", "1337");
#endif

  return res;
}

//------------------------------------------------------------------------------
bool Blackboard::IsDirtyTrigger(void* id)
{
  // has the blackboard changed since the id last polled?
  if (_triggeredIds.count(id) > 0)
    return false;

  _triggeredIds.insert(id);
  return true;
}

//------------------------------------------------------------------------------
void Blackboard::Reset()
{
  AssocDelete(&_floatVars);
  AssocDelete(&_intVars);
  AssocDelete(&_vec2Vars);
  AssocDelete(&_vec3Vars);
  AssocDelete(&_vec4Vars);

  _expressions.clear();
  _expressionNames.clear();
}

//------------------------------------------------------------------------------
#if WITH_BLACKBOARD_SAVE && WITH_BLACKBOARD_TCP && WITH_UNPACKED_RESOUCES
void Blackboard::SaveData()
{
  FILE* f = RESOURCE_MANAGER.OpenWriteFile(_datafile.c_str());

  int numKeysframes = (int)_rawKeyframes.size();
  RESOURCE_MANAGER.WriteFile(f, (const char*)&numKeysframes, sizeof(int));

  // save all the keyframe rawdata
  for (auto kv : _rawKeyframes)
  {
    const vector<char>& buf = kv.second;
    RESOURCE_MANAGER.WriteFile(f, buf.data(), (int)buf.size());
  }

  RESOURCE_MANAGER.CloseFile(f);
}
#endif

//------------------------------------------------------------------------------
void Blackboard::LoadData()
{
  vector<char> buf;
  if (!RESOURCE_MANAGER.LoadFile(_datafile.c_str(), &buf))
    return;

  ProcessAnimationBuffer(buf.data(), (int)buf.size());
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
  _intVars[name] = new Keyframes<int>(value);
}

//------------------------------------------------------------------------------
void Blackboard::AddFloatVar(const string& name, float value)
{
  _floatVars[name] = new Keyframes<float>(value);
}

//------------------------------------------------------------------------------
void Blackboard::AddVec2Var(const string& name, const vec2& value)
{
  _vec2Vars[name] = new Keyframes<vec2>(value);
}

//------------------------------------------------------------------------------
void Blackboard::AddVec3Var(const string& name, const vec3& value)
{
  _vec3Vars[name] = new Keyframes<vec3>(value);
}

//------------------------------------------------------------------------------
void Blackboard::AddVec4Var(const string& name, const vec4& value)
{
  _vec4Vars[name] = new Keyframes<vec4>(value);
}

//------------------------------------------------------------------------------
int Blackboard::GetIntVar(const string& name)
{
  return GetVar(name, 0, _intVars);
}

//------------------------------------------------------------------------------
float Blackboard::GetFloatVar(const string& name)
{
  return GetVar(name, 0, _floatVars);
}

//------------------------------------------------------------------------------
vec2 Blackboard::GetVec2Var(const string& name)
{
  return GetVar(name, 0, _vec2Vars);
}

//------------------------------------------------------------------------------
vec3 Blackboard::GetVec3Var(const string& name)
{
  return GetVar(name, 0, _vec3Vars);
}

//------------------------------------------------------------------------------
vec4 Blackboard::GetVec4Var(const string& name)
{
  return GetVar(name, 0, _vec4Vars);
}

//------------------------------------------------------------------------------
string Blackboard::GetFullName(const string& name)
{
  if (name.find('.') != string::npos)
  {
    // if the name contains a '.', assume it's a fully qualified name, and reset the
    // current namespace
    _curNamespace.clear();
  }

  return _curNamespace.empty() ? name : _curNamespace + "." + name;
}

//------------------------------------------------------------------------------
template <typename T>
T Blackboard::GetVar(const string& name, float t, unordered_map<string, Keyframes<T>*>& vars)
{
  string fullname = GetFullName(name);
  auto it = vars.find(fullname);
  if (it == vars.end())
  {
    LOG_ERROR("Unknown variable: ", fullname);
    return T();
  }

  return GetValueAtTime(t, it->second);
}

//------------------------------------------------------------------------------
bool Blackboard::ParseBlackboard(InputBuffer& buf, deque<string>& namespaceStack)
{
  auto fnFullName = [&](const string& str)
  {
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

    string keyword = ParseIdentifier(buf, false);
    buf.SkipWhitespace();

    string id;
    if (keyword == "namespace")
    {
      // get namespace, and add to namespace stack
      id = ParseIdentifier(buf, false);
      namespaceStack.push_back(id);

      buf.SkipWhitespace();

      // grab the inner scope, and recurse into it
      InputBuffer inner;
      CHECKED_OP(buf.InnerScope("{}", &inner));
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
      id = ParseIdentifier(buf, false);
      buf.SkipWhitespace();

      CHECKED_OP(buf.Expect('='));

      buf.SkipWhitespace();
      AddIntVar(fnFullName(id), ParseInt(buf));

      buf.SkipWhitespace();
      CHECKED_OP(buf.Expect(';'));
    }
    else if (keyword == "float")
    {
      id = ParseIdentifier(buf, false);
      buf.SkipWhitespace();

      CHECKED_OP(buf.Expect('='));

      buf.SkipWhitespace();
      AddFloatVar(fnFullName(id), ParseFloat(buf));

      buf.SkipWhitespace();
      CHECKED_OP(buf.Expect(';'));
    }
    else if (keyword == "vec2")
    {
      id = ParseIdentifier(buf, false);
      buf.SkipWhitespace();

      CHECKED_OP(buf.Expect('='));

      buf.SkipWhitespace();
      AddVec2Var(fnFullName(id), ParseVec2(buf));

      buf.SkipWhitespace();
      CHECKED_OP(buf.Expect(';'));
    }
    else if (keyword == "vec3")
    {
      id = ParseIdentifier(buf, false);
      buf.SkipWhitespace();

      CHECKED_OP(buf.Expect('='));

      buf.SkipWhitespace();
      AddVec3Var(fnFullName(id), ParseVec3(buf));

      buf.SkipWhitespace();
      CHECKED_OP(buf.Expect(';'));
    }
    else if (keyword == "vec4")
    {
      id = ParseIdentifier(buf, false);
      buf.SkipWhitespace();

      CHECKED_OP(buf.Expect('='));

      buf.SkipWhitespace();
      AddVec4Var(fnFullName(id), ParseVec4(buf));

      buf.SkipWhitespace();
      CHECKED_OP(buf.Expect(';'));
    }
    else if (keyword == "expr")
    {
      id = ParseIdentifier(buf, false);
      buf.SkipWhitespace();

      CHECKED_OP(buf.Expect('='));

      buf.SkipWhitespace();
      string str = ParseString(buf);

      // parse the expression
      vector<eval::Token> expr;
      eval::Parse(str.c_str(), &expr);
      _expressions[fnFullName(id)] = Expression{str, expr};
      _expressionNames.push_back(fnFullName(id));

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

//------------------------------------------------------------------------------
template <typename T>
int CopyFromBuffer(const char* buf, T* out)
{
  memcpy(out, buf, sizeof(T));
  return sizeof(T);
}

//------------------------------------------------------------------------------
template <typename T>
int Blackboard::LoadKeyframes(const char* buf, const string& name, unordered_map<string, Keyframes<T>*>* res)
{
  const char* org = buf;

  Keyframes<T>* k = new Keyframes<T>();

  buf += CopyFromBuffer(buf, &k->firstValue);
  buf += CopyFromBuffer(buf, &k->lastValue);

  buf += CopyFromBuffer(buf, &k->firstTime);
  buf += CopyFromBuffer(buf, &k->lastTime);

  buf += CopyFromBuffer(buf, &k->sampleStep);

  int numValues;
  buf += CopyFromBuffer(buf, &numValues);
  k->values.resize(numValues);
  memcpy(k->values.data(), buf, numValues * sizeof(T));
  buf += numValues * sizeof(T);

  (*res)[name] = k;

  return (int)(buf - org);
}

//------------------------------------------------------------------------------
float Blackboard::GetFloatVar(const string& name, float t)
{
  return GetVar(name, t, _floatVars);
}

//------------------------------------------------------------------------------
vec2 Blackboard::GetVec2Var(const string& name, float t)
{
  return GetVar(name, t, _vec2Vars);
}

//------------------------------------------------------------------------------
vec3 Blackboard::GetVec3Var(const string& name, float t)
{
  return GetVar(name, t, _vec3Vars);
}

//------------------------------------------------------------------------------
float Blackboard::GetExpr(const string& name, eval::Environment* env)
{
  string fullname = GetFullName(name);
  auto it = _expressions.find(fullname);
  if (it == _expressions.end())
  {
    LOG_ERROR("Unknown expression: ", fullname);
    return 0;
  }

  eval::Evaluator e;
  return e.Evaluate(it->second.tokens, env);
}

//------------------------------------------------------------------------------
template <typename T>
T Blackboard::GetValueAtTime(float t, const Keyframes<T>* keyframes)
{
  // if both first and last time are 0, then the animation isn't keyframed, so
  // we need to skip this optimization
  if (keyframes->values.empty() || (keyframes->firstTime != 0 && keyframes->lastTime != 0))
  {
    if (t <= keyframes->firstTime)
      return keyframes->firstValue;

    if (t >= keyframes->lastTime)
      return keyframes->lastValue;
  }

  float relT = t - keyframes->firstTime;

  float step = keyframes->sampleStep;
  int idx0 = min((int)(relT / step), (int)keyframes->values.size() - 1);
  int idx1 = min(idx0 + 1, (int)keyframes->values.size() - 1);

  // calc lerp term
  float frac = (relT - idx0 * step) / step;
  return lerp(keyframes->values[idx0], keyframes->values[idx1], frac);
}

//------------------------------------------------------------------------------
void Blackboard::ProcessAnimationBuffer(const char* buf, int bufSize)
{
  int numKeysframes;
  buf += CopyFromBuffer(buf, &numKeysframes);

  for (int keyframeIdx = 0; keyframeIdx < numKeysframes; ++keyframeIdx)
  {
    const char* bufStart = buf;
    int namelen;
    buf += CopyFromBuffer(buf, &namelen);
    string name;
    name.resize(namelen);
    memcpy((void*)name.data(), buf, namelen);
    buf += namelen;

    int dims;
    buf += CopyFromBuffer(buf, &dims);

    switch (dims)
    {
    case 1: buf += LoadKeyframes(buf, name, &_floatVars); break;
    case 2: buf += LoadKeyframes(buf, name, &_vec2Vars); break;
    case 3: buf += LoadKeyframes(buf, name, &_vec3Vars); break;
    }

    const char* bufEnd = buf;
#if WITH_BLACKBOARD_SAVE
    _rawKeyframes[name] = vector<char>(bufStart, bufEnd);
#endif
  }
}

//------------------------------------------------------------------------------
#if WITH_BLACKBOARD_TCP
#pragma comment(lib, "ws2_32.lib")

//------------------------------------------------------------------------------
void Blackboard::SetBlockingIo(bool blocking)
{
  u_long v = !blocking;
  ioctlsocket(_sockfd, FIONBIO, &v);
}

//------------------------------------------------------------------------------
void Blackboard::Disconnect()
{
  _connected = false;
}

//------------------------------------------------------------------------------
bool Blackboard::Connect(const char* host, const char* service)
{
  _host = host;
  _serviceName = service;

  if (_sockfd != 0)
    closesocket(_sockfd);

  addrinfo hints;
  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;

  // Connect to host
  _addrinfo = nullptr;
  int r1 = getaddrinfo(host, service, &hints, &_addrinfo);
  if (r1 != 0)
  {
    LOG_WARN("getaddrinfo err: ", r1);
    return false;
  }

  _sockfd = socket(_addrinfo->ai_family, _addrinfo->ai_socktype, _addrinfo->ai_protocol);

  // Set up for non-blocking connect
  SetBlockingIo(false);
  _readState = ReadState::Connecting;

  return true;
}

//------------------------------------------------------------------------------
void Blackboard::Process()
{
  // Handle non-blocking connect
  if (_readState == Blackboard::ReadState::Connecting)
  {
    int res = connect(_sockfd, _addrinfo->ai_addr, (int)_addrinfo->ai_addrlen);
    if (res == 0)
    {
      // done connecting
      _connected = true;
      _readState = Blackboard::ReadState::ReadHeader;
    }
    else
    {
      int lastError = WSAGetLastError();
      if (lastError == WSAEWOULDBLOCK || lastError == WSAEALREADY)
      {
        // still connecting
        _connected = false;
        return;
      }
      else if (lastError == WSAEISCONN)
      {
        // done connecting
        _connected = true;
        _readState = Blackboard::ReadState::ReadHeader;
      }
      else
      {
        _connected = false;
        return;
      }
    }
  }

  if (!_connected)
    return;

  int res =
      recv(_sockfd, _readBuffer.buf + _readBuffer.writeOfs, Buffer::BUFFER_SIZE - _readBuffer.writeOfs, 0);
  if (res <= 0)
  {
    // TODO, we're in async mode, so is -1 really an error?
    if (res == 0)
    {
      // 0 indicates orderly shutdown, -1 indicates an error
      Disconnect();
    }
    return;
  }

  _readBuffer.writeOfs += res;

  // keep processing until we don't have a full frame
  while (true)
  {
    if (_readState == Blackboard::ReadState::ReadHeader)
    {
      if (_readBuffer.writeOfs >= sizeof(Header))
      {
        memcpy((void*)&_header, _readBuffer.buf, sizeof(Header));
        _readState = Blackboard::ReadState::ReadPayload;
      }
      else
      {
        return;
      }
    }

    if (_readState == Blackboard::ReadState::ReadPayload)
    {
      if (_readBuffer.writeOfs >= _header.payloadSize)
      {
        // got a full frame, so process it
        char* buf = _readBuffer.buf;
        int headerSize = sizeof(Header);
        int bufSize = *(int*)buf - headerSize;
        ProcessAnimationBuffer(buf + headerSize, bufSize);

        // copy the rest of the buffer to the second buffer, reset pointers,
        // and ping pong
        memcpy(_readBuffer.dbl, _readBuffer.buf + _header.payloadSize,
            _readBuffer.writeOfs - _header.payloadSize);

        _readBuffer.readOfs = 0;
        _readBuffer.writeOfs = _readBuffer.writeOfs - _header.payloadSize;
        _readState = Blackboard::ReadState::ReadHeader;
        swap(_readBuffer.buf, _readBuffer.dbl);
      }
      else
      {
        return;
      }
    }
  }
}

#endif

#if WITH_IMGUI && WITH_EXPRESSION_EDITOR
//------------------------------------------------------------------------------
void Blackboard::DrawExpressionEditor()
{
#define IM_ARRAYSIZE(_ARR)      ((int)(sizeof(_ARR)/sizeof(*_ARR)))

  static bool opened = true;
  ImGui::Begin("Expression Window");

  auto fnGetParam = [](void* data, int idx, const char** out_text)
  {
    Blackboard* self = (Blackboard*)data;
    const string& name = self->_expressionNames[idx];
    const string& expr = self->_expressions[name].repr;
    static char buf[256];
    _snprintf(buf, 255, "%s (%s)", name.c_str(), expr.c_str());
    *out_text = buf;
    return true;
  };

  static float endTime = 30.f;
  static int numSteps = 1000;
  static float scaleMin = -1.0f;
  static float scaleMax = +1.0f;
  ImGui::Combo("Expression", &_curExpr, fnGetParam, this, (int)_expressionNames.size());
  ImGui::InputFloat("End time", &endTime);
  ImGui::InputInt("Num steps", &numSteps);
  ImGui::InputFloat("Scale min", &scaleMin);
  ImGui::InputFloat("Scale max", &scaleMax);
  numSteps = max(1, numSteps);

  Expression expr = _expressions[_expressionNames[_curExpr]];

  eval::Environment env;
  eval::Evaluator e;

  float t = 0;
  float tInc = endTime / numSteps;
  float* res = (float*)_alloca(numSteps * sizeof(float));
  for (int i = 0; i < numSteps; ++i)
  {
    env.constants["t"] = t;
    t += tInc;
    res[i] = e.Evaluate(expr.tokens, &env);
  }

  ImGuiWindow* window = ImGui::GetCurrentWindow();

  ImVec2 size = ImGui::GetContentRegionAvail();
  const float* vals[] = { res };
  ImGui::PlotLinesMulti("Func", vals, numSteps, 0, 1, NULL, scaleMin, scaleMax, size);
  ImGui::End();
}
#endif