#include "blackboard.hpp"
#include "resource_manager.hpp"
#include "init_sequence.hpp"

using namespace tano;
using namespace bristol;

Blackboard* Blackboard::_instance;

//------------------------------------------------------------------------------
Blackboard::~Blackboard()
{
  Reset();
}

//------------------------------------------------------------------------------
bool Blackboard::Create(const char* filename, const char* datafile)
{
  assert(!_instance);
  _instance = new Blackboard();
  return _instance->Init(filename, datafile);
}

//------------------------------------------------------------------------------
void Blackboard::Destory()
{
#if WITH_BLACKBOARD_SAVE && WITH_BLACKBOARD_TCP && WITH_UNPACKED_RESOUCES
  _instance->SaveData();
#endif
  delete exch_null(_instance);
}

//------------------------------------------------------------------------------
Blackboard& Blackboard::Instance()
{
  assert(_instance);
  return *_instance;
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
            _triggeredIds.clear();
          }
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

  LoadData();

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
template <typename T>
T Blackboard::GetVar(const string& name, float t, unordered_map<string, Keyframes<T>*>& vars)
{
  string fullname = _curNamespace.empty() ? name : _curNamespace + "." + name;

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
      vec2 tmp;
      ParseVec2(buf, &tmp);
      AddVec2Var(fnFullName(id), tmp);

      buf.SkipWhitespace();
      CHECKED_OP(buf.Expect(';'));
    }
    else if (keyword == "vec3")
    {
      CHECKED_OP(ParseIdentifier(buf, &id, false));
      buf.SkipWhitespace();

      CHECKED_OP(buf.Expect('='));

      buf.SkipWhitespace();
      vec3 tmp;
      ParseVec3(buf, &tmp);
      AddVec3Var(fnFullName(id), tmp);

      buf.SkipWhitespace();
      CHECKED_OP(buf.Expect(';'));
    }
    else if (keyword == "vec4")
    {
      CHECKED_OP(ParseIdentifier(buf, &id, false));
      buf.SkipWhitespace();

      CHECKED_OP(buf.Expect('='));

      buf.SkipWhitespace();
      vec4 tmp;
      ParseVec4(buf, &tmp);
      AddVec4Var(fnFullName(id), tmp);

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
  int idx0 = (int)(relT / step);
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
