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

  WSAStartup(MAKEWORD(2, 2), &_wsaData);
  Connect("127.0.0.1", "1337");

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
template <typename T>
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

  if (connect(_sockfd, _addrinfo->ai_addr, (int)_addrinfo->ai_addrlen) < 0)
  {
    LOG_INFO(to_string("connect err: %d", errno).c_str());
    _connected = false;
    return false;
  }

  _connected = true;

  SetBlockingIo(false);

  // if (_cbConnected)
  //  _cbConnected();

  _readState = ReadState::ReadHeader;

  return true;
}

template <typename T>
int CopyFromBuffer(const char* buf, T* out)
{
  memcpy(out, buf, sizeof(T));
  return sizeof(T);
}

//------------------------------------------------------------------------------
void Blackboard::ProcessAnimationBuffer(int bufSize, const char* buf)
{
  int numKeysframes;
  buf += CopyFromBuffer(buf, &numKeysframes);

  for (int keyframeIdx = 0; keyframeIdx < numKeysframes; ++keyframeIdx)
  {
    int namelen;
    buf += CopyFromBuffer(buf, &namelen);
    string name;
    name.resize(namelen);
    memcpy((void*)name.data(), buf, namelen);
    buf += namelen;

    int dims;
    buf += CopyFromBuffer(buf, &dims);

    V3 initialValue, lastValue;
    buf += CopyFromBuffer(buf, &initialValue);
    buf += CopyFromBuffer(buf, &lastValue);

    float firstTimestamp, lastTimestamp;
    buf += CopyFromBuffer(buf, &firstTimestamp);
    buf += CopyFromBuffer(buf, &lastTimestamp);

    float sampleStep;
    buf += CopyFromBuffer(buf, &sampleStep);

    int numValues;
    buf += CopyFromBuffer(buf, &numValues);
    vector<float> values;
    values.resize(numValues);
    for (int i = 0; i < numValues; ++i)
    {
      buf += CopyFromBuffer(buf, &values[i]);
    }
    int a = 10;
  }

}

//------------------------------------------------------------------------------
void Blackboard::Process()
{
  if (!_connected)
    return;

  // Note, once a frame is processed, any remaining data is copied to the start of the buffer
  int res =
      recv(_sockfd, _readBuffer.buf + _readBuffer.writeOfs, Buffer::BUFFER_SIZE - _readBuffer.writeOfs, 0);
  if (res <= 0)
  {
    if (res == 0)
    {
      // 0 indicates orderly shutdown, -1 indicates an error
      Disconnect();
    }
    return;
  }

  _readBuffer.writeOfs += res;

  while (true)
  {
    switch (_readState)
    {
      case Blackboard::ReadState::ReadHeader:
      {
        if (_readBuffer.writeOfs >= sizeof(Header))
        {
          memcpy((void*)&_header, _readBuffer.buf, sizeof(Header));
          _readState = Blackboard::ReadState::ReadPayload;
        }
        else
        {
          goto DONE;
        }
        break;
      }

      case Blackboard::ReadState::ReadPayload:
      {
        if (_readBuffer.writeOfs >= _header.payloadSize)
        {
          // got a full frame!

          // copy the rest of the buffer to the second buffer, reset pointers, 
          // and ping pong
          memcpy(_readBuffer.dbl, _readBuffer.buf + _header.payloadSize,
              _readBuffer.writeOfs - _header.payloadSize);

          char* buf = _readBuffer.buf;
          int bufSize = *(int*)buf;
          ProcessAnimationBuffer(bufSize, buf + 4);

          _readBuffer.readOfs = 0;
          _readBuffer.writeOfs = _header.payloadSize;
          _readState = Blackboard::ReadState::ReadHeader;
          swap(_readBuffer.buf, _readBuffer.dbl);
        }
        else
        {
          goto DONE;
        }
      }
    }
  }

DONE:;
}

#endif
