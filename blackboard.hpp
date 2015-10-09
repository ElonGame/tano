#pragma once
#include "tano_math.hpp"

namespace parser
{
  struct InputBuffer;
}

namespace tano
{
  class Blackboard
  {
  public:
    void SetNamespace(const string& ns);
    void ClearNamespace();

    void AddIntVar(const string& name, int value);
    void AddFloatVar(const string& name, float value);
    void AddVec2Var(const string& name, const vec2& value);
    void AddVec3Var(const string& name, const vec3& value);
    void AddVec4Var(const string& name, const vec4& value);

    int GetIntVar(const string& name);
    float GetFloatVar(const string& name);
    vec2 GetVec2Var(const string& name);
    vec3 GetVec3Var(const string& name);
    vec4 GetVec4Var(const string& name);

    float GetFloatVar(const string& name, float t);
    vec2 GetVec2Var(const string& name, float t);
    vec3 GetVec3Var(const string& name, float t);

    template <typename T>
    struct Keyframes;
    template <typename T>
    T GetVar(const string& name, float t, unordered_map<string, Keyframes<T>*>& vars);

    static bool Create(const char* filename, const char* datafile);
    static void Destory();

    static Blackboard& Instance();

#if WITH_BLACKBOARD_TCP
    bool Connect(const char* host, const char* service);
    void Disconnect();
    void Process();
    void SetBlockingIo(bool blocking);
#endif

    bool IsDirtyTrigger(void* id);

    void DrawExpressionEditor();

  private:

    ~Blackboard();

    bool Init(const char* filename, const char* datafile);
    bool ParseBlackboard(parser::InputBuffer& buf, deque<string>& namespaceStack);
    void Reset();

#if WITH_BLACKBOARD_SAVE && WITH_BLACKBOARD_TCP && WITH_UNPACKED_RESOUCES
    void SaveData();
#endif
    void LoadData();

    void ProcessAnimationBuffer(const char* buf, int bufSize);

    static Blackboard* _instance;

    string _curNamespace;

    template <typename T>
    struct Keyframes
    {
      Keyframes() {}
      Keyframes(const T& v) : firstValue(v), lastValue(v) {}
      T firstValue;
      T lastValue;
      float firstTime = 0;
      float lastTime = 0;
      float sampleStep = 1;
      vector<T> values;
    };

    template <typename T>
    int LoadKeyframes(const char* buf, const string& name, unordered_map<string, Keyframes<T>*>* res);

    template <typename T>
    T GetValueAtTime(float t, const Keyframes<T>* keyframes);

    unordered_map<string, Keyframes<int>*> _intVars;
    unordered_map<string, Keyframes<float>*> _floatVars;
    unordered_map<string, Keyframes<vec2>*> _vec2Vars;
    unordered_map<string, Keyframes<vec3>*> _vec3Vars;
    unordered_map<string, Keyframes<vec4>*> _vec4Vars;

    string _filename;
    string _datafile;

    unordered_set<void*> _triggeredIds;

#if WITH_BLACKBOARD_SAVE
    unordered_map<string, vector<char>> _rawKeyframes;
#endif

#if WITH_BLACKBOARD_TCP

    struct Header
    {
      // total frame size, including header
      int payloadSize = 0;
    };

    enum class ReadState
    {
      Connecting,
      ReadHeader,
      ReadPayload,
    };

    struct Buffer
    {
      void Reset()
      {
        writeOfs = 0;
        readOfs = 0;
      }
      bool BufferFull() { return readOfs == writeOfs; }
      int readOfs = 0;
      int writeOfs = 0;
      enum
      {
        BUFFER_SIZE = 64 * 1024
      };
      char buf1[BUFFER_SIZE];
      char buf2[BUFFER_SIZE];
      char* buf = &buf1[0];
      char* dbl = &buf1[1];
    };

    Header _header;
    ReadState _readState = ReadState::ReadHeader;
    SOCKET _sockfd = 0;
    string _host;
    string _serviceName;
    bool _connected = false;
    struct addrinfo* _addrinfo = nullptr;
    Buffer _readBuffer;
    WSADATA _wsaData;
#endif
  };

#define BLACKBOARD Blackboard::Instance()
}