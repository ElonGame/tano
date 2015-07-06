#pragma once
#include "tano_math.hpp"

namespace tano
{
  struct InputBuffer;

  class Blackboard
  {
  public:

    void SetNamespace(const string& ns);
    void ClearNamespace();

    void AddIntVar(const string& name, int value);
    void AddFloatVar(const string& name, float value);
    void AddVec2Var(const string& name, const V2& value);
    void AddVec3Var(const string& name, const V3& value);
    void AddVec4Var(const string& name, const V4& value);

    int GetIntVar(const string& name);
    float GetFloatVar(const string& name);
    V2 GetVec2Var(const string& name);
    V3 GetVec3Var(const string& name);
    V4 GetVec4Var(const string& name);

    template<typename T>
    T GetVar(const string& name, unordered_map<string, T>& vars);

    static bool Create(const char* filename);
    static void Destory();

    static Blackboard& Instance();

  private:
    bool Init(const char* filename);
    bool ParseBlackboard(InputBuffer& buf, deque<string>& namespaceStack);
    void Reset();

    static Blackboard* _instance;

    string _curNamespace;

    unordered_map<string, int> _intVars;
    unordered_map<string, float> _floatVars;
    unordered_map<string, V2> _vec2Vars;
    unordered_map<string, V3> _vec3Vars;
    unordered_map<string, V4> _vec4Vars;
  };

#define BLACKBOARD Blackboard::Instance()

}