#pragma once

namespace tano
{
  struct InputBuffer;

  class Blackboard
  {
  public:
#define ADD_VAR(type, name, storage)                          \
  void Add ## name ## Var(const string& name, type value)     \
  {                                                           \
    storage[name] = value;                                    \
  }                                                           \
  type Get ## name ## Var(const string& name)                 \
  {                                                           \
    auto it = storage.find(name);                             \
    assert(it != storage.end());                              \
    return it->second;                                        \
    }

    ADD_VAR(float, Float, _floatVars);
    ADD_VAR(int, Int, _intVars);
    ADD_VAR(Vector2, Vec2, _vec2Vars);
    ADD_VAR(Vector3, Vec3, _vec3Vars);
    ADD_VAR(Vector4, Vec4, _vec4Vars);

    static bool Create(const char* filename);
    static void Destory();

    static Blackboard& Instance();

  private:
    bool Init(const char* filename);
    bool ParseBlackboard(InputBuffer& buf);
    void Reset();

    static Blackboard* _instance;

    unordered_map<string, float> _floatVars;
    unordered_map<string, int> _intVars;
    unordered_map<string, Vector2> _vec2Vars;
    unordered_map<string, Vector3> _vec3Vars;
    unordered_map<string, Vector4> _vec4Vars;
  };

#define BLACKBOARD Blackboard::Instance()

}