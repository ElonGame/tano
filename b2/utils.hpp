#pragma once

#include <functional>
#include <unordered_map>
#include <string>
#include <stdlib.h>

#ifdef _WIN32
#include <windows.h>
#endif

namespace bristol
{
  #ifdef _WIN32
  class CriticalSection
  {
  public:
    CriticalSection();
    ~CriticalSection();
    void enter();
    void leave();
  private:
    CRITICAL_SECTION _cs;
  };

  class ScopedCs
  {
  public:
    ScopedCs(CriticalSection &cs);
    ~ScopedCs();
  private:
    CriticalSection &_cs;
  };

  struct ScopedCriticalSection
  {
    ScopedCriticalSection(CRITICAL_SECTION* cs) : cs(cs) { EnterCriticalSection(cs); }
    ~ScopedCriticalSection() { LeaveCriticalSection(cs); }
    CRITICAL_SECTION* cs;
  };

  class ScopedHandle
  {
  public:
    ScopedHandle(HANDLE h) : _handle(h) {}
    ~ScopedHandle() {
      if (_handle != INVALID_HANDLE_VALUE)
        CloseHandle(_handle);
    }
    operator bool() { return _handle != INVALID_HANDLE_VALUE; }
    operator HANDLE() { return _handle; }
    HANDLE handle() const { return _handle; }
  private:
    HANDLE _handle;
  };
#endif
  struct ScopedObj
  {
    typedef std::function<void()> Fn;
    ScopedObj(const Fn& fn) : fn(fn) {}
    ~ScopedObj() { fn(); }
    Fn fn;
  };

  // Macro for creating "local" names
#define GEN_NAME2(prefix, line) prefix##line
#define GEN_NAME(prefix, line) GEN_NAME2(prefix, line)
#define MAKE_SCOPED(type) type GEN_NAME(ANON, __LINE__)

#define SCOPED_CONC_CS(x) MAKE_SCOPED(Concurrency::critical_section::scoped_lock)(x);
#define SCOPED_OBJ(x) MAKE_SCOPED(ScopedObj)(x);
#define DEFER(x) MAKE_SCOPED(ScopedObj)(x);
#define SCOPED_CS(cs) ScopedCs lock(cs);

  // A macro to disallow the copy constructor and operator= functions
  // This should be used in the private: declarations for a class
#define DISALLOW_COPY_AND_ASSIGN(TypeName)  \
  TypeName(const TypeName&) = delete;       \
  void operator=(const TypeName&) = delete; \


  template<class T> 
  void SeqDelete(T *seq)
  {
    for (auto it = begin(*seq); it != end(*seq); ++it)
      delete *it;
    seq->clear();
  }

  template<class T> 
  void AssocDelete(T* t)
  {
    for (typename T::iterator it = t->begin(); it != t->end(); ++it)
      delete it->second;
    t->clear();
  }

  template <typename T>
  void Append(const std::vector<T>& src, std::vector<T>* dst)
  {
    size_t oldSize = dst->size();
    dst->resize(dst->size() + src.size());
    std::copy(src.begin(), src.end(), dst->begin() + oldSize);
  }

  template<class T>
  void ReleaseObj(T t)
  {
    if (t)
      t->Release();
  }

  template<class T>
  void DeleteObj(T t)
  {
    delete t;
  }

  template <class T>
  T exch_null(T &t)
  {
    T tmp = t;
    t = nullptr;
    return tmp;
  }
  template <class T>
  T exch(T &t, const T &v)
  {
    T tmp = t;
    t = v;
    return tmp;
  }

  template<typename T>
  T min3(const T &a, const T &b, const T &c) {
    return std::min(a, std::min(b, c));
  }

  template<typename T>
  T max3(const T &a, const T &b, const T &c) {
    return std::max(a, std::max(b, c));
  }

  template<typename T>
  T min4(const T &a, const T &b, const T &c, const T &d) {
    return std::min(a, min3(b, c, d));
  }

  template<typename T>
  T max4(const T &a, const T &b, const T &c, const T &d) {
    return std::max(a, max3(b, c, d));
  }


#define SAFE_RELEASE(x) if( (x) != 0 ) { (x)->Release(); (x) = 0; }
#define SAFE_FREE(x) if( (x) != 0 ) { free((void*)(x)); (x) = 0; }
#define SAFE_DELETE(x) if( (x) != 0 ) { delete (x); (x) = 0; }
#define SAFE_ADELETE(x) if( (x) != 0 ) { delete [] (x); (x) = 0; }

#define ELEMS_IN_ARRAY(x) sizeof(x) / sizeof((x)[0])

  template<typename Container, typename Key>
  bool contains(const Container &c, const Key key)
  {
    typename Container::const_iterator it = c.find(key);
    return it != c.end();
  }

  template<typename Key, typename Value>
  Value lookup_default(Key str, const std::unordered_map<Key, Value> &candidates, Value default_value)
  {
    auto it = candidates.find(str);
    return it != candidates.end() ? it->second : default_value;
  }

  template <typename T, typename U>
  T lerp(T a, T b, U v)
  {
    return (T)((1-v) * a + v * b);
  }

  template <typename T>
  T randf(T a, T b)
  {
    float t = (float)rand() / RAND_MAX;
    return lerp(a, b, t);
  }

  template <typename T>
  T Clamp(T minValue, T maxValue, T value)
  {
    return value < minValue ? minValue : value > maxValue ? maxValue : value;
  }

  //
  // SmoothStep - Like the step function but provides a smooth transition from a to b
  //              using the cubic function 3x^2 - 2x^3.  From The Renderman Companion.
  //

  inline float SmoothStepT(float a, float b, float t)
  {
    if (t <= a)
      return 0;
    else if (t >= b)
      return 1;
    t = (t - a) / (b - a);    // normalize t to 0..1
    return t*t*(3 - 2 * t);
  }

  inline float SmoothStep(float a, float b, float t)
  {
    return lerp(a, b, SmoothStepT(a, b, t));
  }

  float GaussianRand(float mean, float variance);
  std::string ToString(const char* format, ...);
  void DebugOutput(const char* fmt, ...);

  #if 0
  template <typename T>
  bool LoadProto(const char* filename, T* out)
  {
    FILE* f = fopen(filename, "rb");
    if (!f)
      return false;

    fseek(f, 0, 2);
    size_t s = ftell(f);
    fseek(f, 0, 0);
    std::string str;
    str.resize(s);
    fread((char*)str.c_str(), 1, s, f);
    fclose(f);

    return google::protobuf::TextFormat::ParseFromString(str, out);
  }
  #endif

  struct ScopeGuard
  {
    typedef std::function<void()> Func;
    ScopeGuard(const Func& fn) : fn(fn) {}
    ~ScopeGuard()
    {
      if (!commit)
        fn();
    }

    void Commit() const { commit = true; }

    mutable bool commit = false;
    Func fn;
  };

  template<typename T>
  bool Inside(T value, T start, T end)
  {
    return value >= start && value < end;
  }

  template <typename T>
  void Range(T* dst, T start, T end, T step = (T)1)
  {
    int idx = 0;
    for (T i = start; i < end; i += step)
    {
      dst[idx++] = i;
    }
  }
}

