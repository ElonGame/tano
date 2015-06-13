#pragma once

#include "object_handle.hpp"

namespace tano
{
  //------------------------------------------------------------------------------
  template<typename T>
  struct DeleteMixin
  {
    void destroy(T t)
    {
      delete t;
    }
  };

  //------------------------------------------------------------------------------
  template<typename T>
  struct ReleaseMixin
  {
    void destroy(T t)
    {
      if (t)
        t->Release();
    }
  };

  //------------------------------------------------------------------------------
  template <typename T, int Capacity, template<typename> class DestroyMixin>
  class AppendBuffer : DestroyMixin<T>
  {
  public:

    //------------------------------------------------------------------------------
    ~AppendBuffer()
    {
      for (int i = 0; i < _used; ++i)
      {
        destroy(_elems[i]);
      }
    }

    //------------------------------------------------------------------------------
    T Get(ObjectHandle h)
    {
      if (!h.IsValid())
        return T();

      return _elems[h.id()];
    }

    //------------------------------------------------------------------------------
    void Update(ObjectHandle h, T res)
    {
      _elems[h.id()] = res;
    }

    //------------------------------------------------------------------------------
    u32 Append(T res)
    {
      assert(_used < Capacity);
      _elems[_used] = res;
      return _used++;
    }

    //------------------------------------------------------------------------------
    T Find(const function<bool (const T&)>& fn, int* idx)
    {
      for (int i = 0; i < _used; ++i)
      {
        if (fn(_elems[i]))
        {
          if (idx)
            *idx = i;
          return _elems[i];
        }
      }

      return T();
    }

  private:

    //------------------------------------------------------------------------------
    int _used = 0;
    T _elems[Capacity];
  };
}
