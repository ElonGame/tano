#pragma once

#include "object_handle.hpp"

namespace tano
{
  template <typename T, int Capacity>
  class AppendBuffer
  {
    typedef function<void(T)> Deleter;

  public:

    //------------------------------------------------------------------------------
    AppendBuffer(const Deleter& deleter)
      : _deleter(deleter)
    {
    }

    //------------------------------------------------------------------------------
    ~AppendBuffer()
    {
      for (int i = 0; i < _used; ++i)
      {
        _deleter(_elems[i]);
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
    Deleter _deleter;
    int _used = 0;
    T _elems[Capacity];
  };
}