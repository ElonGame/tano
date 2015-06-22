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
  template<typename T>
  struct NoOpMixin
  {
    void destroy(T) {}
  };

  //------------------------------------------------------------------------------
  template <typename T, int Capacity, template<typename> class DestroyMixin>
  class AppendBuffer : DestroyMixin<T>
  {
  public:

    //------------------------------------------------------------------------------
    ~AppendBuffer()
    {
      for (int i = 0; i < _size; ++i)
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
      assert(_size < Capacity);
      _elems[_size] = res;
      return _size++;
    }

  private:

    //------------------------------------------------------------------------------
    int _size = 0;
    T _elems[Capacity];
  };

  //------------------------------------------------------------------------------
  template <typename T, int Capacity>
  class SimpleAppendBuffer
  {
  public:
    int Size() const
    { 
      return _size;
    }

    bool Empty() const
    {
      return _size == 0;
    }

    void Append(const T& t)
    { 
      assert(_size < Capacity);
      _elems[_size++] = t; 
    }

    void Clear()
    {
      _size = 0;
    }

    const T& operator[](int idx) const
    {
      assert(idx < _size);
      return _elems[idx];
    }

    T& operator[](int idx)
    {
      assert(idx < _size);
      return _elems[idx];
    }

    T* begin()
    {
      return &_elems[0];
    }

    T* end()
    {
      return &_elems[_size];
    }

  private:
    int _size = 0;
    T _elems[Capacity];
  };
}
