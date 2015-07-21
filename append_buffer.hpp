#pragma once

#include "object_handle.hpp"

namespace tano
{
  //------------------------------------------------------------------------------
  template <typename T>
  struct DeleteMixin
  {
    void destroy(T t) { delete t; }
  };

  //------------------------------------------------------------------------------
  template <typename T>
  struct ReleaseMixin
  {
    void destroy(T t)
    {
      if (t)
        t->Release();
    }
  };

  //------------------------------------------------------------------------------
  template <typename T>
  struct NoOpMixin
  {
    void destroy(T) {}
  };

  //------------------------------------------------------------------------------
  template <typename T, int Capacity, template <typename> class DestroyMixin>
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
    void Update(ObjectHandle h, T res) { _elems[h.id()] = res; }

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
  template <typename T>
  class SimpleAppendBufferExternal
  {
  public:
    SimpleAppendBufferExternal(void* start, void* end)
    {
      _mem = (T*)start;
      _capacity = (int)((T*)end - _mem);
    }

    int Size() const { return _size; }

    int Capacity() const { return _capacity; }

    bool Empty() const { return _size == 0; }

    T& Append(const T& t)
    {
      assert(_size < _capacity);
      _mem[_size] = t;
      return _mem[_size++];
    }

    void Clear() { _size = 0; }

    void Resize(int s)
    {
      assert(s <= _capacity);
      _size = s;
    }

    T* Data() { return &_mem[0]; }

    int DataSize() { return _size * sizeof(T); }

    const T& operator[](int idx) const
    {
      assert(idx < _size);
      return _mem[idx];
    }

    T& operator[](int idx)
    {
      assert(idx < _size);
      return _mem[idx];
    }

    T* begin() { return &_mem[0]; }

    T* end() { return &_mem[_size]; }

  protected:
    int _capacity = 0;
    int _size = 0;
    T* _mem = 0;
  };

  //------------------------------------------------------------------------------
  template <typename T, int Cap>
  class SimpleAppendBuffer : public SimpleAppendBufferExternal<T>
  {
  public:
    SimpleAppendBuffer() : SimpleAppendBufferExternal(&_elems[0], &_elems[Cap]) {}

  private:
    T _elems[Cap];
  };
}
