namespace tano
{
  // Like STL-deque, but operates on a fixed slab of memory
  template <typename T>
  struct FixedDequeue
  {
    FixedDequeue(void* memory, int capacity)
      : _capacity(capacity)
    {
      _elems = (T*)memory;
    }

    void PushFront(const T& t)
    {
      assert(_numElems < _capacity);
      _frontIdx = Dec(_frontIdx);
      _elems[_frontIdx] = t;
      _numElems++;
    }

    void PushBack(const T& t)
    {
      assert(_numElems < _capacity);
      _elems[_backIdx] = t;
      _backIdx = Inc(_backIdx);
      _numElems++;
    }

    bool IsEmpty() const
    {
      return _numElems == 0;
    }

    bool IsFull() const
    {
      return _numElems == _capacity;
    }

    T Front()
    {
      return _elems[_frontIdx];
    }

    T Back()
    {
      return _elems[Dec(_backIdx)];
    }

    void PopFront()
    {
      assert(_numElems >= 0);
      _frontIdx = Inc(_frontIdx);
      _numElems--;
    }

    void PopBack()
    {
      assert(_numElems >= 0);
      _backIdx = Dec(_backIdx);
      _numElems--;
    }

    int Dec(int value)
    {
      return value == 0 ? _capacity - 1 : value - 1;
    }

    int Inc(int value)
    {
      return value == _capacity - 1 ? 0 : value + 1;
    }

    T* _elems = nullptr;
    // points to the first element
    int _frontIdx = 0;
    // points one past the last element (stl-style)
    int _backIdx = 0;
    int _capacity = 0;
    int _numElems = 0;
  };
}
