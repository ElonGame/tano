#pragma once
#include <stdint.h>

// pretty much stolen from http://molecularmusings.wordpress.com/2011/08/23/flags-on-steroids/

namespace bristol
{
  // input is a struct of the form
  // struct Mode { enum Enum { type1, type2 }; struct Bits { uint32_t type1 : 1; uint32_t type2 : 2 } }
  template <typename F>
  struct Flags
  {

    typedef typename F::Enum Enum;
    typedef typename F::Bits Bits;

    Flags() : _value(0) {}
    Flags(Enum f) : _value(f) {}
    Flags(int f) : _value(f) {}

    bool IsSet(Enum f) const
    {
      return !!(_value & f);
    }

    void Set(Enum f)
    {
      _value |= f;
    }

    bool CheckAndSet(Enum f)
    {
      bool tmp = IsSet(f);
      Set(f);
      return tmp;
    }

    void Toggle(Enum f)
    {
      _value ^= f;
    }

    void Clear(Enum f)
    {
      _value &= ~f;
    }

    bool CheckAndClear(Enum f)
    {
      bool tmp = IsSet(f);
      Clear(f);
      return tmp;
    }

    void Reset()
    {
      _value = 0;
    }

    bool operator==(const Flags& rhs)
    {
      return _value == rhs._value;
    }

    Flags& operator|=(Enum f)
    {
      _value |= f;
      return *this;
    }

    Flags operator|(Enum f)
    {
      return Flags(Enum(_value | f));
    }

    Flags operator|(Flags rhs)
    {
      return Flags(Enum(_value | rhs._value));
    }

    union
    {
      uint32_t _value;
      Bits _bits;
    };
  };
}
