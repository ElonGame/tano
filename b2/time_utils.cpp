#include "time_utils.hpp"
#include <algorithm>

#if _WIN32
#include <windows.h>
#else
#include <mach/mach.h>
#include <mach/mach_time.h>
#endif


using namespace bristol;
using namespace std;

//------------------------------------------------------------------------------
namespace
{
#ifdef _WIN32
  LARGE_INTEGER g_frequency;
  bool Init()
  {
    QueryPerformanceFrequency(&g_frequency);
    return true;
  }
#else
  mach_timebase_info_data_t g_timebaseInfo;

  bool Init()
  {
    mach_timebase_info(&g_timebaseInfo);
    return true;
  }
#endif

  bool init = Init();
}

//------------------------------------------------------------------------------
//
// Timestamp
//
//------------------------------------------------------------------------------
TimeStamp::TimeStamp()
  : _timestamp(0)
{
}

//------------------------------------------------------------------------------
TimeStamp::TimeStamp(const int64_t& t)
  : _timestamp(t)
{
}

//------------------------------------------------------------------------------
bool TimeStamp::IsValid() const 
{ 
  return _timestamp > 0; 
}

//------------------------------------------------------------------------------
TimeStamp TimeStamp::Now()
{
  TimeStamp t;
  #ifdef _WIN32
  LARGE_INTEGER tmp;
  QueryPerformanceCounter(&tmp);
  t._timestamp = tmp.QuadPart;
  #else
  t._timestamp = mach_absolute_time();
  #endif
  return t;
}

//------------------------------------------------------------------------------
TimeStamp& TimeStamp::operator+=(const TimeDuration& rhs)
{
  _timestamp += rhs._timestamp;
  return *this;
}

//------------------------------------------------------------------------------
TimeStamp& TimeStamp::operator-=(const TimeDuration& rhs)
{
  _timestamp = max(_timestamp - rhs._timestamp, 0ll);
  return *this;
}

//------------------------------------------------------------------------------
TimeStamp bristol::operator+(const TimeStamp& lhs, const TimeDuration& rhs)
{
  return TimeStamp(lhs._timestamp + rhs._timestamp);
}

//------------------------------------------------------------------------------
TimeStamp bristol::operator-(const TimeStamp& lhs, const TimeDuration& rhs)
{
  return TimeStamp(max(lhs._timestamp - rhs._timestamp, 0ll));
}

//------------------------------------------------------------------------------
TimeDuration bristol::operator-(const TimeStamp& lhs, const TimeStamp& rhs)
{
  return TimeDuration(lhs._timestamp - rhs._timestamp);
}

//------------------------------------------------------------------------------
bool bristol::operator>(const TimeStamp& lhs, const TimeStamp& rhs)
{
  return lhs._timestamp > rhs._timestamp;
}

//------------------------------------------------------------------------------
int64_t TimeStamp::TotalSeconds() const
{
#ifdef _WIN32
  return _timestamp / g_frequency.QuadPart;
#else
  return _timestamp * g_timebaseInfo.numer / g_timebaseInfo.denom / 1e9;
#endif
}

//------------------------------------------------------------------------------
int64_t TimeStamp::TotalMilliseconds() const
{
#ifdef _WIN32
  return (int64_t)(1e3 * _timestamp / g_frequency.QuadPart);
#else
  return _timestamp * g_timebaseInfo.numer / g_timebaseInfo.denom / 1e3;
#endif
}

//------------------------------------------------------------------------------
int64_t TimeStamp::TotalMicroseconds() const
{
#ifdef _WIN32
  return (int64_t)(1e6 * _timestamp / g_frequency.QuadPart);
#else
  return _timestamp * g_timebaseInfo.numer / g_timebaseInfo.denom / 1e6;
#endif
}

//------------------------------------------------------------------------------
int64_t TimeStamp::TotalNanoseconds() const
{
#ifdef _WIN32
  return (int64_t)(1e9 * _timestamp / g_frequency.QuadPart);
#else
  return _timestamp * g_timebaseInfo.numer / g_timebaseInfo.denom;
#endif
}

//------------------------------------------------------------------------------
//
// TimeDuration
//
//------------------------------------------------------------------------------
TimeDuration::TimeDuration()
  : _timestamp(0)
{
}

//------------------------------------------------------------------------------
TimeDuration::TimeDuration(const int64_t& t)
{ 
  _timestamp = t;
}

//------------------------------------------------------------------------------
void TimeDuration::Clamp(int64_t value)
{
  _timestamp = max(value, _timestamp);
}

//------------------------------------------------------------------------------
TimeDuration TimeDuration::Seconds(int64_t x)
{
#ifdef _WIN32
  return TimeDuration(x * g_frequency.QuadPart);
#else
  return TimeDuration(x * 1e9 * g_timebaseInfo.denom / g_timebaseInfo.numer);
#endif
}

//------------------------------------------------------------------------------
TimeDuration TimeDuration::Milliseconds(int64_t x)
{
#ifdef _WIN32
  return TimeDuration((int64_t)(x * g_frequency.QuadPart / 1e3));
#else
  return TimeDuration(x * 1e3 * g_timebaseInfo.denom / g_timebaseInfo.numer);
#endif
}

//------------------------------------------------------------------------------
TimeDuration TimeDuration::Microseconds(int64_t x)
{
#ifdef _WIN32
  return TimeDuration((int64_t)(x * g_frequency.QuadPart / 1e6));
#else
  return TimeDuration(x * 1e6 * g_timebaseInfo.denom / g_timebaseInfo.numer);
#endif
}

//------------------------------------------------------------------------------
TimeDuration TimeDuration::Nanoseconds(int64_t x)
{
#ifdef _WIN32
  return TimeDuration((int64_t)(x * g_frequency.QuadPart / 1e9));
#else
  return TimeDuration(x * g_timebaseInfo.denom / g_timebaseInfo.numer);
#endif
}

//------------------------------------------------------------------------------
int64_t TimeDuration::TotalSeconds() const
{
#ifdef _WIN32
  return _timestamp / g_frequency.QuadPart;
#else
  return _timestamp * g_timebaseInfo.numer / g_timebaseInfo.denom / 1e9;
#endif
}

//------------------------------------------------------------------------------
int64_t TimeDuration::TotalMilliseconds() const
{
#ifdef _WIN32
  return (int64_t)(1e3 * _timestamp / g_frequency.QuadPart);
#else
  return _timestamp * g_timebaseInfo.numer / g_timebaseInfo.denom / 1e3;
#endif
}

//------------------------------------------------------------------------------
float TimeDuration::TotalSecondsAsFloat() const
{
  return TotalMicroseconds() / 1e6f;
}

//------------------------------------------------------------------------------
int64_t TimeDuration::TotalMicroseconds() const
{
#ifdef _WIN32
  return (int64_t)(1e6 * _timestamp / g_frequency.QuadPart);
#else
  return _timestamp * g_timebaseInfo.numer / g_timebaseInfo.denom / 1e6;
#endif
}

//------------------------------------------------------------------------------
int64_t TimeDuration::TotalNanoseconds() const
{
#ifdef _WIN32
  return (int64_t)(1e9 * _timestamp / g_frequency.QuadPart);
#else
  return _timestamp * g_timebaseInfo.numer / g_timebaseInfo.denom;
#endif
}

//------------------------------------------------------------------------------
TimeDuration bristol::operator-(const TimeDuration& lhs, const TimeDuration& rhs)
{
  return TimeDuration(lhs._timestamp - rhs._timestamp);
}

//------------------------------------------------------------------------------
TimeDuration bristol::operator+(const TimeDuration& lhs, const TimeDuration& rhs)
{
  return TimeDuration(lhs._timestamp + rhs._timestamp);
}

//------------------------------------------------------------------------------
TimeDuration& TimeDuration::operator+=(const TimeDuration& t)
{
  _timestamp += t._timestamp;
  return *this;
}

//------------------------------------------------------------------------------
TimeDuration& TimeDuration::operator-=(const TimeDuration& t)
{
  _timestamp -= t._timestamp;
  return *this;
}

//------------------------------------------------------------------------------
bool bristol::operator<(const TimeDuration& lhs, const TimeDuration& rhs)
{
  return lhs._timestamp < rhs._timestamp;
}

//------------------------------------------------------------------------------
bool bristol::operator<=(const TimeDuration& lhs, const TimeDuration& rhs)
{
  return lhs._timestamp <= rhs._timestamp;
}

//------------------------------------------------------------------------------
bool bristol::operator==(const TimeDuration& lhs, const TimeDuration& rhs)
{
  return lhs._timestamp == rhs._timestamp;
}


//------------------------------------------------------------------------------
bool bristol::operator>(const TimeDuration& lhs, const TimeDuration& rhs)
{
  return lhs._timestamp > rhs._timestamp;
}

//------------------------------------------------------------------------------
bool bristol::operator>=(const TimeDuration& lhs, const TimeDuration& rhs)
{
  return lhs._timestamp >= rhs._timestamp;
}
