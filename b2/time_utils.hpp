#pragma once
#include <stdint.h>

namespace bristol
{
  class TimeDuration;

  class TimeStamp
  {
  public:
    TimeStamp();
    static TimeStamp Now();
    bool IsValid() const;

    int64_t TotalSeconds() const;
    int64_t TotalMilliseconds() const;
    int64_t TotalMicroseconds() const;
    int64_t TotalNanoseconds() const;


    TimeStamp& operator+=(const TimeDuration& rhs);
    TimeStamp& operator-=(const TimeDuration& rhs);

    friend bool operator>(const TimeStamp& lhs, const TimeStamp& rhs);
    friend TimeDuration operator-(const TimeStamp& lhs, const TimeStamp& rhs);
    friend TimeStamp operator+(const TimeStamp& lhs, const TimeDuration& rhs);
    friend TimeStamp operator-(const TimeStamp& lhs, const TimeDuration& rhs);

  private:
    TimeStamp(const int64_t& t);

    // The timestamp is in platform dependent ticks
    int64_t _timestamp = 0;
  };

  class TimeDuration
  {
    friend class TimeStamp;
  public:
    TimeDuration();

    static TimeDuration Seconds(int64_t x);
    static TimeDuration Milliseconds(int64_t x);
    static TimeDuration Microseconds(int64_t x);
    static TimeDuration Nanoseconds(int64_t x);

    int64_t TotalSeconds() const;
    int64_t TotalMicroseconds() const;
    int64_t TotalMilliseconds() const;
    int64_t TotalNanoseconds() const;

    float TotalSecondsAsFloat() const;

    void Clamp(int64_t value = 0);
    int64_t GetTimestamp() const { return _timestamp; }

    friend bool operator<(const TimeDuration& lhs, const TimeDuration& rhs);
    friend bool operator<=(const TimeDuration& lhs, const TimeDuration& rhs);
    friend bool operator==(const TimeDuration& lhs, const TimeDuration& rhs);
    friend bool operator>(const TimeDuration& lhs, const TimeDuration& rhs);
    friend bool operator>=(const TimeDuration& lhs, const TimeDuration& rhs);

    TimeDuration& operator+=(const TimeDuration& t);
    TimeDuration& operator-=(const TimeDuration& t);
    friend TimeDuration operator-(const TimeStamp& lhs, const TimeStamp& rhs);
    friend TimeDuration operator-(const TimeDuration& lhs, const TimeDuration& rhs);
    friend TimeDuration operator+(const TimeDuration& lhs, const TimeDuration& rhs);
    friend TimeStamp operator+(const TimeStamp& lhs, const TimeDuration& rhs);
    friend TimeStamp operator-(const TimeStamp& lhs, const TimeDuration& rhs);
  private:
    explicit TimeDuration(const int64_t& t);
    int64_t _timestamp = 0;
  };

}
