#pragma once

namespace bristol
{
  template <typename V, typename S = V>
  struct RollingAverage
  {
    RollingAverage(size_t numSamples)
    : _samples(numSamples)
    , _samplesUsed(0)
    , _nextSample(0)
    , _sum(0)
    {
    }

    void AddSample(V v)
    {
      if (_samplesUsed < _samples.size())
      {
        _samples[_samplesUsed++] = v;
        _sum += v;
      }
      else
      {
        _sum -= _samples[_nextSample];
        _sum += v;
        _samples[_nextSample] = v;
        _nextSample = (_nextSample + 1) % _samples.size();
      }
    }

    V GetAverage() const
    {
      return _samplesUsed == 0 ? 0 : _sum / _samplesUsed;
    }

    void GetMinMax(V* minValue, V* maxValue) const
    {
      V localMaxValue = std::numeric_limits<V>::min();
      V localMinValue = std::numeric_limits<V>::max();
      for (size_t i = 0; i < _samplesUsed; ++i)
      {
        localMaxValue = max(localMaxValue, _samples[i]);
        localMinValue = min(localMinValue, _samples[i]);
      }

      if (minValue)
        *minValue = localMinValue;

      if (maxValue)
        *maxValue = localMaxValue;
    }

    void CopySamples(V* out, size_t* numSamples)
    {
      *numSamples = _samplesUsed;
      if (_samplesUsed < _samples.size())
      {
        std::copy(_samples.begin(), _samples.begin() + _samplesUsed, out);
      }
      else
      {
        // copy from _nextSample to end
        std::copy(_samples.begin() + _nextSample, _samples.end(), out);

        // copy from begining to _nextSample - 1
        std::copy(_samples.begin(), _samples.begin() + _nextSample, out + (_samplesUsed - _nextSample));
      }
    }

    size_t IsPrimed() const { return _samplesUsed == _samples.size(); }

    std::vector<V> _samples;
    size_t _samplesUsed;
    size_t _nextSample;
    S _sum;
  };
}

