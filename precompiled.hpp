#pragma once
#pragma warning(disable: 4005)
//  C++ exception handler used, but unwind semantics are not enabled. Specify /EHsc
#pragma warning(disable: 4530)

#define WITH_REMOTERY 0
#define WITH_DXGI_DEBUG 1
#define WITH_DEBUG_SHADERS 1
#define WITH_ROCKET 1
#define WITH_ROCKET_PLAYER 1
#define WITH_DEBUG_API 1

#define BORDERLESS_WINDOW 0

#ifndef BORDERLESS_WINDOW 
#define BORDERLESS_WINDOW 0
#endif

#define WITH_BLACKBOARD_TCP 1
#define WITH_BLACKBOARD_SAVE 1

#define WITH_TESTS 0

#if WITH_ROCKET_PLAYER
  #define SYNC_PLAYER
#else
  #ifdef SYNC_PLAYER
    #undef SYNC_PLAYER
  #endif
#endif

#define WITH_IMGUI 1

#define WITH_CONFIG_DLG 0
#define WITH_UNPACKED_RESOUCES 1

#define WITH_MUSIC 1

#define WITH_SCHEDULER_STATS 0

#ifdef _PUBLIC
  #define WITH_UNPACKED_RESOUCES 0
  #define WITH_DEBUG_SHADERS 0
  #define WITH_DXGI_DEBUG 0
  #define WITH_REMOTERY 0
  #define WITH_IMGUI 0
  #define WITH_CONFIG_DLG 1
  #define WITH_ROCKET 0
  #define WITH_ROCKET_PLAYER 0
  #define WITH_BLACKBOARD_TCP 0
  #define WITH_BLACKBOARD_SAVE 0
#elif _DEBUG
  #ifndef WITH_UNPACKED_RESOUCES 
    #define WITH_UNPACKED_RESOUCES 1
  #endif
  #ifndef WITH_DEBUG_SHADERS
    #define WITH_DEBUG_SHADERS 1
  #endif
  #ifndef WITH_DXGI_DEBUG
    #define WITH_DXGI_DEBUG 1
  #endif
  #ifndef WITH_REMOTERY
    #define WITH_REMOTERY 1
  #endif
  #ifndef WITH_IMGUI
    #define WITH_IMGUI 1
  #endif
#else
  #ifndef WITH_UNPACKED_RESOUCES 
    #define WITH_UNPACKED_RESOUCES 0
  #endif
  #ifndef WITH_DEBUG_SHADERS
    #define WITH_DEBUG_SHADERS 0
  #endif
  #ifndef WITH_DXGI_DEBUG
    #define WITH_DXGI_DEBUG 0
  #endif
  #ifndef WITH_REMOTERY
    #define WITH_REMOTERY 0
  #endif
  #ifndef WITH_IMGUI
    #define WITH_IMGUI 0
  #endif
#endif

#if !defined(NOMINMAX)
#define NOMINMAX
#endif

#if WITH_IMGUI
#include "imgui/imgui.h"
#endif

#if WITH_BLACKBOARD_TCP
#include <winsock2.h>
#include <ws2tcpip.h>
#endif

#include <sys/types.h>
#include <errno.h>
#include <fcntl.h>

#include <stdint.h>
#include <assert.h>
#include <time.h>
#include <direct.h>
#include <sys/stat.h>
#include <io.h>

#include <dxgi.h>
#include <dxgidebug.h>
#include <d3d11.h>
#include <D3DX11tex.h>
#include <DirectXMath.h>

#if WITH_ROCKET
#include "sync/device.h"
#include "sync/sync.h"
#endif

#include <atlbase.h>
#include <windows.h>
#include <windowsx.h>

#include <vector>
#include <set>
#include <unordered_set>
#include <map>
#include <unordered_map>
#include <string>
#include <queue>
#include <functional>
#include <memory>
#include <thread>

#if WITH_MUSIC
#include <bass.h>
#endif

#if WITH_REMOTERY
#include "Remotery/lib/Remotery.h"
#else
#define rmt_ScopedCPUSample(x) ;
#endif

#include "bristol/string_utils.hpp"
#include "bristol/flags.hpp"
#include "bristol/utils.hpp"
#include "bristol/error.hpp"
#include "bristol/rolling_average.hpp"
#include "bristol/file_utils.hpp"
#include "bristol/time_utils.hpp"
#include "bristol/file_watcher.hpp"
#include "bristol/dx/graphics_utils.hpp"
#include "bristol/dx/vertex_types.hpp"

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef int64_t s64;

namespace tano
{
  using std::forward;
  using std::vector;
  using std::string;
  using std::wstring;
  using std::set;
  using std::unordered_set;
  using std::map;
  using std::unordered_map;
  using std::deque;
  using std::swap;
  using std::min;
  using std::max;

  using std::pair;
  using std::make_pair;
  using std::sort;
  using std::copy;

  using std::function;
  using std::bind;

  using std::unique_ptr;
  using std::make_unique;
  using std::thread;

  using std::conditional;
  using std::is_void;

  template<typename T>
  void hash_combine(size_t& seed, const T& key)
  {
    std::hash<T> hasher;
    seed ^= hasher(key) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  }

  template<typename T1, typename T2>
  struct std::hash<std::pair<T1, T2>>
  {
    size_t operator()(const pair<T1, T2>& p) const
    {
      size_t seed = 0;
      hash_combine(seed, p.first);
      hash_combine(seed, p.second);
      return seed;
    }
  };


  using DirectX::XMVectorZero;
  using DirectX::XMVECTOR;
  using DirectX::FXMVECTOR;
  using DirectX::XMFLOAT3;
  using DirectX::BoundingSphere;
  using DirectX::XMConvertToRadians;
  using DirectX::SimpleMath::Color;
  using DirectX::SimpleMath::Vector2;
  using DirectX::SimpleMath::Vector3;
  using DirectX::SimpleMath::Vector4;
  using DirectX::SimpleMath::Matrix;
  using DirectX::SimpleMath::Quaternion;
  using DirectX::SimpleMath::Plane;

  using DirectX::XM_PI;

  using namespace std::tr1::placeholders;

  using bristol::Flags;
  using bristol::FileWatcher;

  using bristol::TimeStamp;
  using bristol::TimeDuration;
  using bristol::RollingAverage;
}

#pragma comment(lib, "DXGI.lib")
#pragma comment(lib, "DXGUID.lib")
#pragma comment(lib, "D3D11.lib")
#pragma comment(lib, "D3DX11.lib")
#pragma comment(lib, "psapi.lib")

#ifdef WITH_ROCKET
#pragma comment(lib, "Ws2_32.lib")
#endif

#define STBI_ONLY_PNG
#include "stb/stb_image.h"
