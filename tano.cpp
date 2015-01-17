#include "tano.hpp"
#include "resource_manager.hpp"
#include "demo_engine.hpp"
#include "graphics.hpp"
#include "deferred_context.hpp"
#include "init_sequence.hpp"
#include "generated/app.parse.hpp"
#include "generated/input_buffer.hpp"
#include "effects/particle_tunnel.hpp"

//------------------------------------------------------------------------------
using namespace tano;
using namespace bristol;

static const int WM_APP_CLOSE = WM_APP + 2;

const TCHAR* g_AppWindowClass = _T("TanoClass");
const TCHAR* g_AppWindowTitle = _T("tano - neurotica e.f.s");

App* App::_instance;

//------------------------------------------------------------------------------
#define FMOD_CHECKED(x) { \
  FMOD_RESULT result = x; \
  if (result != FMOD_OK) { \
  LOG_ERROR(ToString("FMOD error! (%d) %s\n", result, FMOD_ErrorString(result)).c_str()); \
  return false; \
    } \
}

//------------------------------------------------------------------------------
static char g_ModuleName[MAX_PATH];

//------------------------------------------------------------------------------
static bool GlobalInit()
{
  GetModuleFileNameA(NULL, g_ModuleName, MAX_PATH);
  return true;
}

//------------------------------------------------------------------------------
static bool GlobalClose()
{
  return true;
}


//------------------------------------------------------------------------------
App::App()
  : _hinstance(NULL)
#if WITH_MUSIC
  , _sound(nullptr)
  , _system(nullptr)
  , _channel(nullptr)
#endif
{
}

//------------------------------------------------------------------------------
bool App::Create()
{
  if (!_instance)
    _instance = new App();

  return true;
}

//------------------------------------------------------------------------------
bool App::Destroy()
{
#if WITH_ANT_TWEAK_BAR
  TwTerminate();
#endif

  if (!ResourceManager::Destroy())
    return false;

  if (!DemoEngine::Destroy())
    return false;

  if (!Graphics::Destroy())
    return false;

  delete exch_null(_instance);
  return true;
}

//------------------------------------------------------------------------------
App& App::Instance()
{
  return *_instance;
}

//------------------------------------------------------------------------------
bool App::Init(HINSTANCE hinstance)
{
  _hinstance = hinstance;

  BEGIN_INIT_SEQUENCE();

#if WITH_UNPACKED_RESOUCES
  INIT(FindAppRoot("app.gb"));
#else
  INIT(FindAppRoot("resource.dat"));
#endif

  if (_appRoot.empty())
  {
    MessageBoxA(0, "Unable to find app root", "Error", MB_ICONEXCLAMATION);
    return false;
  }

  INIT(ResourceManager::Create("resources.txt"));
  INIT(LoadSettings());

  INIT(Graphics::Create(_hinstance));

  int width = GetSystemMetrics(SM_CXFULLSCREEN);
  int height = GetSystemMetrics(SM_CYFULLSCREEN);

  GRAPHICS.CreateDefaultSwapChain(3 * width / 4, 3 * height / 4, DXGI_FORMAT_R16G16B16A16_FLOAT, WndProc, hinstance);

  INIT(DemoEngine::Create());
  ParticleTunnel::Register();

/*
  DEMO_ENGINE.RegisterFactory(effect::EffectSetting::Type::Particle, ParticleTunnel::Create);
  //DEMO_ENGINE.RegisterFactory(effect::EffectSetting::Type::Particle, SceneTest::Create);
  DEMO_ENGINE.RegisterFactory(effect::EffectSetting::Type::Generator, GeneratorTest::Create);
  DEMO_ENGINE.RegisterFactory(effect::EffectSetting::Type::Plexus, PlexusTest::Create);
*/


  if (!DEMO_ENGINE.Init(_settings.demo_config.c_str(), hinstance))
  {
    DeferredContext* ctx = GRAPHICS.CreateDeferredContext(true);

    float black[] ={ 0, 0, 0, 0 };
    ctx->SetSwapChain(GRAPHICS.DefaultSwapChain(), black);
    // If we get this far, then the graphics are initialized, so just spin displaying an
    // error message
    while (true)
    {
      if (GetAsyncKeyState(VK_ESCAPE) & (1 << 15))
        break;

      GRAPHICS.Present();
    }

    GRAPHICS.DestroyDeferredContext(ctx);

    return false;
  }

#if WITH_MUSIC
  FMOD_CHECKED(FMOD::System_Create(&_system));

  u32 version;
  FMOD_CHECKED(_system->getVersion(&version));

  if (version < FMOD_VERSION)
  {
    LOG_ERROR(ToString("Error!  You are using an old version of FMOD %08x.  This program requires %08x\n", version, FMOD_VERSION).c_str());
    return false;
  }

  FMOD_CHECKED(_system->init(1, FMOD_INIT_NORMAL, 0));
  FMOD_CHECKED(_system->createStream("Distance.mp3", FMOD_HARDWARE | FMOD_LOOP_NORMAL | FMOD_2D, 0, &_sound));
#endif

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool App::Run()
{
  MSG msg ={ 0 };

  DEMO_ENGINE.Start();

#if WITH_MUSIC
  //FMOD_RESULT result = _system->playSound(FMOD_CHANNEL_FREE, _sound, false, &_channel);
#endif

  while (WM_QUIT != msg.message)
  {
    if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
    {
      TranslateMessage(&msg);
      DispatchMessage(&msg);
    }
    else
    {
      DEMO_ENGINE.Tick();

#if WITH_UNPACKED_RESOUCES
      RESOURCE_MANAGER.Tick();
#endif

#if WITH_MUSIC
      _system->update();
#endif
      GRAPHICS.Present();
    }
  }

#if WITH_MUSIC
  FMOD_CHECKED(_sound->release());
  FMOD_CHECKED(_system->close());
  FMOD_CHECKED(_system->release());
#endif

  return true;
}

//------------------------------------------------------------------------------
bool App::LoadSettings()
{
  BEGIN_INIT_SEQUENCE();

  vector<char> buf;
  INIT(RESOURCE_MANAGER.LoadFile(PathJoin(_appRoot.c_str(), "app.gb").c_str(), &buf));

  INIT(ParseAppSettings(InputBuffer(buf.data(), buf.size()), &_settings));
  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
void App::SaveSettings()
{
//   if (_appRootFilename.empty())
//     return;
}

//------------------------------------------------------------------------------
LRESULT App::WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
#if WITH_ANT_TWEAK_BAR
  if (TwEventWin(hWnd, message, wParam, lParam))
  {
    return 0;
  }
#endif
  switch (message)
  {
    case WM_SIZE:
      //GRAPHICS.Resize(LOWORD(lParam), HIWORD(lParam));
      break;

    case WM_APP_CLOSE:
      DestroyWindow(hWnd);
      break;

    case WM_DESTROY:
      PostQuitMessage(0);
      break;

    case WM_LBUTTONUP:
    case WM_MBUTTONUP:
    case WM_RBUTTONUP:
      break;

    case WM_LBUTTONDOWN:
    case WM_MBUTTONDOWN:
    case WM_RBUTTONDOWN:
      break;

    case WM_MOUSEMOVE:
      break;

    case WM_MOUSEWHEEL:
      break;

    case WM_KEYUP:

      switch (wParam) {

        case 'S':
#if WITH_UNPACKED_RESOUCES
          if (GetKeyState(VK_CONTROL) & (1 << 15))
          {
            TANO.SaveSettings();
            DEMO_ENGINE.SaveSettings();
            //App.AddMessage(MessageType::Info, "Settings saved");
          }
#endif
          break;

        case 'V':
          GRAPHICS.SetVSync(!GRAPHICS.VSync());
          return 0;

        case 'M': {
#if _DEBUG
          static _CrtMemState prevState;
          static bool firstTime = true;
          _CrtMemState curState;
          _CrtMemCheckpoint(&curState);

          if (!firstTime) {
            _CrtMemState stateDiff;
            OutputDebugStringA("*** MEM DIFF ***\n");
            _CrtMemDifference(&stateDiff, &prevState, &curState);
            _CrtMemDumpStatistics(&stateDiff);

          }
          firstTime = false;
          OutputDebugStringA("*** MEM STATS ***\n");
          _CrtMemDumpStatistics(&curState);
          prevState = curState;
#endif
          return 0;
        }

        case VK_ESCAPE:
          PostQuitMessage(0);
          return 0;

        case VK_SPACE:
          DEMO_ENGINE.SetPaused(!DEMO_ENGINE.Paused());
          return 0;

        case VK_PRIOR:
          DEMO_ENGINE.SetPos(DEMO_ENGINE.Pos() - TimeDuration::Seconds(1));
          return 0;

        case VK_NEXT:
          DEMO_ENGINE.SetPos(DEMO_ENGINE.Pos() + TimeDuration::Seconds(1));
          return 0;

        case VK_LEFT:
          DEMO_ENGINE.SetPos(DEMO_ENGINE.Pos() - TimeDuration::Milliseconds(100));
          return 0;

        case VK_RIGHT:
          DEMO_ENGINE.SetPos(DEMO_ENGINE.Pos() + TimeDuration::Milliseconds(100));
          return 0;

        case VK_HOME:
          DEMO_ENGINE.SetPos(TimeDuration::Seconds(0));
          return 0;

        default:
          break;
      }
      break;
  }

  if (DemoEngine::IsInitialized())
  {
    DEMO_ENGINE.WndProc(message, wParam, lParam);
  }

  return DefWindowProc(hWnd, message, wParam, lParam);
}

//------------------------------------------------------------------------------
bool App::FindAppRoot(const char* filename)
{
  char starting_dir[MAX_PATH];
  _getcwd(starting_dir, MAX_PATH);

  // keep going up directory levels until we find @filename, or we hit the bottom..
  char prev_dir[MAX_PATH], cur_dir[MAX_PATH];
  ZeroMemory(prev_dir, sizeof(prev_dir));

  while (true)
  {
    _getcwd(cur_dir, MAX_PATH);
    // check if we haven't moved
    if (!strcmp(cur_dir, prev_dir))
      return false;

    memcpy(prev_dir, cur_dir, MAX_PATH);

    if (FileExists(filename))
    {
      _appRoot = cur_dir;
      return true;
    }

    if (_chdir("..") == -1)
      break;
  }
  _appRoot = starting_dir;
  return false;
}

//------------------------------------------------------------------------------
int CALLBACK WinMain(HINSTANCE instance, HINSTANCE prev_instance, LPSTR cmd_line, int cmd_show)
{
  srand(1337);
#if WITH_MEMORY_TRACKING
  _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
  _CrtSetReportMode(_CRT_WARN, _CRTDBG_MODE_DEBUG);
  _CrtSetReportMode(_CRT_ERROR, _CRTDBG_MODE_DEBUG);
  _CrtSetReportMode(_CRT_ASSERT, _CRTDBG_MODE_DEBUG);
#endif

  if (!GlobalInit())
    return 1;

  if (!App::Create())
    return 1;

  if (!TANO.Init(instance))
    return 1;

  TANO.Run();

  if (!TANO.Destroy())
    return 1;

  GlobalClose();
  return 0;
}
