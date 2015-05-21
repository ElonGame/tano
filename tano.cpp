#include "tano.hpp"
#include "resource_manager.hpp"
#include "demo_engine.hpp"
#include "graphics.hpp"
#include "graphics_context.hpp"
#include "init_sequence.hpp"
#include "generated/app.parse.hpp"
#include "generated/input_buffer.hpp"
#include "effects/particle_tunnel.hpp"
#include "effects/raymarcher.hpp"
#include "effects/cluster.hpp"
#include "effects/landscape.hpp"
#include "effects/blob.hpp"
#include "effects/cloth.hpp"

#if WITH_IMGUI
#include "imgui_helpers.hpp"
#endif

//------------------------------------------------------------------------------
using namespace tano;
using namespace bristol;

static const int WM_APP_CLOSE = WM_APP + 2;

const TCHAR* g_AppWindowClass = _T("TanoClass");
const TCHAR* g_AppWindowTitle = _T("tano - neurotica e.f.s");

#if WITH_REMOTERY
Remotery* g_rmt;
#endif

App* App::_instance;

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
{
  memset(&_ioState, 0, sizeof(_ioState));
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
  BEGIN_INIT_SEQUENCE();

#if WITH_REMOTERY
  rmt_DestroyGlobalInstance(g_rmt);
  g_rmt = nullptr;
#endif

  INIT(RESOURCE_MANAGER_STATIC::Destroy());
  INIT(DemoEngine::Destroy());
  INIT(Graphics::Destroy());

  delete exch_null(_instance);

  END_INIT_SEQUENCE();
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
  INIT(FindAppRoot("resources.dat"));
#endif

  if (_appRoot.empty())
  {
    MessageBoxA(0, "Unable to find app root", "Error", MB_ICONEXCLAMATION);
    return false;
  }

#if WITH_UNPACKED_RESOUCES
  INIT(RESOURCE_MANAGER_STATIC::Create("resources.txt", _appRoot.c_str()));
#else
  INIT(RESOURCE_MANAGER_STATIC::Create("resources.dat"));
#endif
  INIT(LoadSettings());

  INIT(Graphics::Create(_hinstance));

  int width = GetSystemMetrics(SM_CXFULLSCREEN);
  int height = GetSystemMetrics(SM_CYFULLSCREEN);
  width = (int)(0.75f * width);
  height = (int)(0.75f * height);

  //width = 800;
  //height = 600;

  GRAPHICS.CreateDefaultSwapChain(width, height, DXGI_FORMAT_R16G16B16A16_FLOAT, WndProc, hinstance);

#if WITH_IMGUI
  INIT(InitImGui());
#endif

#if WITH_REMOTERY
  rmt_CreateGlobalInstance(&g_rmt);
#endif

  INIT(DemoEngine::Create());
  ParticleTunnel::Register();
  RayMarcher::Register();
  Cluster::Register();
  Landscape::Register();
  Blob::Register();
  Cloth::Register();

  INIT(DEMO_ENGINE.Init(_settings.demo_config.c_str(), hinstance));

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool App::Run()
{
  MSG msg ={ 0 };

  DEMO_ENGINE.Start();

  while (WM_QUIT != msg.message)
  {
    if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
    {
      TranslateMessage(&msg);
      DispatchMessage(&msg);
      continue;
    }
    rmt_ScopedCPUSample(App_Run);

    UpdateIoState();

#if WITH_IMGUI
    UpdateImGui();
#endif

    DEMO_ENGINE.Tick();

#if WITH_UNPACKED_RESOUCES
    RESOURCE_MANAGER.Tick();
#endif

#if WITH_IMGUI
    ImGui::Render();
#endif
    GRAPHICS.Present();
  }

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
}

//------------------------------------------------------------------------------
const IoState& App::GetIoState() const
{
  return _ioState;
}

//------------------------------------------------------------------------------
LRESULT App::WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
#if WITH_IMGUI
  if (ImGuiWndProc(hWnd, message, wParam, lParam))
    return 0;
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
      TANO._ioState.buttons[IoState::ButtonLeft] = false;
      break;

    case WM_MBUTTONUP:
      TANO._ioState.buttons[IoState::ButtonMiddle] = false;
      break;

    case WM_RBUTTONUP:
      TANO._ioState.buttons[IoState::ButtonRight] = false;
      break;

    case WM_LBUTTONDOWN:
      TANO._ioState.buttons[IoState::ButtonLeft] = true;
      break;

    case WM_MBUTTONDOWN:
      TANO._ioState.buttons[IoState::ButtonMiddle] = true;
      break;

    case WM_RBUTTONDOWN:
      TANO._ioState.buttons[IoState::ButtonRight] = true;
      break;

    case WM_MOUSEMOVE:
      TANO._ioState.mouseX = (signed short)(lParam);
      TANO._ioState.mouseY = (signed short)(lParam >> 16);
      break;

    case WM_MOUSEWHEEL:
      TANO._ioState.mouseWheel += GET_WHEEL_DELTA_WPARAM(wParam) > 0 ? +1.0f : -1.0f;
      break;

    case WM_KEYUP:

      switch (wParam) {

        case 'S':
#if WITH_UNPACKED_RESOUCES
          if (GetKeyState(VK_CONTROL) & (1 << 15))
          {
            TANO.SaveSettings();
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
          DEMO_ENGINE.SetPos(DEMO_ENGINE.Pos() - TimeDuration::Seconds(30));
          return 0;

        case VK_NEXT:
          DEMO_ENGINE.SetPos(DEMO_ENGINE.Pos() + TimeDuration::Seconds(30));
          return 0;

        case VK_LEFT:
          DEMO_ENGINE.SetPos(DEMO_ENGINE.Pos() - TimeDuration::Seconds(1));
          return 0;

        case VK_RIGHT:
          DEMO_ENGINE.SetPos(DEMO_ENGINE.Pos() + TimeDuration::Seconds(1));
          return 0;

        case VK_UP:
          DEMO_ENGINE.SetPos(DEMO_ENGINE.Pos() - TimeDuration::Milliseconds(100));
          return 0;

        case VK_DOWN:
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
void App::UpdateIoState()
{
  u8 keystate[256];
  GetKeyboardState(keystate);
  for (int i = 0; i < 256; i++)
    _ioState.keysPressed[i] = (keystate[i] & 0x80) != 0;

  _ioState.controlPressed = (keystate[VK_CONTROL] & 0x80) != 0;
  _ioState.shiftPressed = (keystate[VK_SHIFT] & 0x80) != 0;
  _ioState.altPressed = (keystate[VK_MENU] & 0x80) != 0;
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

