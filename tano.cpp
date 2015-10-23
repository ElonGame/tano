#include "tano.hpp"
#include "resource_manager.hpp"
#include "demo_engine.hpp"
#include "graphics.hpp"
#include "graphics_extra.hpp"
#include "graphics_context.hpp"
#include "init_sequence.hpp"
#include "debug_api.hpp"
#include "scheduler.hpp"
#include "arena_allocator.hpp"
#include "stop_watch.hpp"
#include "perlin2d.hpp"
#include "blackboard.hpp"
#include "generated/app.parse.hpp"
#include "effects/intro.hpp"
#include "effects/landscape.hpp"
#include "effects/credits.hpp"
#include "effects/plexus.hpp"
#include "effects/tunnel.hpp"
#include "effects/Tubes.hpp"
#include "random.hpp"

#if WITH_IMGUI
#include "imgui_helpers.hpp"
#endif

#pragma comment(lib, "winmm.lib")

//------------------------------------------------------------------------------
using namespace tano;
using namespace tano::scheduler;
using namespace bristol;

static const int WM_APP_CLOSE = WM_APP + 2;

const TCHAR* g_AppWindowClass = _T("TanoClass");
const TCHAR* g_AppWindowTitle = _T("radio silence- neurotica e.f.s");

const int ARENA_MEMORY_SIZE = 512 * 1024 * 1024;
static u8 scratchMemory[ARENA_MEMORY_SIZE];

namespace tano
{
  ArenaAllocator g_ScratchMemory;
}

KeyUpTrigger tano::g_KeyUpTrigger;

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
{
  memset(&_ioState, 0, sizeof(_ioState));
  bristol::SetLogCallback([](const LogEntry& entry)
      {
        InitSequence::AddLog(entry.file, entry.line, entry.msg);
      });
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

  Blackboard::Destory();
  INIT(RESOURCE_MANAGER_STATIC::Destroy());
  DemoEngine::Destroy();
  DebugApi::Destroy();
  INIT(Graphics::Destroy());
#if WITH_ENKI_SCHEDULER
  g_TS.StopThreads(true)
#else
  Scheduler::Destroy();
#endif

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
  BEGIN_INIT_SEQUENCE();

#if WITH_UNPACKED_RESOUCES
  INIT_FATAL(FindAppRoot("app.gb"));
#else
  INIT_FATAL(FindAppRoot("resources.dat"));
#endif

  if (_appRoot.empty())
  {
    MessageBoxA(0, "Unable to find app root", "Error", MB_ICONEXCLAMATION);
    return false;
  }

#if WITH_UNPACKED_RESOUCES
  INIT_FATAL(RESOURCE_MANAGER_STATIC::Create("resources.txt", _appRoot.c_str()));
  RESOURCE_MANAGER.AddPath("D:/OneDrive/tano");
  RESOURCE_MANAGER.AddPath("C:/OneDrive/tano");
#else
  INIT_FATAL(RESOURCE_MANAGER_STATIC::Create("resources.dat"));
#endif
  bool b = LoadSettings();
  INIT_FATAL(LoadSettings());

  INIT_FATAL(Blackboard::Create("config/scratch.bb", "data/scratch.dat"));

#if WITH_ENKI_SCHEDULER
  g_TS.Initialize();
#else
  INIT_FATAL(Scheduler::Create());
#endif

#if WITH_CONFIG_DLG
  INIT_FATAL(Graphics::CreateWithConfigDialog(hinstance, WndProc));
#else
  INIT_FATAL(Graphics::Create(hinstance));

  int width = GetSystemMetrics(SM_CXFULLSCREEN);
  int height = GetSystemMetrics(SM_CYFULLSCREEN);
  width = (int)(0.75f * width);
  height = (int)(0.75f * height);

  int bbWidth = width / BACKBUFFER_SCALE;
  int bbHeight = height / BACKBUFFER_SCALE;

  g_Graphics->CreateDefaultSwapChain(
      width, height, bbWidth, bbHeight, true, DXGI_FORMAT_R8G8B8A8_UNORM, WndProc, hinstance);
#endif
  INIT_FATAL(DebugApi::Create(g_Graphics->GetGraphicsContext()));

  INIT_FATAL(g_ScratchMemory.Init(scratchMemory, scratchMemory + ARENA_MEMORY_SIZE));
  Perlin2D::Init();

#if WITH_IMGUI
  INIT_FATAL(InitImGui(g_Graphics->GetSwapChain(g_Graphics->DefaultSwapChain())->_hwnd));
#endif

#if WITH_REMOTERY
  rmt_CreateGlobalInstance(&g_rmt);
#endif

  DemoEngine::Create();

  Intro::Register();
  Landscape::Register();
  Credits::Register();
  Plexus::Register();
  Tunnel::Register();
  Tubes::Register();

  INIT_FATAL(g_DemoEngine->Init(_settings.demo_config.c_str(), hinstance));

  END_INIT_SEQUENCE();
}

//------------------------------------------------------------------------------
bool App::Run()
{
  MSG msg = {0};

  g_DemoEngine->Start();

  RollingAverage<float> avgFrameTime(200);
  StopWatch stopWatch;
  u64 numFrames = 0;
  bool renderImgui = true;

  while (WM_QUIT != msg.message && !g_DemoEngine->IsDone())
  {
    rmt_ScopedCPUSample(App_Run);

    g_ScratchMemory.NewFrame();
    _perfCallbacks.clear();
    {
      rmt_ScopedCPUSample(App_PeekMessage);
      if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
      {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
        continue;
      }
    }
    stopWatch.Start();

    UpdateIoState();

#if WITH_DEBUG_API
    DEBUG_API.BeginFrame();
#endif

#if WITH_IMGUI
    UpdateImGui();
#endif

// ImGui::ShowTestWindow();
    g_Graphics->ClearRenderTarget(g_Graphics->GetBackBuffer());

#if WITH_IMGUI && WITH_EXPRESSION_EDITOR
    static bool showExpressionEditor = false;
    static bool showRandomDistribution = false;
    if (g_KeyUpTrigger.IsTriggered('0')) showExpressionEditor = !showExpressionEditor;
    if (g_KeyUpTrigger.IsTriggered('9')) showRandomDistribution = !showRandomDistribution;

    if (showExpressionEditor)
      g_Blackboard->DrawExpressionEditor();

    if (showRandomDistribution)
      ShowRandomDistribution();
#endif

    g_DemoEngine->Tick();

#if WITH_UNPACKED_RESOUCES
    RESOURCE_MANAGER.Tick();
#endif

#if WITH_DEBUG_API
    DEBUG_API.EndFrame();
#endif

#if WITH_BLACKBOARD_TCP
    g_Blackboard->Process();
#endif

#if WITH_IMGUI
    float times[200];
    size_t numSamples;
    avgFrameTime.CopySamples(times, &numSamples);
    if (numSamples > 0)
    {
      ImGui::Begin("Perf", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
      float minValue, maxValue;
      avgFrameTime.GetMinMax(&minValue, &maxValue);
      ImGui::Text("Min: %.2f Max: %.2f Avg: %.2f",
          minValue * 1000,
          maxValue * 1000,
          avgFrameTime.GetAverage() * 1000);
      ImGui::PlotLines(
          "Frame time", times, (int)numSamples, 0, 0, FLT_MAX, FLT_MAX, ImVec2(200, 50));

      // Invoke any custom perf callbacks
      for (const fnPerfCallback& cb : _perfCallbacks)
        cb();
      ImGui::End();
    }

    if (g_KeyUpTrigger.IsTriggered('H'))
      renderImgui = !renderImgui;
    if (renderImgui)
      ImGui::Render();
#endif

    double frameTime = stopWatch.Stop();
    if (++numFrames > 10)
      avgFrameTime.AddSample((float)frameTime);

    g_Graphics->Present();
  }

  return true;
}

//------------------------------------------------------------------------------
bool App::LoadSettings()
{
  BEGIN_INIT_SEQUENCE();

  vector<char> buf;
  // INIT(RESOURCE_MANAGER.LoadFile(PathJoin(_appRoot.c_str(), "app.gb").c_str(), &buf));
  INIT_FATAL(RESOURCE_MANAGER.LoadFile("app.gb", &buf));
  _settings = ParseAppSettings(InputBuffer(buf.data(), buf.size()));

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
  ImGuiWndProc(hWnd, message, wParam, lParam);
#endif

  switch (message)
  {
    case WM_SIZE:
      // g_Graphics->Resize(LOWORD(lParam), HIWORD(lParam));
      break;

    case WM_APP_CLOSE: DestroyWindow(hWnd); break;

    case WM_DESTROY: PostQuitMessage(0); break;

    case WM_LBUTTONUP: TANO._ioState.buttons[IoState::ButtonLeft] = false; break;

    case WM_MBUTTONUP: TANO._ioState.buttons[IoState::ButtonMiddle] = false; break;

    case WM_RBUTTONUP: TANO._ioState.buttons[IoState::ButtonRight] = false; break;

    case WM_LBUTTONDOWN: TANO._ioState.buttons[IoState::ButtonLeft] = true; break;

    case WM_MBUTTONDOWN: TANO._ioState.buttons[IoState::ButtonMiddle] = true; break;

    case WM_RBUTTONDOWN: TANO._ioState.buttons[IoState::ButtonRight] = true; break;

    case WM_MOUSEMOVE:
      TANO._ioState.mouseX = (signed short)(lParam);
      TANO._ioState.mouseY = (signed short)(lParam >> 16);
      break;

    case WM_MOUSEWHEEL:
      TANO._ioState.mouseWheel += GET_WHEEL_DELTA_WPARAM(wParam) > 0 ? +1.0f : -1.0f;
      break;

    case WM_KEYDOWN:

      switch (wParam)
      {
        case VK_LEFT:
          g_DemoEngine->SetPos(g_DemoEngine->Pos() - TimeDuration::Seconds(1));
          return 0;

        case VK_RIGHT:
          g_DemoEngine->SetPos(g_DemoEngine->Pos() + TimeDuration::Seconds(1));
          return 0;

        case VK_UP:
          g_DemoEngine->SetPos(g_DemoEngine->Pos() - TimeDuration::Milliseconds(100));
          return 0;

        case VK_DOWN:
          g_DemoEngine->SetPos(g_DemoEngine->Pos() + TimeDuration::Milliseconds(100));
          return 0;
      }
      break;

    case WM_KEYUP:

      switch (wParam)
      {

        case 'S':
#if WITH_UNPACKED_RESOUCES
          if (GetKeyState(VK_CONTROL) & (1 << 15))
          {
            TANO.SaveSettings();
          }
#endif
          break;

        case 'V': g_Graphics->SetVSync(!g_Graphics->GetVSync()); return 0;

        case 'M': {
#if _DEBUG
          static _CrtMemState prevState;
          static bool firstTime = true;
          _CrtMemState curState;
          _CrtMemCheckpoint(&curState);

          if (!firstTime)
          {
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

        case VK_ESCAPE: PostQuitMessage(0); return 0;

        case VK_SPACE: g_DemoEngine->SetPaused(!g_DemoEngine->Paused()); return 0;

        case VK_PRIOR:
          g_DemoEngine->SetPos(g_DemoEngine->Pos() - TimeDuration::Seconds(30));
          return 0;

        case VK_NEXT:
          g_DemoEngine->SetPos(g_DemoEngine->Pos() + TimeDuration::Seconds(30));
          return 0;

        case VK_HOME: g_DemoEngine->SetPos(TimeDuration::Seconds(0)); return 0;

        default: break;
      }
      break;
  }

  if (g_DemoEngine)
  {
    g_DemoEngine->WndProc(message, wParam, lParam);
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
void App::AddPerfCallback(const fnPerfCallback& cb)
{
  _perfCallbacks.push_back(cb);
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
