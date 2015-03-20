#pragma once
#include "generated/app.types.hpp"

struct sync_device;

namespace tano
{

  struct IoState
  {
    enum Button
    {
      ButtonLeft,
      ButtonMiddle,
      ButtonRight,
    };

    float deltaTime;
    float mouseX, mouseY;
    float wheel;
    u8 keysPressed[256];
    u8 shiftPressed : 1;
    u8 controlPressed : 1;
    u8 altPressed : 1;
  };

  class App
  {
  public:

    static App& Instance();

    bool Init(HINSTANCE hinstance);
    bool Run();

    static bool Create();
    static bool Destroy();

    const IoState& GetIoState() const;

  private:
    App();

    bool FindAppRoot(const char* filename);
    bool LoadSettings();
    void SaveSettings();

    static LRESULT WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

    static App* _instance;
    HINSTANCE _hinstance;

    string _appRoot;
    AppSettings _settings;

    bristol::LogSinkApp _logSinkApp;
    bristol::LogSinkConsole _logSinkConsole;

    IoState _ioState;
  };
}

#define TANO tano::App::Instance()
