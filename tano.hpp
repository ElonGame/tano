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
      NumButtons
    };

    float deltaTime;
    float mouseX, mouseY;
    float mouseWheel;
    bool buttons[NumButtons];
    u8 keysPressed[256];
    u8 shiftPressed : 1;
    u8 controlPressed : 1;
    u8 altPressed : 1;
  };

  struct KeyUpTrigger
  {
    KeyUpTrigger()
    {
      memset(triggers, 0, 512);
    }

    bool IsTriggered(int key)
    {
      bool tmp = triggers[key];
      triggers[key] = false;
      return tmp;
    }

    void SetTrigger(int key)
    {
      triggers[key] = true;
    }

    bool triggers[512];
  };

  extern KeyUpTrigger g_KeyUpTrigger;


  class App
  {
  public:

    static App& Instance();

    bool Init(HINSTANCE hinstance);
    bool Run();

    static bool Create();
    static bool Destroy();

    const IoState& GetIoState() const;

    typedef function<void()> fnPerfCallback;
    void AddPerfCallback(const fnPerfCallback& cb);

  private:
    App();

    bool FindAppRoot(const char* filename);
    bool LoadSettings();
    void SaveSettings();
    void UpdateIoState();

    static LRESULT WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

    static App* _instance;
    HINSTANCE _hinstance;

    string _appRoot;
    AppSettings _settings;

    bristol::LogSinkApp _logSinkApp;
    bristol::LogSinkConsole _logSinkConsole;
    vector<function<void()>> _perfCallbacks;

    IoState _ioState;
  };
}

#define TANO tano::App::Instance()
