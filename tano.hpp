#pragma once

namespace tano
{
  class App
  {
  public:

    static App& Instance();

    bool Init(HINSTANCE hinstance);
    bool Run();

    static bool Create();
    static bool Destroy();

  private:
    App();

    void FindAppRoot();
    void LoadSettings();
    void SaveSettings();

    static LRESULT WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

    static App* _instance;
    HINSTANCE _hinstance;

#if WITH_MUSIC
    FMOD::System *_system;
    FMOD::Sound  *_sound;
    FMOD::Channel *_channel;
#endif

    string _appRootFilename;
    string _appRoot;

    bristol::LogSinkApp _logSinkApp;
    bristol::LogSinkConsole _logSinkConsole;
  };
}

#define TANO tano::App::Instance()
