#pragma once
#if WITH_IMGUI

namespace tano
{
  class PropertyManager
  {
  public:
    typedef function<void()> cbRenderCallback;
    typedef function<void()> cbSaveCallback;

    PropertyManager();
    void Register(const char* label, const cbRenderCallback& cbRender, const cbSaveCallback& cbSave);
    void Tick();

  private:
    struct Callbacks
    {
      cbRenderCallback render;
      cbSaveCallback save;
    };

    int _curItem = 0;
    vector<Callbacks> _callbacks;
    vector<char> _comboString;
    bool _windowOpened = true;
  };

}

#endif