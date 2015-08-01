#pragma once
#if WITH_IMGUI

namespace tano
{
  class PropertyManager
  {
  public:
    typedef function<void()> cbRenderCallback;
    typedef function<void(bool)> cbSaveCallback;
    typedef function<void(int)> cbSetVersionCallback;

    PropertyManager();
    void Register(const char* label,
        const vector<int>& versions,
        const cbRenderCallback& cbRender,
        const cbSaveCallback& cbSave,
        const cbSetVersionCallback& cbSetVersion);
    void Tick();
    void SetActive(const char* label);

  private:
    struct Properties
    {
      void StringifyProperties()
      {
        stringVersions.clear();
        for (int i : versions)
        {
          char buf[32];
          sprintf(buf, "%d", i);
          stringVersions.push_back(buf);
        }
      }

      string label;
      vector<int> versions;
      int curVersion;

      cbRenderCallback cbRender;
      cbSaveCallback cbSave;
      cbSetVersionCallback cbSetVersion;
      vector<string> stringVersions;
    };

    int _curItem = 0;

    vector<Properties> _properties;
    bool _windowOpened = true;
  };
}

#endif