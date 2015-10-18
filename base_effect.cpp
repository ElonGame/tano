#include "base_effect.hpp"
#include "graphics.hpp"
#include "path_utils.hpp"
#include "resource_manager.hpp"
#include "property_manager.hpp"
#include "demo_engine.hpp"

using namespace tano;
using namespace bristol;

//------------------------------------------------------------------------------
void FindFileVersions(const char* base, vector<int>* versions, unordered_map<int, string>* versionToFile)
{
  versions->clear();
  versionToFile->clear();

  Path p(base);
  string path = p.GetPath();

  char wildcard[MAX_PATH];
  sprintf(wildcard, "%s*", base);

  WIN32_FIND_DATAA findData;
  HANDLE hFile = FindFirstFileA(wildcard, &findData);
  if (hFile != INVALID_HANDLE_VALUE)
  {
    char fullPath[MAX_PATH + 1];
    HRESULT hr = GetFinalPathNameByHandleA(hFile, fullPath, MAX_PATH, 0);
    do
    {
      int len = (int)strlen(findData.cFileName) - 1;
      int idx = len;
      while (idx >= 0 && isdigit(findData.cFileName[idx]))
        idx--;

      if (idx == len)
      {
        // no version found, so assume this is the canonical version
        versions->push_back(-1);
        versionToFile->insert(make_pair(-1, path + findData.cFileName));
      }
      else
      {
        int version;
        if (sscanf(&findData.cFileName[idx + 1], "%d", &version))
        {
          versions->push_back(version);
          versionToFile->insert(make_pair(version, path + findData.cFileName));
        }
      }

    } while (FindNextFileA(hFile, &findData));
  }
}

//------------------------------------------------------------------------------
BaseEffect::BaseEffect(const string& instanceName, const string& config, u32 id)
    : _instanceName(instanceName)
    , _configName(config)
    , _id(id)
    , _running(false)
    , _ctx(g_Graphics->GetGraphicsContext())
    , _firstTick(true)
{
  FindFileVersions(_configName.c_str(), &_configFileVersions, &_configVersionToFilename);
}

#if WITH_IMGUI
//------------------------------------------------------------------------------
void BaseEffect::RegisterParameters()
{
  PROPERTIES.Register(GetName(),
    _configFileVersions,
    bind(&BaseEffect::RenderParameterSet, this),
    bind(&BaseEffect::SaveParameterSet, this, _1),
    bind(&BaseEffect::LoadParameterVersion, this, _1));

  PROPERTIES.SetActive(GetName());
}
#endif

//------------------------------------------------------------------------------
bool BaseEffect::Show()
{
  return true;
}

//------------------------------------------------------------------------------
bool BaseEffect::Hide()
{
  return true;
}

//------------------------------------------------------------------------------
bool BaseEffect::OnConfigChanged(const vector<char>& buf)
{
  return true;
}

//------------------------------------------------------------------------------
bool BaseEffect::Init()
{
  return true;
}

//------------------------------------------------------------------------------
bool BaseEffect::InitAnimatedParameters()
{
  return true;
}

//------------------------------------------------------------------------------
bool BaseEffect::Update(const UpdateState& state)
{
  return true;
}

//------------------------------------------------------------------------------
bool BaseEffect::FixedUpdate(const FixedUpdateState& state)
{
  return true;
}

//------------------------------------------------------------------------------
bool BaseEffect::Render()
{
  return true;
}

//------------------------------------------------------------------------------
bool BaseEffect::Close()
{
  return true;
}

//------------------------------------------------------------------------------
void BaseEffect::SetDuration(TimeDuration startTime, TimeDuration endTime)
{
  _startTime = startTime;
  _endTime = endTime;
}

//------------------------------------------------------------------------------
void BaseEffect::SetStartTime(TimeDuration startTime)
{
  _startTime = startTime;
}

//------------------------------------------------------------------------------
bool BaseEffect::Running() const
{
  return _running;
}

//------------------------------------------------------------------------------
void BaseEffect::SetRunning(bool b)
{
  _running = b;
}

//------------------------------------------------------------------------------
void BaseEffect::LoadParameterVersion(int version)
{
  auto it = _configVersionToFilename.find(version);
  if (it == _configVersionToFilename.end())
  {
    LOG_WARN("Trying to load unknown config file version: ", version);
    return;
  }

  string filename = it->second;

  vector<char> buf;
  if (!RESOURCE_MANAGER.LoadFile(filename.c_str(), &buf))
  {
    LOG_WARN("Unable to load config file: ", filename);
    return;
  }

  _currentConfigVersion = version;

  OnConfigChanged(buf);
}

//------------------------------------------------------------------------------
void BaseEffect::CommonUpdate(float deltaTime)
{
  _freeflyCamera.Update(deltaTime);
}
