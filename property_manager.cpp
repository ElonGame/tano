#if WITH_IMGUI

#include "property_manager.hpp"
#include "demo_engine.hpp"

using namespace tano;

//------------------------------------------------------------------------------
PropertyManager::PropertyManager()
{
}

//------------------------------------------------------------------------------
void PropertyManager::Register(const char* label,
    const vector<int>& versions,
    const cbRenderCallback& cbRender,
    const cbSaveCallback& cbSave,
    const cbSetVersionCallback& cbSetVersion)
{
  bool found = false;
  for (Properties& p : _properties)
  {
    // Overwrite existing version
    if (p.label == label)
    {
      p.versions = versions;
      p.cbRender = cbRender;
      p.cbSave = cbSave;
      p.cbSetVersion = cbSetVersion;
      p.StringifyProperties();
      found = true;
      break;
    }
  }

  if (!found)
  {
    _properties.push_back(Properties{string(label), versions, 0, cbRender, cbSave, cbSetVersion});
    _properties.back().StringifyProperties();
  }
}

//------------------------------------------------------------------------------
void PropertyManager::SetActive(const char* label)
{
  _curItem = 0;
  for (int i = 0; i < (int)_properties.size(); ++i)
  {
    if (_properties[i].label == label)
    {
      _curItem = i;
      break;
    }
  }
}

//------------------------------------------------------------------------------
void PropertyManager::Tick()
{
  if (ImGui::Begin("Properties", &_windowOpened, ImGuiWindowFlags_AlwaysAutoResize))
  {
    ImGui::LabelText("Time", "%f", DEMO_ENGINE.Pos().TotalMilliseconds() / 1000.f);

    auto fnGetParam = [](void* data, int idx, const char** out_text)
    {
      PropertyManager* self = (PropertyManager*)data;
      *out_text = self->_properties[idx].label.c_str();
      return true;
    };

    // Draw the parameter set combo box
    ImGui::Combo("parameter set", &_curItem, fnGetParam, this, (int)_properties.size());

    // Call the selected effect's parameter renderer
    if (_curItem < _properties.size())
    {
      Properties& p = _properties[_curItem];

      if (ImGui::Button("save"))
        p.cbSave(false);
      ImGui::SameLine();
      if (ImGui::Button("+1"))
        p.cbSave(true);
      ImGui::SameLine();

      auto fnGetVersion = [](void* data, int idx, const char** out_text)
      {
        PropertyManager* self = (PropertyManager*)data;
        const Properties& p = self->_properties[self->_curItem];
        *out_text = p.stringVersions[idx].c_str();
        return true;
      };

      if (ImGui::Combo("version", &p.curVersion, fnGetVersion, this, (int)p.versions.size()))
      {
        p.cbSetVersion(p.versions[p.curVersion]);
      }

      ImGui::Separator();
      p.cbRender();
    }
  }

  ImGui::End();
}

#endif