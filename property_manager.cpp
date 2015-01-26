#include "property_manager.hpp"
#include "demo_engine.hpp"

using namespace tano;

//------------------------------------------------------------------------------
PropertyManager::PropertyManager()
{
  _comboString.push_back(0);
}

//------------------------------------------------------------------------------
void PropertyManager::Register(const char* label, const cbRenderCallback& cbRender, const cbSaveCallback& cbSave)
{
  // the combo string is a series of 0-termianted strings, with an extra 0 as a sentinal.
  // to add a new string, drop the sentinal, add the new guy, then add the sentinal.
  _comboString.pop_back();
  _comboString.insert(_comboString.end(), label, label + strlen(label) + 1);
  _comboString.push_back(0);

  _callbacks.push_back({ cbRender, cbSave });
}

//------------------------------------------------------------------------------
void PropertyManager::Tick()
{
  if (_callbacks.empty())
    return;

  if (!ImGui::Begin("Properties", &_windowOpened))
  {
    // FIXME: this path is never taken
    ImGui::End();
    return;
  }

  ImGui::LabelText("Time", "%f", DEMO_ENGINE.Pos().TotalMilliseconds() / 1000.f);

  // draw the combo box, and invoke the selected effects renderer
  ImGui::Combo("parameter set", &_curItem, _comboString.data());
  ImGui::Separator();
  _callbacks[_curItem].render();
  ImGui::Separator();
  if (ImGui::Button("save"))
    _callbacks[_curItem].save();

  if (ImGui::Button("pause"))
    DEMO_ENGINE.SetPaused(!DEMO_ENGINE.Paused());
  ImGui::SameLine();

  if (ImGui::Button("reset"))
    DEMO_ENGINE.SetDuration(TimeDuration());
  ImGui::SameLine();

  ImGui::End();
}
