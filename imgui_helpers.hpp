#pragma once
#if WITH_IMGUI
#include "imgui/imgui_internal.h"

namespace tano
{
  bool InitImGui(HWND hWnd);
  void UpdateImGui();
  LRESULT WINAPI ImGuiWndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);
}

namespace ImGui
{
  typedef const float* PlotValues;
  void PlotLinesMulti(const char* label,
      PlotValues* values,
      int values_count,
      int values_offset = 0,
      int graph_count = 1,
      const char* overlay_text = NULL,
      float scale_min = FLT_MAX,
      float scale_max = FLT_MAX,
      ImVec2 graph_size = ImVec2(0, 0),
      int stride = sizeof(float));
}
#endif