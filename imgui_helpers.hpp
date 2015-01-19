#pragma once

namespace tano
{
  bool InitImGui();
  void UpdateImGui();
  LRESULT WINAPI ImGuiWndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);
}