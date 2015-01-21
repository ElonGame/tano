#include "graphics_extra.hpp"
#include "graphics.hpp"
#include "_win32/resource.h"

using namespace tano;
using namespace bristol;

namespace tano
{
  //------------------------------------------------------------------------------
  bool InitConfigDialog(HINSTANCE hInstance)
  {
    int res = (int)DialogBox(hInstance, MAKEINTRESOURCE(IDD_SETUP_DLG), NULL, dialogWndProc);
    return res == IDOK;
  }

  //------------------------------------------------------------------------------
  bool EnumerateDisplayModes(HWND hWnd)
  {
    vector<VideoAdapter>& videoAdapters = GRAPHICS._curSetup.videoAdapters;
    videoAdapters.clear();

    // Create DXGI factory to enumerate adapters
    if (FAILED(CreateDXGIFactory(IID_PPV_ARGS(&GRAPHICS._curSetup.dxgi_factory))))
    {
      return false;
    }

    // Enumerate the adapters
    IDXGIAdapter* adapter;
    for (int i = 0; SUCCEEDED(GRAPHICS._curSetup.dxgi_factory->EnumAdapters(i, &adapter)); ++i)
    {
      videoAdapters.push_back(VideoAdapter());
      VideoAdapter &curAdapter = videoAdapters.back();
      curAdapter.adapter = adapter;
      adapter->GetDesc(&curAdapter.desc);
      HWND hAdapter = GetDlgItem(hWnd, IDC_VIDEO_ADAPTER);
      ComboBox_AddString(hAdapter, wide_char_to_utf8(curAdapter.desc.Description).c_str());
      ComboBox_SetCurSel(hAdapter, 0);
      GRAPHICS._curSetup.selectedAdapter = 0;

      CComPtr<IDXGIOutput> output;
      vector<CComPtr<IDXGIOutput>> outputs;
      vector<DXGI_MODE_DESC> displayModes;

      // Only enumerate the first adapter
      for (int j = 0; SUCCEEDED(adapter->EnumOutputs(j, &output)); ++j)
      {
        DXGI_OUTPUT_DESC outputDesc;
        output->GetDesc(&outputDesc);
        UINT numModes;
        output->GetDisplayModeList(DXGI_FORMAT_R8G8B8A8_UNORM, 0, &numModes, NULL);
        size_t prevSize = displayModes.size();
        displayModes.resize(displayModes.size() + numModes);
        output->GetDisplayModeList(DXGI_FORMAT_R8G8B8A8_UNORM, 0, &numModes, displayModes.data() + prevSize);
        if (!GRAPHICS._displayAllModes)
          break;
      }

      // Only keep the version of each display mode with the highest refresh rate
      auto &safeRational = [](const DXGI_RATIONAL &r) { return r.Denominator == 0 ? 0 : r.Numerator / r.Denominator; };
      if (GRAPHICS.DisplayAllModes())
      {
        curAdapter.displayModes = displayModes;
      }
      else
      {
        map<pair<int, int>, DXGI_RATIONAL> highestRate;
        for (size_t i = 0; i < displayModes.size(); ++i)
        {
          auto &cur = displayModes[i];
          auto key = make_pair(cur.Width, cur.Height);
          if (safeRational(cur.RefreshRate) > safeRational(highestRate[key]))
          {
            highestRate[key] = cur.RefreshRate;
          }
        }

        for (auto it = begin(highestRate); it != end(highestRate); ++it)
        {
          DXGI_MODE_DESC desc;
          desc.Width = it->first.first;
          desc.Height = it->first.second;
          desc.RefreshRate = it->second;
          curAdapter.displayModes.push_back(desc);
        }

        auto &resSorter = [&](const DXGI_MODE_DESC &a, const DXGI_MODE_DESC &b)
        {
          return a.Width < b.Width;
        };

        sort(begin(curAdapter.displayModes), end(curAdapter.displayModes), resSorter);
      }

      HWND hDisplayMode = GetDlgItem(hWnd, IDC_DISPLAY_MODES);
      for (size_t k = 0; k < curAdapter.displayModes.size(); ++k)
      {
        auto &cur = curAdapter.displayModes[k];
        char buf[256];
        sprintf(buf, "%dx%d (%dHz)", cur.Width, cur.Height, safeRational(cur.RefreshRate));
        ComboBox_InsertString(hDisplayMode, k, buf);
        // encode width << 16 | height in the item data
        ComboBox_SetItemData(hDisplayMode, k, (cur.Width << 16) | cur.Height);
      }

      int cnt = ComboBox_GetCount(hDisplayMode);
      ComboBox_SetCurSel(hDisplayMode, cnt-1);

      adapter->Release();
    }

    HWND hMultisample = GetDlgItem(hWnd, IDC_MULTISAMPLE);
    ComboBox_InsertString(hMultisample, -1, "No multisample");
    ComboBox_InsertString(hMultisample, -1, "2x multisample");
    ComboBox_InsertString(hMultisample, -1, "4x multisample");
    ComboBox_InsertString(hMultisample, -1, "8x multisample");

    ComboBox_SetItemData(hMultisample, 0, 1);
    ComboBox_SetItemData(hMultisample, 1, 2);
    ComboBox_SetItemData(hMultisample, 2, 4);
    ComboBox_SetItemData(hMultisample, 3, 8);

    ComboBox_SetCurSel(hMultisample, 0);
    GRAPHICS._curSetup.multisampleCount = 1;

#if DISTRIBUTION
    GRAPHICS._curSetup.windowed = false;
#else
    GRAPHICS._curSetup.windowed = true;
    Button_SetCheck(GetDlgItem(hWnd, IDC_WINDOWED), TRUE);
#endif

    return true;
  }

  //------------------------------------------------------------------------------
  INT_PTR CALLBACK dialogWndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
  {
    switch (message)
    {
      case WM_INITDIALOG:
      {
        if (!EnumerateDisplayModes(hWnd))
        {
          EndDialog(hWnd, -1);
        }
#if !WITH_CONFIG_DLG
        if (!GRAPHICS._curSetup.videoAdapters.empty())
        {
          ComboBox_SetCurSel(GetDlgItem(hWnd, IDC_DISPLAY_MODES), 3 * GRAPHICS._curSetup.videoAdapters[0].displayModes.size() / 4);
        }
        EndDialog(hWnd, 1);
#endif
        RECT rect;
        GetClientRect(hWnd, &rect);
        int width = GetSystemMetrics(SM_CXSCREEN);
        int height = GetSystemMetrics(SM_CYSCREEN);
        SetWindowPos(hWnd, NULL, width/2 - (rect.right - rect.left) / 2, height/2 - (rect.bottom - rect.top)/2, -1, -1,
          SWP_NOZORDER | SWP_NOSIZE);
        break;
      }

      case WM_COMMAND:
        switch (LOWORD(wParam))
        {
          case IDCANCEL:
            EndDialog(hWnd, 0);
            return TRUE;

          case IDOK:
            EndDialog(hWnd, 1);
            return TRUE;
        }
        break; // end WM_COMMAND

      case WM_DESTROY:
      {
        // read the settings
        Setup& cur = GRAPHICS._curSetup;
        cur.windowed = !!Button_GetCheck(GetDlgItem(hWnd, IDC_WINDOWED));
        cur.SelectedDisplayMode = ComboBox_GetCurSel(GetDlgItem(hWnd, IDC_DISPLAY_MODES));

        HWND hMultisample = GetDlgItem(hWnd, IDC_MULTISAMPLE);
        cur.multisampleCount = (int)ComboBox_GetItemData(hMultisample, ComboBox_GetCurSel(hMultisample));

        HWND hDisplayModes = GetDlgItem(hWnd, IDC_DISPLAY_MODES);
        int sel = ComboBox_GetCurSel(hDisplayModes);
        int data = (int)ComboBox_GetItemData(hDisplayModes, sel);
        cur.width = HIWORD(data);
        cur.height = LOWORD(data);
        break;
      }
    }
    return FALSE;
  }

  //------------------------------------------------------------------------------
  void SetClientSize(HWND hwnd, int width, int height)
  {
    RECT client_rect;
    RECT window_rect;
    GetClientRect(hwnd, &client_rect);
    GetWindowRect(hwnd, &window_rect);
    window_rect.right -= window_rect.left;
    window_rect.bottom -= window_rect.top;
    const int dx = window_rect.right - client_rect.right;
    const int dy = window_rect.bottom - client_rect.bottom;

    SetWindowPos(hwnd, NULL, -1, -1, width + dx, height + dy, SWP_NOZORDER | SWP_NOREPOSITION);
  }

  //------------------------------------------------------------------------------
  void VertexFlagsToLayoutDesc(u32 vertexFlags, vector<D3D11_INPUT_ELEMENT_DESC>* desc)
  {
    u32 ofs = 0;
    if (vertexFlags & VF_POS)
    {
      desc->push_back({ "POSITION", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 });
      ofs += 12;
    }

    if (vertexFlags & VF_POS_XY)
    {
      desc->push_back({ "POSITION", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 });
      ofs += 8;
    }

    if (vertexFlags & VF_NORMAL)
    {
      desc->push_back({ "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, ofs, D3D11_INPUT_PER_VERTEX_DATA, 0 });
      ofs += 12;
    }

    // check the ordering flag, to see if uvs or col should be added first
    if (vertexFlags & VF_ORDER_TEX_COL)
    {
      // uv before col
      if (vertexFlags & VF_TEX0)
      {
        desc->push_back({ "TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, ofs, D3D11_INPUT_PER_VERTEX_DATA, 0 });
        ofs += 8;
      }

      if (vertexFlags & VF_COLOR)
      {
        desc->push_back({ "COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, ofs, D3D11_INPUT_PER_VERTEX_DATA, 0 });
        ofs += 16;
      }

      if (vertexFlags & VF_COLOR_U32)
      {
        desc->push_back({ "COLOR", 0, DXGI_FORMAT_R8G8B8A8_UNORM, 0, ofs, D3D11_INPUT_PER_VERTEX_DATA, 0 });
        ofs += 4;
      }
    }
    else
    {
      // col before uv
      if (vertexFlags & VF_COLOR)
      {
        desc->push_back({ "COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, ofs, D3D11_INPUT_PER_VERTEX_DATA, 0 });
        ofs += 16;
      }

      if (vertexFlags & VF_COLOR_U32)
      {
        desc->push_back({ "COLOR", 0, DXGI_FORMAT_R8G8B8A8_UNORM, 0, ofs, D3D11_INPUT_PER_VERTEX_DATA, 0 });
        ofs += 4;
      }

      if (vertexFlags & VF_TEX0)
      {
        desc->push_back({ "TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, ofs, D3D11_INPUT_PER_VERTEX_DATA, 0 });
        ofs += 8;
      }
    }
  }
}


//------------------------------------------------------------------------------
bool SwapChain::CreateBackBuffers(u32 width, u32 height, DXGI_FORMAT format)
{
  _width = width;
  _height = height;

  Graphics& g = GRAPHICS;

  // Reset the buffers if the swap chain is already registered
  RenderTargetResource* rt;
  if (g._renderTargets.Get(_name, &rt))
  {
    rt->reset();
    _swapChain->ResizeBuffers(0, width, height, format, 0);
  }
  else
  {
    rt = new RenderTargetResource();
  }

  if (FAILED(_swapChain->GetBuffer(0, IID_PPV_ARGS(&rt->texture.resource))))
    return false;

  rt->texture.resource->GetDesc(&rt->texture.desc);

  // Create render target view
  D3D11_RENDER_TARGET_VIEW_DESC rtViewDesc;
  ZeroMemory(&rtViewDesc, sizeof(rtViewDesc));
  rtViewDesc.Format = rt->texture.desc.Format;
  rtViewDesc.ViewDimension = rt->texture.desc.SampleDesc.Count == 1 ? D3D11_RTV_DIMENSION_TEXTURE2D : D3D11_RTV_DIMENSION_TEXTURE2DMS;
  if (FAILED(g._device->CreateRenderTargetView(rt->texture.resource, &rtViewDesc, &rt->rtv.resource)))
    return false;

  rt->rtv.resource->GetDesc(&rt->rtv.desc);

  CD3D11_TEXTURE2D_DESC depthStencilDesc(
    DXGI_FORMAT_D24_UNORM_S8_UINT, width, height, 1, 1,
    D3D11_BIND_DEPTH_STENCIL, D3D11_USAGE_DEFAULT, 0, _desc.SampleDesc.Count);

  // Create depth stencil buffer and view
  if (FAILED(g._device->CreateTexture2D(&depthStencilDesc, NULL, &rt->depth_stencil.resource)))
    return false;

  rt->depth_stencil.resource->GetDesc(&rt->depth_stencil.desc);

  if (FAILED(g._device->CreateDepthStencilView(rt->depth_stencil.resource, NULL, &rt->dsv.resource)))
    return false;

  rt->dsv.resource->GetDesc(&rt->dsv.desc);

  // Register the buffer with GRAPHICS
  u32 idx = g._renderTargets.Insert(_name, rt);
  _renderTarget = ObjectHandle(ObjectHandle::kRenderTarget, idx);

  _viewport = CD3D11_VIEWPORT(0.0f, 0.0f, (float)width, (float)height);
  return true;
}

//------------------------------------------------------------------------------
void SwapChain::Present()
{
  _swapChain->Present(GRAPHICS._vsync ? 1 : 0, 0);
}

//------------------------------------------------------------------------------
ScopedRenderTarget::ScopedRenderTarget(int width, int height, DXGI_FORMAT format, const BufferFlags& bufferFlags)
{
  h = GRAPHICS.GetTempRenderTarget(width, height, format, bufferFlags);
}

//------------------------------------------------------------------------------
ScopedRenderTarget::ScopedRenderTarget(DXGI_FORMAT format, const BufferFlags& bufferFlags)
{
  h = GRAPHICS.GetTempRenderTarget(format, bufferFlags);
}

//------------------------------------------------------------------------------
ScopedRenderTarget::~ScopedRenderTarget()
{
  if (h.IsValid())
  {
    GRAPHICS.ReleaseTempRenderTarget(h);
  }
}
