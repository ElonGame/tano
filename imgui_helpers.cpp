#if WITH_IMGUI
#ifdef GetWindowFont
#undef GetWindowFont
#endif

#include "imgui/imgui.cpp"
#include "imgui/imgui_demo.cpp"
#include "imgui/imgui_draw.cpp"
#include "imgui_helpers.hpp"
#include "graphics.hpp"
#include "graphics_context.hpp"
#include "init_sequence.hpp"
#include "tano.hpp"

using namespace tano;
using namespace bristol;

namespace
{
  struct CUSTOMVERTEX
  {
    float pos[2];
    float uv[2];
    unsigned int col;
  };

  struct VERTEX_CONSTANT_BUFFER
  {
    float mvp[4][4];
  };

  LARGE_INTEGER ticksPerSecond;
  LARGE_INTEGER lastTime = {0};

  GraphicsContext* g_ctx;
  ConstantBuffer<VERTEX_CONSTANT_BUFFER> g_cb;

  GpuObjects g_gpuObjects;
  GpuState g_gpuState;
  ObjectHandle g_texture;
}

static int VB_SIZE = 1024 * 1024;
static int IB_SIZE = 1024 * 1024;

//------------------------------------------------------------------------------
bool InitDeviceD3D()
{
  BEGIN_INIT_SEQUENCE();

  // Setup rasterizer
  D3D11_RASTERIZER_DESC RSDesc;
  memset(&RSDesc, 0, sizeof(D3D11_RASTERIZER_DESC));
  RSDesc.FillMode = D3D11_FILL_SOLID;
  RSDesc.CullMode = D3D11_CULL_NONE;
  RSDesc.FrontCounterClockwise = FALSE;
  RSDesc.DepthBias = 0;
  RSDesc.SlopeScaledDepthBias = 0.0f;
  RSDesc.DepthBiasClamp = 0;
  RSDesc.DepthClipEnable = TRUE;
  RSDesc.ScissorEnable = TRUE;
  RSDesc.AntialiasedLineEnable = FALSE;

  // MAGNUS: check for multisampling
  // RSDesc.MultisampleEnable = (sd.SampleDesc.Count > 1) ? TRUE : FALSE;

  // Create the blending setup
  D3D11_BLEND_DESC desc;
  ZeroMemory(&desc, sizeof(desc));
  desc.AlphaToCoverageEnable = false;
  desc.RenderTarget[0].BlendEnable = true;
  desc.RenderTarget[0].SrcBlend = D3D11_BLEND_SRC_ALPHA;
  desc.RenderTarget[0].DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
  desc.RenderTarget[0].BlendOp = D3D11_BLEND_OP_ADD;
  desc.RenderTarget[0].SrcBlendAlpha = D3D11_BLEND_INV_SRC_ALPHA;
  desc.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_ZERO;
  desc.RenderTarget[0].BlendOpAlpha = D3D11_BLEND_OP_ADD;
  desc.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;

  CD3D11_DEPTH_STENCIL_DESC depthDesc = CD3D11_DEPTH_STENCIL_DESC(D3D11_DEFAULT);
  depthDesc.DepthEnable = FALSE;

  INIT(g_gpuState.Create(&depthDesc, &desc, &RSDesc));
  INIT(g_gpuObjects.CreateDynamicVb(VB_SIZE, sizeof(CUSTOMVERTEX)));
  INIT(g_gpuObjects.CreateDynamicIb(IB_SIZE, DXGI_FORMAT_R32_UINT));

// (MAGNUS) Ignoring the render target view for now.. maybe later i want the gui to have its
// own layer and compose with the main?
#if 0
    // Create the render target
    {
        ID3D11Texture2D* pBackBuffer;               
        D3D11_RENDER_TARGET_VIEW_DESC render_target_view_desc;
        ZeroMemory(&render_target_view_desc, sizeof(render_target_view_desc));
        render_target_view_desc.Format = sd.BufferDesc.Format;
        render_target_view_desc.ViewDimension = D3D11_RTV_DIMENSION_TEXTURE2D;

        g_pSwapChain->GetBuffer(0, __uuidof(ID3D11Texture2D), (LPVOID*)&pBackBuffer);
        g_pd3dDevice->CreateRenderTargetView(pBackBuffer, &render_target_view_desc, &g_mainRenderTargetView);
        g_pd3dDeviceImmediateContext->OMSetRenderTargets(1, &g_mainRenderTargetView, NULL);
        pBackBuffer->Release();
    }
#endif

  // Create shaders
  vector<D3D11_INPUT_ELEMENT_DESC> inputDesc = {
      CD3D11_INPUT_ELEMENT_DESC("POSITION", DXGI_FORMAT_R32G32_FLOAT),
      CD3D11_INPUT_ELEMENT_DESC("TEXCOORD", DXGI_FORMAT_R32G32_FLOAT),
      CD3D11_INPUT_ELEMENT_DESC("COLOR", DXGI_FORMAT_R8G8B8A8_UNORM),
  };
  INIT(g_gpuObjects.LoadVertexShader("shaders/out/imgui", "VsMain", 0, &inputDesc));
  INIT(g_gpuObjects.LoadPixelShader("shaders/out/imgui", "PsMain"));

  // Create the constant buffer
  INIT(g_cb.Create());

  END_INIT_SEQUENCE();
}

// This is the main rendering function that you have to implement and provide to ImGui (via setting
// up 'RenderDrawListsFn' in the ImGuiIO structure)
// If text or lines are blurry when integrating ImGui in your engine:
// - in your Render function, try translating your projection matrix by (0.5f,0.5f) or
// (0.375f,0.375f)
static void ImImpl_RenderDrawLists(ImDrawData* draw_data)
{

  if (draw_data->TotalVtxCount > VB_SIZE || draw_data->TotalIdxCount > IB_SIZE)
  {
    LOG_WARN("Too many verts or indices");
    return;
  }

  g_ctx->SetRenderTarget(g_Graphics->GetBackBuffer(), g_Graphics->GetDepthStencil(), nullptr);

  ImDrawVert* vtx_dst = g_ctx->MapWriteDiscard<ImDrawVert>(g_gpuObjects._vb);
  ImDrawIdx* idx_dst = g_ctx->MapWriteDiscard<ImDrawIdx>(g_gpuObjects._ib);
  for (int n = 0; n < draw_data->CmdListsCount; n++)
  {
    const ImDrawList* cmd_list = draw_data->CmdLists[n];
    memcpy(vtx_dst, &cmd_list->VtxBuffer[0], cmd_list->VtxBuffer.size() * sizeof(ImDrawVert));
    memcpy(idx_dst, &cmd_list->IdxBuffer[0], cmd_list->IdxBuffer.size() * sizeof(ImDrawIdx));
    vtx_dst += cmd_list->VtxBuffer.size();
    idx_dst += cmd_list->IdxBuffer.size();
  }

  g_ctx->Unmap(g_gpuObjects._vb);
  g_ctx->Unmap(g_gpuObjects._ib);

  // Setup orthographic projection matrix into our constant buffer
  {
    const float L = 0.0f;
    const float R = ImGui::GetIO().DisplaySize.x;
    const float B = ImGui::GetIO().DisplaySize.y;
    const float T = 0.0f;
    const float mvp[4][4] = {
        {2.0f / (R - L), 0.0f, 0.0f, 0.0f},
        {
            0.0f, 2.0f / (T - B), 0.0f, 0.0f,
        },
        {0.0f, 0.0f, 0.5f, 0.0f},
        {(R + L) / (L - R), (T + B) / (B - T), 0.5f, 1.0f},
    };
    memcpy(&g_cb.mvp, mvp, sizeof(mvp));
  }

  // Setup viewport
  {
    D3D11_VIEWPORT vp;
    memset(&vp, 0, sizeof(D3D11_VIEWPORT));
    vp.Width = ImGui::GetIO().DisplaySize.x;
    vp.Height = ImGui::GetIO().DisplaySize.y;
    vp.MinDepth = 0.0f;
    vp.MaxDepth = 1.0f;
    vp.TopLeftX = 0;
    vp.TopLeftY = 0;
    g_ctx->SetViewports(1, vp);
  }

  // Bind shader and vertex buffers
  unsigned int stride = sizeof(CUSTOMVERTEX);
  unsigned int offset = 0;

  g_ctx->SetGpuObjects(g_gpuObjects);
  g_ctx->SetGpuState(g_gpuState);
  g_ctx->SetGpuStateSamplers(g_gpuState, ShaderType::PixelShader);
  g_ctx->SetConstantBuffer(g_cb, ShaderType::VertexShader, 0);

  // Render command list
  int vtx_offset = 0;
  int idx_offset = 0;
  for (int n = 0; n < draw_data->CmdListsCount; n++)
  {
    const ImDrawList* cmd_list = draw_data->CmdLists[n];
    for (int cmd_i = 0; cmd_i < cmd_list->CmdBuffer.size(); cmd_i++)
    {
      const ImDrawCmd* pcmd = &cmd_list->CmdBuffer[cmd_i];
      if (pcmd->UserCallback)
      {
        pcmd->UserCallback(cmd_list, pcmd);
      }
      else
      {
        const D3D11_RECT r = {(LONG)pcmd->ClipRect.x,
            (LONG)pcmd->ClipRect.y,
            (LONG)pcmd->ClipRect.z,
            (LONG)pcmd->ClipRect.w};
        ObjectHandle* h = (ObjectHandle*)pcmd->TextureId;
        if (h && h->IsValid())
          g_ctx->SetShaderResource(*h, ShaderType::PixelShader);
        else
          g_ctx->SetShaderResource(g_texture, ShaderType::PixelShader);

        g_ctx->SetScissorRect(1, &r);
        g_ctx->DrawIndexed(pcmd->ElemCount, idx_offset, vtx_offset);
      }
      idx_offset += pcmd->ElemCount;
    }
    vtx_offset += cmd_list->VtxBuffer.size();
  }

  // reset to full screen scissor rect
  SwapChain* swapChain = g_Graphics->GetSwapChain(g_Graphics->DefaultSwapChain());
  CD3D11_RECT rect = CD3D11_RECT((LONG)swapChain->_viewport.TopLeftX,
      (LONG)swapChain->_viewport.TopLeftY,
      (LONG)(swapChain->_viewport.TopLeftX + swapChain->_viewport.Width),
      (LONG)(swapChain->_viewport.TopLeftY + swapChain->_viewport.Height));
  g_ctx->SetScissorRect(1, &rect);
}

//------------------------------------------------------------------------------
void LoadFontsTexture()
{
  // Load one or more font
  ImGuiIO& io = ImGui::GetIO();
  // ImFont* my_font1 = io.Fonts->AddFontDefault();
  // ImFont* my_font2 = io.Fonts->AddFontFromFileTTF("extra_fonts/Karla-Regular.ttf", 15.0f);
  ImFont* my_font3 = io.Fonts->AddFontFromFileTTF("imgui/extra_fonts/ProggyClean.ttf", 13.0f);
  my_font3->DisplayOffset.y += 1;
  // ImFont* my_font4 = io.Fonts->AddFontFromFileTTF("imgui/extra_fonts/ProggyTiny.ttf", 10.0f);
  // my_font4->DisplayOffset.y += 1;
  // ImFont* my_font5 = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 20.0f,
  // io.Fonts->GetGlyphRangesJapanese());

  // Build
  unsigned char* pixels;
  int width, height;
  io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
  g_texture =
      g_Graphics->CreateTexture(width, height, DXGI_FORMAT_R8G8B8A8_UNORM, pixels, width * 4);

  // Store our identifier
  // io.Fonts->TexID = (void *)*(u32*)&h;
}

namespace tano
{
  //------------------------------------------------------------------------------
  LRESULT WINAPI ImGuiWndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
  {
    ImGuiIO& io = ImGui::GetIO();
    switch (msg)
    {
      case WM_LBUTTONDOWN: io.MouseDown[0] = true; return true;
      case WM_LBUTTONUP: io.MouseDown[0] = false; return true;
      case WM_RBUTTONDOWN: io.MouseDown[1] = true; return true;
      case WM_RBUTTONUP: io.MouseDown[1] = false; return true;
      case WM_MOUSEWHEEL:
        io.MouseWheel += GET_WHEEL_DELTA_WPARAM(wParam) > 0 ? +1.0f : -1.0f;
        return true;
      case WM_MOUSEMOVE:
        // Mouse position, in pixels (set to -1,-1 if no mouse / on another screen, etc.)
        io.MousePos.x = (float)((signed short)(lParam) / BACKBUFFER_SCALE);
        io.MousePos.y = (float)((signed short)(lParam >> 16) / BACKBUFFER_SCALE);
        return true;
      case WM_CHAR:
        // You can also use ToAscii()+GetKeyboardState() to retrieve characters.
        if (wParam > 0 && wParam < 0x10000)
          io.AddInputCharacter((unsigned short)wParam);
        return true;
      case WM_KEYDOWN:
        io.KeysDown[wParam & 0xff] = true;
        io.KeyCtrl = wParam == VK_CONTROL;
        io.KeyShift = wParam == VK_SHIFT;
        return true;
      case WM_KEYUP:
        io.KeysDown[wParam & 0xff] = false;
        g_KeyUpTrigger.SetTrigger(wParam & 0xff);
        return true;
    }

    return false;
  }

  //------------------------------------------------------------------------------
  void UpdateImGui()
  {
    ImGuiIO& io = ImGui::GetIO();

    // Setup time step
    LARGE_INTEGER currentTime;
    QueryPerformanceCounter(&currentTime);
    io.DeltaTime = (float)(currentTime.QuadPart - lastTime.QuadPart) / ticksPerSecond.QuadPart;
    lastTime = currentTime;

    // Setup inputs
    // (we already got mouse position, buttons, wheel from the window message callback)
    //     BYTE keystate[256];
    //     GetKeyboardState(keystate);
    //     for (int i = 0; i < 256; i++)
    //       io.KeysDown[i] = (keystate[i] & 0x80) != 0;
    //     io.KeyCtrl = (keystate[VK_CONTROL] & 0x80) != 0;
    //     io.KeyShift = (keystate[VK_SHIFT] & 0x80) != 0;
    // io.MousePos : filled by WM_MOUSEMOVE event
    // io.MouseDown : filled by WM_*BUTTON* events
    // io.MouseWheel : filled by WM_MOUSEWHEEL events

    // Start the frame
    ImGui::NewFrame();
  }

  //------------------------------------------------------------------------------
  bool InitImGui(HWND hWnd)
  {
    g_ctx = g_Graphics->GetGraphicsContext();
    InitDeviceD3D();

    int display_w, display_h;
    g_Graphics->GetBackBufferSize(&display_w, &display_h);

    ImGuiIO& io = ImGui::GetIO();
    io.DisplaySize = ImVec2((float)display_w,
        (float)display_h);       // Display size, in pixels. For clamping windows positions.
    io.DeltaTime = 1.0f / 60.0f; // Time elapsed since last frame, in seconds (in this sample app
                                 // we'll override this every frame because our time step is
                                 // variable)
    io.KeyMap[ImGuiKey_Tab] = VK_TAB; // Keyboard mapping. ImGui will use those indices to peek into
                                      // the io.KeyDown[] array that we will update during the
                                      // application lifetime.
    io.KeyMap[ImGuiKey_LeftArrow] = VK_LEFT;
    io.KeyMap[ImGuiKey_RightArrow] = VK_RIGHT;
    io.KeyMap[ImGuiKey_UpArrow] = VK_UP;
    io.KeyMap[ImGuiKey_DownArrow] = VK_UP;
    io.KeyMap[ImGuiKey_Home] = VK_HOME;
    io.KeyMap[ImGuiKey_End] = VK_END;
    io.KeyMap[ImGuiKey_Delete] = VK_DELETE;
    io.KeyMap[ImGuiKey_Backspace] = VK_BACK;
    io.KeyMap[ImGuiKey_Enter] = VK_RETURN;
    io.KeyMap[ImGuiKey_Escape] = VK_ESCAPE;
    io.KeyMap[ImGuiKey_A] = 'A';
    io.KeyMap[ImGuiKey_C] = 'C';
    io.KeyMap[ImGuiKey_V] = 'V';
    io.KeyMap[ImGuiKey_X] = 'X';
    io.KeyMap[ImGuiKey_Y] = 'Y';
    io.KeyMap[ImGuiKey_Z] = 'Z';
    io.RenderDrawListsFn = ImImpl_RenderDrawLists;
    io.ImeWindowHandle = hWnd;

    QueryPerformanceFrequency(&ticksPerSecond);

    // Load fonts
    LoadFontsTexture();

    return true;
  }
}

//------------------------------------------------------------------------------
namespace ImGui
{
  void PlotExMulti(const char* label,
      float (*values_getter)(int graph_idx, void* data, int idx),
      void* data,
      int values_count,
      int values_offset,
      int graph_count,
      const char* overlay_text,
      float scale_min,
      float scale_max,
      ImVec2 graph_size)
  {
    ImGuiWindow* window = GetCurrentWindow();
    if (window->SkipItems)
      return;

    ImGuiState& g = *GImGui;
    const ImGuiStyle& style = g.Style;

    const ImVec2 label_size = ImGui::CalcTextSize(label, NULL, true);
    if (graph_size.x == 0.0f)
      graph_size.x = CalcItemWidth() + (style.FramePadding.x * 2);
    if (graph_size.y == 0.0f)
      graph_size.y = label_size.y + (style.FramePadding.y * 2);

    const ImRect frame_bb(
        window->DC.CursorPos, window->DC.CursorPos + ImVec2(graph_size.x, graph_size.y));
    const ImRect inner_bb(frame_bb.Min + style.FramePadding, frame_bb.Max - style.FramePadding);
    const ImRect total_bb(frame_bb.Min,
        frame_bb.Max
            + ImVec2(label_size.x > 0.0f ? style.ItemInnerSpacing.x + label_size.x : 0.0f, 0));
    ItemSize(total_bb, style.FramePadding.y);
    if (!ItemAdd(total_bb, NULL))
      return;

    // TODO(magnus): should values_offset be used for the scale?
    // Determine scale from values if not specified
    if (scale_min == FLT_MAX || scale_max == FLT_MAX)
    {
      float v_min = FLT_MAX;
      float v_max = -FLT_MAX;
      for (int i = 0; i < graph_count; ++i)
      {
        for (int j = 0; j < values_count; j++)
        {
          const float v = values_getter(i, data, j);
          v_min = ImMin(v_min, v);
          v_max = ImMax(v_max, v);
        }
        if (scale_min == FLT_MAX)
          scale_min = v_min;
        if (scale_max == FLT_MAX)
          scale_max = v_max;
      }
    }
    float scale_span = scale_max - scale_min;

    RenderFrame(
        frame_bb.Min, frame_bb.Max, window->Color(ImGuiCol_FrameBg), true, style.FrameRounding);

    int res_w = ImMin((int)graph_size.x, values_count) - 1;
    int item_count = values_count - 1;

    // todo(magnus): tooltip is broken
#if 0
    // Tooltip on hover
    int v_hovered = -1;
    if (IsHovered(inner_bb, 0))
    {
      const float t = ImClamp(
          (g.IO.MousePos.x - inner_bb.Min.x) / (inner_bb.Max.x - inner_bb.Min.x), 0.0f, 0.9999f);
      const int v_idx = (int)(t * item_count);
      IM_ASSERT(v_idx >= 0 && v_idx < values_count);

      const float v0 = values_getter(data, (v_idx + values_offset) % values_count);
      const float v1 = values_getter(data, (v_idx + 1 + values_offset) % values_count);
      ImGui::SetTooltip("%d: %8.4g\n%d: %8.4g", v_idx, v0, v_idx + 1, v1);
      v_hovered = v_idx;
    }
#endif

    const float t_step = 1.0f / (float)res_w;

    ImVec4 graph_colors[] = {ImVec4(1, 0, 0, 1),
        ImVec4(0, 1, 0, 1),
        ImVec4(0, 0, 1, 1),
        ImVec4(1, 1, 0, 1),
        ImVec4(1, 0, 1, 1)};

    for (int ii = 0; ii < graph_count; ++ii)
    {
      // get starting value
      float v0 = values_getter(ii, data, (0 + values_offset) % values_count);
      float t0 = 0.0f;
      // Point in the normalized space of our target rectangle
      ImVec2 tp0 = ImVec2(t0, 1.0f - ImSaturate((v0 - scale_min) / scale_span));

      //const ImU32 col_base = window->Color(ImGuiCol_PlotLines);
      const ImU32 col_base = window->Color(graph_colors[ii % IM_ARRAYSIZE(graph_colors)]);
      const ImU32 col_hovered = window->Color(ImGuiCol_PlotLinesHovered);

      // figure out where 0,0 is in the bounding box
      float originY = -scale_min / (scale_max - scale_min);
      window->DrawList->AddLine(
        ImVec2(inner_bb.Min.x, inner_bb.Max.y - originY * inner_bb.GetHeight()),
        ImVec2(inner_bb.Max.x, inner_bb.Max.y - originY * inner_bb.GetHeight()),
        col_base);

      for (int n = 0; n < res_w; n++)
      {
        const float t1 = t0 + t_step;
        const int v1_idx = (int)(t0 * item_count + 0.5f);
        IM_ASSERT(v1_idx >= 0 && v1_idx < values_count);
        const float v1 = values_getter(ii, data, (v1_idx + values_offset + 1) % values_count);
        const ImVec2 tp1 = ImVec2(t1, 1.0f - ImSaturate((v1 - scale_min) / scale_span));

        // NB: Draw calls are merged together by the DrawList system. Still, we should render our
        // batch
        // are lower level to save a bit of CPU.
        ImVec2 pos0 = ImLerp(inner_bb.Min, inner_bb.Max, tp0);
        ImVec2 pos1 = ImLerp(inner_bb.Min, inner_bb.Max, tp1);
        window->DrawList->AddLine(pos0, pos1, col_base);

        t0 = t1;
        tp0 = tp1;
      }
    }

    // Text overlay
    if (overlay_text)
      RenderTextClipped(ImVec2(frame_bb.Min.x, frame_bb.Min.y + style.FramePadding.y),
          frame_bb.Max,
          overlay_text,
          NULL,
          NULL,
          ImGuiAlign_Center);

    RenderText(ImVec2(frame_bb.Max.x + style.ItemInnerSpacing.x, inner_bb.Min.y), label);
  }

  //------------------------------------------------------------------------------
  struct ImGuiPlotMultiArrayGetterData
  {
    const float** Values;
    int Stride;

    ImGuiPlotMultiArrayGetterData(const float** values, int stride) { Values = values; Stride = stride; }
  };


  static float Plot_MultiArrayGetter(int data_idx, void* data, int idx)
  {
    ImGuiPlotMultiArrayGetterData* plot_data = (ImGuiPlotMultiArrayGetterData*)data;
    const float* dd = plot_data->Values[data_idx];
    const float v = *(float*)(void*)((unsigned char*)dd + (size_t)idx * plot_data->Stride);
    return v;
  }

  void PlotLinesMulti(const char* label,
      PlotValues* values,
      int values_count,
      int values_offset,
      int graph_count,
      const char* overlay_text,
      float scale_min,
      float scale_max,
      ImVec2 graph_size,
      int stride)
  {
    ImGuiPlotMultiArrayGetterData data(values, stride);
    PlotExMulti(label,
        &Plot_MultiArrayGetter,
        (void*)&data,
        values_count,
        values_offset,
        graph_count,
        overlay_text,
        scale_min,
        scale_max,
        graph_size);
  }
}

#endif