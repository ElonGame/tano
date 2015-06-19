#if WITH_IMGUI
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
    float        pos[2];
    float        uv[2];
    unsigned int col;
  };

  struct VERTEX_CONSTANT_BUFFER
  {
    float mvp[4][4];
  };

  INT64 ticks_per_second = 0;
  INT64 last_time = 0;

  GraphicsContext* g_ctx;
  ConstantBuffer<VERTEX_CONSTANT_BUFFER> g_cb;

  GpuObjects g_gpuObjects;
  GpuState g_gpuState;
  ObjectHandle g_texture;
}


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
  //RSDesc.MultisampleEnable = (sd.SampleDesc.Count > 1) ? TRUE : FALSE;

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
  INIT(g_gpuObjects.CreateDynamicVb(200000, sizeof(CUSTOMVERTEX)));

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


// This is the main rendering function that you have to implement and provide to ImGui (via setting up 'RenderDrawListsFn' in the ImGuiIO structure)
// If text or lines are blurry when integrating ImGui in your engine:
// - in your Render function, try translating your projection matrix by (0.5f,0.5f) or (0.375f,0.375f)
static void ImImpl_RenderDrawLists(ImDrawList** const cmd_lists, int cmd_lists_count)
{
  g_ctx->SetRenderTarget(GRAPHICS.GetBackBuffer(), GRAPHICS.GetDepthStencil(), nullptr);

  size_t total_vtx_count = 0;
  for (int n = 0; n < cmd_lists_count; n++)
    total_vtx_count += cmd_lists[n]->vtx_buffer.size();
  if (total_vtx_count == 0)
    return;

  // Copy and convert all vertices into a single contiguous buffer
  D3D11_MAPPED_SUBRESOURCE mappedResource;
  if (!g_ctx->Map(g_gpuObjects._vb, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource))
    return;

  CUSTOMVERTEX* vtx_dst = (CUSTOMVERTEX*)mappedResource.pData;
  for (int n = 0; n < cmd_lists_count; n++)
  {
    const ImDrawList* cmd_list = cmd_lists[n];
    const ImDrawVert* vtx_src = &cmd_list->vtx_buffer[0];
    for (size_t i = 0; i < cmd_list->vtx_buffer.size(); i++)
    {
      vtx_dst->pos[0] = vtx_src->pos.x;
      vtx_dst->pos[1] = vtx_src->pos.y;
      vtx_dst->uv[0] = vtx_src->uv.x;
      vtx_dst->uv[1] = vtx_src->uv.y;
      vtx_dst->col = vtx_src->col;
      vtx_dst++;
      vtx_src++;
    }
  }
  g_ctx->Unmap(g_gpuObjects._vb);

  // Setup orthographic projection matrix into our constant buffer
  {
    const float L = 0.0f;
    const float R = ImGui::GetIO().DisplaySize.x;
    const float B = ImGui::GetIO().DisplaySize.y;
    const float T = 0.0f;
    const float mvp[4][4] =
    {
      { 2.0f/(R-L), 0.0f, 0.0f, 0.0f },
      { 0.0f, 2.0f/(T-B), 0.0f, 0.0f, },
      { 0.0f, 0.0f, 0.5f, 0.0f },
      { (R+L)/(L-R), (T+B)/(B-T), 0.5f, 1.0f },
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

  // Render command lists
  int vtx_offset = 0;
  for (int n = 0; n < cmd_lists_count; n++)
  {
    // Render command list
    const ImDrawList* cmd_list = cmd_lists[n];
    for (size_t cmd_i = 0; cmd_i < cmd_list->commands.size(); cmd_i++)
    {
      const ImDrawCmd* pcmd = &cmd_list->commands[cmd_i];
      const D3D11_RECT r ={ (LONG)pcmd->clip_rect.x, (LONG)pcmd->clip_rect.y, (LONG)pcmd->clip_rect.z, (LONG)pcmd->clip_rect.w };

      //ObjectHandle h = *(ObjectHandle*)&pcmd->texture_id;
      g_ctx->SetShaderResource(g_texture, ShaderType::PixelShader);
      g_ctx->SetScissorRect(1, &r);
      g_ctx->Draw(pcmd->vtx_count, vtx_offset);
      vtx_offset += pcmd->vtx_count;
    }
  }

  // reset to full screen scissor rect
  SwapChain* swapChain = GRAPHICS.GetSwapChain(GRAPHICS.DefaultSwapChain());
  CD3D11_RECT rect = CD3D11_RECT(
    (LONG)swapChain->_viewport.TopLeftX,
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
  //ImFont* my_font1 = io.Fonts->AddFontDefault();
  //ImFont* my_font2 = io.Fonts->AddFontFromFileTTF("extra_fonts/Karla-Regular.ttf", 15.0f);
  ImFont* my_font3 = io.Fonts->AddFontFromFileTTF("imgui/extra_fonts/ProggyClean.ttf", 13.0f); my_font3->DisplayOffset.y += 1;
  //ImFont* my_font4 = io.Fonts->AddFontFromFileTTF("imgui/extra_fonts/ProggyTiny.ttf", 10.0f); my_font4->DisplayOffset.y += 1;
  //ImFont* my_font5 = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 20.0f, io.Fonts->GetGlyphRangesJapanese());

  // Build
  unsigned char* pixels;
  int width, height;
  io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
  g_texture = GRAPHICS.CreateTexture(width, height, DXGI_FORMAT_R8G8B8A8_UNORM, pixels, width*4);

  // Store our identifier
  //io.Fonts->TexID = (void *)*(u32*)&h;
}

namespace tano
{
  //------------------------------------------------------------------------------
  LRESULT WINAPI ImGuiWndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
  {
    ImGuiIO& io = ImGui::GetIO();
    switch (msg)
    {
      case WM_LBUTTONDOWN:
        io.MouseDown[0] = true;
        return true;
      case WM_LBUTTONUP:
        io.MouseDown[0] = false;
        return true;
      case WM_RBUTTONDOWN:
        io.MouseDown[1] = true;
        return true;
      case WM_RBUTTONUP:
        io.MouseDown[1] = false;
        return true;
      case WM_MOUSEWHEEL:
        io.MouseWheel += GET_WHEEL_DELTA_WPARAM(wParam) > 0 ? +1.0f : -1.0f;
        return true;
      case WM_MOUSEMOVE:
        // Mouse position, in pixels (set to -1,-1 if no mouse / on another screen, etc.)
        io.MousePos.x = (signed short)(lParam);
        io.MousePos.y = (signed short)(lParam >> 16);
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
    INT64 current_time;
    QueryPerformanceCounter((LARGE_INTEGER *)&current_time);
    io.DeltaTime = (float)(current_time - last_time) / ticks_per_second;
    last_time = current_time;

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
  bool InitImGui()
  {
    g_ctx = GRAPHICS.GetGraphicsContext();
    InitDeviceD3D();

    int display_w, display_h;
    GRAPHICS.GetBackBufferSize(&display_w, &display_h);

    ImGuiIO& io = ImGui::GetIO();
    io.DisplaySize = ImVec2((float)display_w, (float)display_h);    // Display size, in pixels. For clamping windows positions.
    io.DeltaTime = 1.0f/60.0f;                                      // Time elapsed since last frame, in seconds (in this sample app we'll override this every frame because our time step is variable)
    io.KeyMap[ImGuiKey_Tab] = VK_TAB;                               // Keyboard mapping. ImGui will use those indices to peek into the io.KeyDown[] array that we will update during the application lifetime.
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

    // Load fonts
    LoadFontsTexture();

    return true;
  }
}
#endif