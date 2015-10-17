#pragma once
#include "object_handle.hpp"
#include "graphics.hpp"
#include "gpu_objects.hpp"

namespace tano
{
  class GraphicsContext
  {
    friend class Graphics;
  public:

    void SetSwapChain(ObjectHandle h, const Color& clearColor);
    void SetSwapChain(ObjectHandle h, const float* clearColor);
    void SetRenderTarget(ObjectHandle renderTarget, const Color* clearTarget);
    void SetRenderTarget(ObjectHandle renderTarget, ObjectHandle depthStencil, const Color* clearTarget);
    void SetRenderTargets(
      const ObjectHandle* renderTargets,
      int numRenderTargets,
      ObjectHandle depthStencil,
      const Color** clearTarget);
    void GenerateMips(ObjectHandle h);

    void SetVertexBuffer(ID3D11Buffer *buf, uint32_t stride);
    void SetIndexBuffer(ID3D11Buffer *buf, DXGI_FORMAT format);
    void SetVertexShader(ObjectHandle vs);
    void SetPixelShader(ObjectHandle ps);
    void SetComputeShader(ObjectHandle cs);
    void SetGeometryShader(ObjectHandle cs);
    void SetLayout(ObjectHandle layout);
    void SetVertexBuffer(ObjectHandle vb);
    void SetIndexBuffer(ObjectHandle ib);
    void SetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY top);
    void SetRasterizerState(ObjectHandle rs);
    void SetDepthStencilState(ObjectHandle dss, UINT stencil_ref);
    void SetBlendState(ObjectHandle bs, const float *blendFactors, UINT sampleMask);
    void SetShaderResource(ObjectHandle h, ShaderType shaderType = ShaderType::PixelShader, int slot = 0);
    void SetShaderResources(const vector<ObjectHandle>& handles, ShaderType shaderType);
    void SetShaderResources(const ObjectHandle* handles, int numHandles, ShaderType shaderType);
    void SetUnorderedAccessView(ObjectHandle h, Color* clearColor);
    void SetSamplerState(ObjectHandle h, ShaderType shaderType = ShaderType::PixelShader, u32 slot = 0);
    void SetSamplers(const ObjectHandle* h, u32 slot, u32 numSamplers, ShaderType shaderType);
    template <typename T>
    void SetConstantBuffer(const ConstantBuffer<T>& buffer, u32 shaderFlags, u32 slot)
    {
        SetConstantBuffer(buffer.handle, &buffer, sizeof(T), shaderFlags, slot);
    }

    void SetConstantBuffer(ObjectHandle h, const void* buf, size_t len, u32 shaderFlags, u32 slot);
    void SetBundle(const GpuBundle& bundle);
    void SetBundleWithSamplers(const GpuBundle& bundle, ShaderType shaderType);
    void SetGpuObjects(const GpuObjects& obj);
    void SetGpuState(const GpuState& state);
    void SetGpuStateSamplers(const GpuState& state, ShaderType shaderType);
    void SetGpuStateAndSamplers(const GpuState& state, ShaderType shaderType);
    void SetViewports(u32 numViewports, const D3D11_VIEWPORT& viewport);
    void SetScissorRect(u32 numRects, const D3D11_RECT* rects);

    void UnsetShaderResources(u32 first, u32 count, ShaderType shaderType);
    void UnsetUnorderedAccessViews(int first, int count);
    void UnsetRenderTargets(int first, int count);
    void DrawIndexed(int indexCount, int startIndex, int baseVertex);
    void Draw(int vertexCount, int startVertexLocation);
    void Dispatch(int threadGroupCountX, int threadGroupCountY, int threadGroupCountZ);

    template <typename T>
    T* MapWriteDiscard(ObjectHandle h, int* pitch = nullptr);
    HRESULT Map(ObjectHandle h, UINT sub, D3D11_MAP type, UINT flags, D3D11_MAPPED_SUBRESOURCE *res);
    void Unmap(ObjectHandle h, UINT sub = 0);
    void CopyToBuffer(ObjectHandle h, UINT sub, D3D11_MAP type, UINT flags, const void* data, u32 len);
    void CopyToBuffer(ObjectHandle h, const void* data, u32 len);

    void Flush();

  private:
    GraphicsContext(ID3D11DeviceContext* ctx);
    ID3D11DeviceContext *_ctx;

    u32 _default_stencil_ref;
    float _default_blend_factors[4];
    u32 _default_sample_mask;

  };

  template <typename T>
  T* GraphicsContext::MapWriteDiscard(ObjectHandle h, int* pitch)
  {
    D3D11_MAPPED_SUBRESOURCE res;
    if (!SUCCEEDED(Map(h, 0, D3D11_MAP_WRITE_DISCARD, 0, &res)))
      return nullptr;

    if (pitch)
      *pitch = res.RowPitch;

    return (T*)res.pData;
  }

}
