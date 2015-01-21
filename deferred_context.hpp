#pragma once
#include "object_handle.hpp"
#include "graphics.hpp"
#include "gpu_objects.hpp"

namespace tano
{
  class DeferredContext
  {
    friend class Graphics;
  public:

    void SetSwapChain(ObjectHandle h, const Color& clearColor);
    void SetSwapChain(ObjectHandle h, const float* clearColor);
    void SetRenderTarget(ObjectHandle render_target, const Color* clearTarget);
    void SetRenderTargets(ObjectHandle *render_targets, const Color* clearTarget, int num_render_targets);
    void GenerateMips(ObjectHandle h);

    void SetVB(ID3D11Buffer *buf, uint32_t stride);
    void SetIB(ID3D11Buffer *buf, DXGI_FORMAT format);
    void SetVS(ObjectHandle vs);
    void SetPS(ObjectHandle ps);
    void SetCS(ObjectHandle cs);
    void SetGS(ObjectHandle cs);
    void SetLayout(ObjectHandle layout);
    void SetVB(ObjectHandle vb);
    void SetIB(ObjectHandle ib);
    void SetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY top);
    void SetRasterizerState(ObjectHandle rs);
    void SetDepthStencilState(ObjectHandle dss, UINT stencil_ref);
    void SetBlendState(ObjectHandle bs, const float *blendFactors, UINT sampleMask);
    void SetShaderResource(ObjectHandle h, ShaderType shaderType);
    void SetShaderResources(const vector<ObjectHandle>& handles, ShaderType shaderType);
    void SetUAV(ObjectHandle h, Color* clearColor);
    void SetSamplerState(ObjectHandle h, u32 slot, ShaderType shaderType);
    void SetSamplers(const ObjectHandle* h, u32 slot, u32 numSamplers, ShaderType shaderType);
    template <typename T>
    void SetConstantBuffer(const ConstantBuffer<T>& buffer, ShaderType shaderType, u32 slot)
    {
      return SetConstantBuffer(buffer.handle, &buffer, sizeof(T), shaderType, slot);
    }

    void SetConstantBuffer(ObjectHandle h, const void* buf, size_t len, ShaderType shaderType, u32 slot);
    void SetGpuObjects(const GpuObjects& obj);
    void SetGpuState(const GpuState& state);
    void SetGpuStateSamplers(const GpuState& state, ShaderType shaderType);
    void SetViewports(u32 numViewports, const D3D11_VIEWPORT& viewport);
    void SetScissorRect(u32 numRects, const D3D11_RECT* rects);

    void UnsetSRVs(u32 first, u32 count, ShaderType shaderType);
    void UnsetUAVs(int first, int count);
    void UnsetRenderTargets(int first, int count);
    void DrawIndexed(int count, int start_index, int base_vertex);
    void Draw(int vertexCount, int startVertexLocation);
    void Dispatch(int threadGroupCountX, int threadGroupCountY, int threadGroupCountZ);

    template <typename T>
    T* MapWriteDiscard(ObjectHandle h);
    bool Map(ObjectHandle h, UINT sub, D3D11_MAP type, UINT flags, D3D11_MAPPED_SUBRESOURCE *res);
    void Unmap(ObjectHandle h, UINT sub = 0);
    void CopyToBuffer(ObjectHandle h, UINT sub, D3D11_MAP type, UINT flags, const void* data, u32 len);
    void CopyToBuffer(ObjectHandle h, const void* data, u32 len);

    void Flush();

  private:
    DeferredContext();
    ID3D11DeviceContext *_ctx;

    u32 _default_stencil_ref;
    float _default_blend_factors[4];
    u32 _default_sample_mask;

  };

  template <typename T>
  T* DeferredContext::MapWriteDiscard(ObjectHandle h)
  {
    D3D11_MAPPED_SUBRESOURCE res;
    if (!Map(h, 0, D3D11_MAP_WRITE_DISCARD, 0, &res))
      return nullptr;

    return (T*)res.pData;
  }

}
