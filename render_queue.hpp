// A start to something based on
// https://molecularmusings.wordpress.com/2014/12/16/stateless-layered-multi-threaded-rendering-part-3-api-design-details/

struct CommandBase
{
  u8* data = nullptr;
  CommandBase* next = nullptr;
};

struct SetSwapChain : CommandBase
{
  SetSwapChain(ObjectHandle h, const Color& color) : handle(h), clearColor(color) {}
  ObjectHandle handle;
  Color clearColor;
};

struct SetConstantBuffer : CommandBase
{
  ObjectHandle handle;
  ShaderType shader;
};

struct RenderQueue
{
  static const u32 INITIAL_MEMORY_SIZE = 128 * 1024 * 1024;

  RenderQueue() : _memory(INITIAL_MEMORY_SIZE) {}

  void FrameStart();

  template <typename T, typename... Args>
  T* AllocateCommand(u32 dataSize, Args&&... args)
  {
    T* cmd = new (AllocChunk(sizeof(T)))T(forward<Args>(args)...);
    _memoryOfs += sizeof(T);

    if (dataSize)
      cmd->data = AllocChunk(dataSize);

    return cmd;
  }

  template <typename T, typename... Args>
  T* AppendCommand(CommandBase* parent, u32 dataSize, Args&&... args)
  {
    T* cmd = new (AllocChunk(sizeof(T)))T(forward<Args>(args)...);
    _memoryOfs += sizeof(T);

    if (parent)
      parent->next = cmd;

    if (dataSize)
      cmd->data = AllocChunk(dataSize);

    return cmd;
  }

  u8* AllocChunk(u32 size)
  {
    u8* tmp = &_memory[_memoryOfs];
    _memoryOfs += size;
    return tmp;
  }

  vector<u8> _memory;
  u32 _memoryOfs = 0;
};

void RenderQueue::FrameStart()
{
  // reset the render queue
  _memoryOfs = 0;
}

SetSwapChain* setSwapChain = renderQueue.AllocateCommand<SetSwapChain>(0, GRAPHICS.DefaultSwapChain(), Color(0, 0, 0, 0));
SetConstantBuffer* setConstantBuffer = renderQueue.AppendCommand<SetConstantBuffer>(setSwapChain, 0);
