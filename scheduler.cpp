#include "scheduler.hpp"
#include <thread>

using namespace tano;
using namespace tano::scheduler;
using namespace bristol;

//------------------------------------------------------------------------------
Scheduler* Scheduler::_instance;

#define SINGLE_THREADED 0

//------------------------------------------------------------------------------
bool Scheduler::Create()
{
  if (_instance)
    return true;

  _instance = new Scheduler();
  return _instance->Init();
}

//------------------------------------------------------------------------------
void Scheduler::Destroy()
{
  delete exch_null(_instance);
}

//------------------------------------------------------------------------------
Scheduler& Scheduler::Instance()
{
  return *_instance;
}

//------------------------------------------------------------------------------
Scheduler::Scheduler()
  : _taskAlloc(_taskMemory, _taskMemory + MAX_TASKS * sizeof(Task), sizeof(Task))
  , _taskQueue(_queueMemory, _queueMemory + MAX_TASKS * sizeof(Task*))
{
}

//------------------------------------------------------------------------------
Scheduler::~Scheduler()
{
  InterlockedExchange(&_done, TRUE);
  WakeAllConditionVariable(&_cvTask);

  for (thread& t : _threads)
    t.join();

  DeleteCriticalSection(&_csTask);
  DeleteCriticalSection(&_csAlloc);
}

//------------------------------------------------------------------------------
bool Scheduler::Init()
{
  InitializeCriticalSection(&_csAlloc);
  InitializeCriticalSection(&_csTask);
  InitializeConditionVariable(&_cvTask);

  // Create the worker theads
  _maxNumThreads = thread::hardware_concurrency();
  if(!_maxNumThreads)
  {
    LOG_ERROR("Unable to get # hardware threads");
    return false;
  }

#if SINGLE_THREADED
  _maxNumThreads = 0;
#endif

  for (int i = 0; i < _maxNumThreads-1; ++i)
  {
    _threads.push_back(thread(&Scheduler::WorkerThread, this));
  }

  return true;
}


//------------------------------------------------------------------------------
Task* Scheduler::GetAvailableTask()
{
  // Note, _csTask must be acquired before accessing the queue
  if (_taskQueue.IsEmpty())
    return nullptr;

  return _taskQueue.Pop();
}

//------------------------------------------------------------------------------
void Scheduler::QueueTask(Task* task)
{
  while (true)
  {
    {
      ScopedCriticalSection cs(&_csTask);
      if (!_taskQueue.IsFull())
      {
        _taskQueue.Push(task);
        WakeConditionVariable(&_cvTask);
        return;
      }
    }
    HelpWithWork();
  }
}

//------------------------------------------------------------------------------
void Scheduler::WorkerThread()
{
  bool lock = true;
  while (!InterlockedCompareExchange(&_done, TRUE, TRUE))
  {
    if (lock)
    {
      // Lock must be help when sleeping on the CV, so acquire it here
      EnterCriticalSection(&_csTask);
    }

    // Queue is locked, so look for tasks
    Task* task = GetAvailableTask();
    if (task)
    {
      LeaveCriticalSection(&_csTask);
      WorkOnTask(task);
      lock = true;
    }
    else
    {
      // No task available, so wait for CV to signal. This will release the CS
      SleepConditionVariableCS(&_cvTask, &_csTask, INFINITE);
      lock = false;
    }
  }

  LeaveCriticalSection(&_csTask);
}

//------------------------------------------------------------------------------
Task* Scheduler::AllocTask()
{
  void* memory;
  {
    ScopedCriticalSection cs(&_csAlloc);
    memory = _taskAlloc.Alloc();
  }

  Task* task = new (memory)Task();

  InterlockedIncrement(&_allocGeneration);
  task->generation = _allocGeneration;
  return task;
}

//------------------------------------------------------------------------------
void Scheduler::FreeTask(Task* task)
{
  InterlockedIncrement(&_allocGeneration);
  task->generation = _allocGeneration;
  {
    ScopedCriticalSection cs(&_csAlloc);
    _taskAlloc.Free(task);
  }
}

//------------------------------------------------------------------------------
bool Scheduler::IsTaskFinished(const TaskId& taskId)
{
  Task* task = GetTask(taskId);

  // If the generations don't match, then the task has been recycled, so it
  // must be finished
  if (task->generation != taskId.generation)
    return true;

  return task->openTasks == 0;
}

//------------------------------------------------------------------------------
void Scheduler::WorkOnTask(Task* task)
{
  while (!CanExecuteTask(task))
  {
    // task can't be worked on right now, so help with another task
    HelpWithWork();
  }

  // execute the kernel to perform the actual work
  task->kernel(task->taskData);
  
#if WITH_SCHEDULER_STATS
  {
    ScopedCriticalSection cs(&_csTask);
    _threadCount[GetCurrentThreadId()]++;
  }
#endif

  FinishTask(task);
}

//------------------------------------------------------------------------------
bool Scheduler::CanExecuteTask(Task* task)
{
  // A task can be executed if it doesn't have any open children
  return InterlockedCompareExchange(&task->openTasks, 1, 1) == 1;
}

//------------------------------------------------------------------------------
void Scheduler::HelpWithWork()
{
  Task* task;
  {
    ScopedCriticalSection cs(&_csTask);
    task = GetAvailableTask();
  }
  if (task)
  {
    WorkOnTask(task);
  }
  else
  {
    Sleep(0);
  }
}

//------------------------------------------------------------------------------
void Scheduler::FinishTask(Task* task)
{
  u32 openTasks = InterlockedDecrement(&task->openTasks);
  if (task->parent != Task::NO_PARENT)
  {
    Task* parent = OffsetToTask(task->parent);
    FinishTask(parent);
  }

  if (openTasks == 0)
  {
    FreeTask(task);
  }
}

//------------------------------------------------------------------------------
TaskId Scheduler::AddTask(const KernelData& kernelData, const Kernel& kernel)
{
  Task* task = AllocTask();

  task->kernel = kernel;
  task->taskData.kernelData = kernelData;
  task->openTasks = 1;

  QueueTask(task);

  return TaskId{ TaskToOffset(task), task->generation };
}

//------------------------------------------------------------------------------
TaskId Scheduler::AddStreamingTask(
  const KernelData& kernelData,
  const Kernel& kernel,
  int elementCount,
  const StreamData& inputStream0,
  const StreamData& outputStream0)
{
  Task* task = AllocTask();

  task->kernel = kernel;
  task->taskData.kernelData = kernelData;
  task->taskData.streamingData.elementCount = elementCount;
  task->taskData.streamingData.inputStreams[0] = inputStream0;
  task->taskData.streamingData.outputStreams[0] = outputStream0;

  QueueTask(task);

  return TaskId{ TaskToOffset(task), task->generation };
}

//------------------------------------------------------------------------------
void Scheduler::Wait(const TaskId& taskId)
{
  while (!IsTaskFinished(taskId))
  {
    HelpWithWork();
  }
}

//------------------------------------------------------------------------------
TaskOffset Scheduler::TaskToOffset(const Task* task)
{
  return (TaskOffset)(task - (Task*)_taskMemory);
}

//------------------------------------------------------------------------------
Task* Scheduler::OffsetToTask(TaskOffset offset)
{
  return (Task*)_taskMemory + offset;
}

//------------------------------------------------------------------------------
Task* Scheduler::GetTask(const TaskId& taskId)
{
  return OffsetToTask(taskId.offset);
}
