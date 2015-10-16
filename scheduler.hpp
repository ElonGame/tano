// based on https://molecularmusings.wordpress.com/2012/04/05/building-a-load-balanced-task-scheduler-part-1-basics/

#pragma once
#include "free_list.hpp"
#include "circular_buffer.hpp"

#include "enkits/TaskScheduler.h"


namespace tano
{
  namespace scheduler
  {
    struct KernelData
    {
      void* data = nullptr;
      size_t size = 0;
    };

    struct StreamData
    {
      void* data = nullptr;
      size_t stride = 0;
    };

    struct TaskData
    {
      TaskData() { memset(this, 0, sizeof(TaskData)); }
      KernelData kernelData;
      struct StreamingData
      {
        int elementCount = 0;
        StreamData inputStreams[4];
        StreamData outputStreams[4];
      } streamingData;
    };

    typedef void(*Kernel)(const TaskData&);

    typedef u32 TaskOffset;

    struct Task
    {
      enum { NO_PARENT = 0xffffffff };
      char padding[sizeof(void*)];
      int generation = 0;
      u32 openTasks = 0;
      TaskOffset parent = NO_PARENT;
      TaskData taskData;
      Kernel kernel;
    };

    struct TaskId
    {
      TaskOffset offset;
      int generation;
    };

    class Scheduler
    {
    public:
      static bool Create();
      static void Destroy();
      static Scheduler& Instance();

      TaskId AddTask(const KernelData& kernelData, const Kernel& kernel);
      TaskId AddStreamingTask(
        const KernelData& kernelData,
        const Kernel& kernel,
        int elementCount,
        const StreamData& inputStream0,
        const StreamData& outputStream0);

      void Wait(const TaskId& taskId);

    private:
      Scheduler();
      ~Scheduler();
      bool Init();

      TaskOffset TaskToOffset(const Task* task);
      Task* OffsetToTask(TaskOffset offset);
      Task* GetTask(const TaskId& taskId);

      void WorkerThread();

      void QueueTask(Task* task);

      Task* AllocTask();
      void FreeTask(Task* task);

      bool IsTaskFinished(const TaskId& taskId);
      void WorkOnTask(Task* task);
      bool CanExecuteTask(Task* task);
      void HelpWithWork();
      void FinishTask(Task* task);

      vector<thread> _threads;
      int _maxNumThreads = 0;

      FreeList _taskAlloc;
      enum { MAX_TASKS = 16 * 1024 };
      char _taskMemory[MAX_TASKS * sizeof(Task)];
      char _queueMemory[MAX_TASKS * sizeof(Task*)];
      static Scheduler* _instance;
      CircularBuffer<Task*> _taskQueue;

      u32 _done = FALSE;
      u32 _allocGeneration = 0;

#if WITH_SCHEDULER_STATS
      unordered_map<u32, u32> _threadCount;
#endif

      CRITICAL_SECTION _csAlloc;
      CRITICAL_SECTION _csTask;
      CONDITION_VARIABLE _cvTask;
    };
  }

#define SCHEDULER Scheduler::Instance()

}
extern enki::TaskScheduler g_TS;
