/*
  - LogSinks are added to a global vector when created
  - When writing log messages, a LogStream object is created by the LOG_XXX macros,
    and these then accept LogObject<T> objects via the << operator
  - When the LogStream goes out of scope, all the LogObjects are sent to the LogSinks
*/

#include "error.hpp"
#include <time.h>

using namespace std;

//-----------------------------------------------------------------------------
namespace bristol
{
  std::vector<LogSink*> g_logSinks;
  LogLevel g_logLevel = LogLevelNone;

  fnLogCallback g_logCallback;
  bool g_breakOnError = true;


  //-----------------------------------------------------------------------------
  void SetBreakOnError(bool value)
  {
    g_breakOnError = value;
  }

  //-----------------------------------------------------------------------------
  void SetLogCallback(const fnLogCallback& cb)
  {
    g_logCallback = cb;
  }

  //-----------------------------------------------------------------------------
  LogLevel GetLogLevel()
  {
    return g_logLevel;
  }

  //-----------------------------------------------------------------------------
  void SetLogLevel(LogLevel level)
  {
    g_logLevel = level;
  }
}

using namespace bristol;

//-----------------------------------------------------------------------------
LogSink::LogSink()
{
  g_logSinks.push_back(this);
}

//-----------------------------------------------------------------------------
LogSink::~LogSink()
{
  auto it = find(g_logSinks.begin(), g_logSinks.end(), this);
  if (it != g_logSinks.end())
    g_logSinks.erase(it);
}

//-----------------------------------------------------------------------------
LogSinkFile::LogSinkFile()
    : _file(nullptr)
{
}

//-----------------------------------------------------------------------------
LogSinkFile::~LogSinkFile()
{
  if (_file)
    fclose(_file);
}

//-----------------------------------------------------------------------------
bool LogSinkFile::Open(const char* filename)
{
  _file = fopen(filename, "at");
  return !!_file;
}

//-----------------------------------------------------------------------------
void LogSinkFile::Log(const LogEntry& entry)
{
  if (!_file)
    return;

  // create log prefix, with severity and time stamp
  static char levelPrefix[] = { '-', 'D', 'I', 'W', 'E' };

  time_t current_time;
  struct tm * time_info;
  char timeString[9];  // space for "HH:MM:SS\0"

  time(&current_time);
  time_info = localtime(&current_time);

  strftime(timeString, sizeof(timeString), "%H:%M:%S", time_info);
  fprintf(_file, "[%c] %s - %s", levelPrefix[(int)entry.level], timeString, entry.msg);
  fflush(_file);
}

//-----------------------------------------------------------------------------
void LogSinkConsole::Log(const LogEntry& entry)
{
  if (!(entry.flags & LogFlagsNaked))
  {
    // create clickable console prefix
    char buf[1024];
    sprintf(buf, "%s(%d): ", entry.file, entry.line);
    OutputDebugStringA(buf);
  }

  OutputDebugStringA(entry.msg);
  OutputDebugStringA("\n");
}

//-----------------------------------------------------------------------------
void LogSinkApp::Log(const LogEntry& entry)
{
  // NOTE: this is probably a bad way to propagate logs. A better way would just
  // be to create a custom LogSink
  g_logCallback(entry);
}

//-----------------------------------------------------------------------------
LogStream::LogStream(LogLevel level, const char* file, uint32_t line, uint32_t flags)
  : _level(level)
  , _file(file)
  , _line(line)
  , _flags(flags)
{
}

//-----------------------------------------------------------------------------
LogStream::~LogStream()
{
  if (_level < g_logLevel)
    return;

  LogEntry entry { _level, _flags, _file, _line, _curMessage.c_str() };

  for (LogSink* sink : g_logSinks)
    sink->Log(entry);

  if (_level == LogLevelError && g_breakOnError)
  {
    DebugBreak();
  }

}

//-----------------------------------------------------------------------------
LogLevel GetLogLevel();
