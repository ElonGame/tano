#pragma once
namespace test {

#define CHECKED_OP(x) do { if (!(x)) return false; } while(false);

struct InputBuffer
{
  InputBuffer();
  InputBuffer(const char* buf, size_t len);
  bool Peek(char* res);
  void Rewind(size_t len);
  bool Get(char* res);
  bool OneOf(const char* str, size_t len, char* res);
  bool Expect(char ch);
  bool SkipUntil(char ch, bool consume);
  bool SkipWhile(const function<bool(char)>& fn, size_t* end);
  bool Satifies(const function<bool(char)>& fn, char* ch);
  void SkipWhitespace();
  bool SubStr(size_t start, size_t len, string* res);

  static bool IsDigit(char ch);
  static bool IsAlphaNum(char ch);

  bool Consume();
  bool ConsumeIf(char ch, bool* res);
  bool Eof();

  const char* _buf;
  size_t _idx;
  size_t _len;
};

}
