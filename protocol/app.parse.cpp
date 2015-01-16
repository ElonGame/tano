#include "parse_base.hpp"
#include "input_buffer.hpp"
#include "app.parse.hpp"
#include "app.types.hpp"

namespace test {

bool ParseAppConfig(InputBuffer& buf, AppConfig* res)
{
  CHECKED_OP(buf.Expect('{'));

  string id;
  while (true)
  {
    buf.SkipWhitespace();
    // check for a closing tag
    bool end;
    CHECKED_OP(buf.ConsumeIf('}', &end));
    if (end)
      break;

    CHECKED_OP(ParseIdentifier(buf, &id));
    buf.SkipWhitespace();

    if (id == "soundtrack")
    {
      CHECKED_OP(ParseString(buf, &res->soundtrack));
      buf.SkipWhitespace();
      CHECKED_OP(buf.Expect(';'));
    }
  }
  return true;
}


void Serialize(OutputBuffer& buf, const AppConfig& msg)
{
  buf.EnsureCapacity(32);
  buf.Advance(sprintf(buf.Cur(), "{\n"));

  Serialize(buf, 2, "soundtrack", msg.soundtrack);

  buf.Advance(sprintf(buf.Cur(), "};\n"));
}

void Serialize(OutputBuffer& buf, int indent, const char* member, const AppConfig& msg)
{
  int len = strlen(member);
  buf.EnsureCapacity(len + indent + 16);
  buf.Advance(sprintf(buf.Cur(), "%*s: {\n", len + indent, member));

  Serialize(buf, indent + 2, "soundtrack", msg.soundtrack);

  buf.Advance(sprintf(buf.Cur(), "%*s", 3 + indent, "};\n"));
}



}
