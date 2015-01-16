#pragma once

namespace test {

struct InputBuffer;
struct OutputBuffer;
struct AppConfig;

bool ParseAppConfig(InputBuffer& buf, AppConfig* res);

// Functions are created for both the case when the struct is a top-level struct,
// and for when it's a member of another struct
void Serialize(OutputBuffer& buf, const AppConfig& msg);
void Serialize(OutputBuffer& buf, int indent, const char* member, const AppConfig& msg);

}
