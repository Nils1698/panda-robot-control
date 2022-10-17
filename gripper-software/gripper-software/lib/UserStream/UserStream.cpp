#include "UserStream.h"

UserStream debug_stream = UserStream("debug");
UserStream debug_com_stream = UserStream("DEBUG COM");
UserStream error_stream = UserStream("ERROR");
UserStream info_stream = UserStream("INFO", false);
UserStream warning_stream = UserStream("WARNING");

UserStream  signal_stream = UserStream("SIGNAL", false);