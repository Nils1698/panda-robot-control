#pragma once

#include <Arduino.h>
#include "Streaming.h"
#include "UserStream.h"

#define DONE_SIGNAL "done" // TODO declare properly
#define ACK_SIGNAL "ack"   // TODO declare properly

// #define SIGNAL_DONE UserSerial<<"done\n";
// #define SIGNAL_ACK  UserSerial<<"ack\n";
// #define SIGNAL_DONE(s) UserSerial<<"done:"<<s<<"\n";

namespace UserCom {

static constexpr size_t MAX_BYTES = 100;
static char input_buffer[MAX_BYTES];
static char in_byte;
static int buf_i = 0;

// Deprecated
inline bool checkSerialInput(String &in_str) {
	while (UserSerial.available() > 0) {
		in_byte = UserSerial.read();
		input_buffer[buf_i++] = in_byte;
		if (in_byte == '\n')
		{
			input_buffer[buf_i] = '\0';
			in_str = String(input_buffer);
			in_str.replace("\r", "");
			in_str.replace("\n", "");
			buf_i = 0;
			return true;
		}
		else if (buf_i == MAX_BYTES - 1)
		{
			warning_stream << "Warning input buffer overflow!\n"; // debug
			buf_i = 0;
		}
	}
	return false;
}

// Deprecated
inline bool getInput(String in_str, String match, float &value)
{
	if (in_str.startsWith(match))
	{
		String val_str = in_str.substring(match.length());
		if (val_str == "o")
		{
			value = 0;
			return true;
		}
		else
		{
			value = val_str.toFloat();
			return value != 0;
		}
	}
	return false;
}

// Deprecated
inline bool getInputs(String in_str, String match, size_t exp_inputs, float *inputs)
{
	if (in_str.startsWith(match))
	{
		String val_str = in_str.substring(match.length());

		char *digit_end;
		const char *str = val_str.c_str();
		for (size_t i = 0; i < exp_inputs; i++)
		{
			inputs[i] = strtof(str, &digit_end);
			if (str == digit_end)
				return false;
			str = digit_end;
		}
		if (strlen(digit_end) > 0)
		{
			debug_stream << "Ignoring rest of input string: ...'" << digit_end << "'\n";
		}
		return true;
	}
	return false;
}

} // USerComm