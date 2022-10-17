#pragma once

#include <Arduino.h>
#include "Streaming.h"
#include "UserStream.h"
#include "Parser.h"

namespace UserInput{

class Listener {

static constexpr size_t MAX_BYTES = 100;

private:
	char input_buffer[MAX_BYTES];
	char in_byte;
	int buf_i = 0;
	String in_str;
	bool input_set = false, input_ready=false;
	
public:
	void update_reception();
	bool has_input();
	void set_input(String input_str);
	String get_input();

	String await_user_string();
	float await_user_float(const Parser::parse_arg<float> float_arg);
};

} // UserInput