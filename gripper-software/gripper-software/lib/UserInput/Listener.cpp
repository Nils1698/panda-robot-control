
#include "Listener.h"

namespace UserInput{

void Listener::update_reception(){
	input_ready = false;

	if(input_set){
		input_set = false;
		input_ready = true;
	}

	while (UserSerial.available() > 0)
	{
		in_byte = UserSerial.read();
		input_buffer[buf_i++] = in_byte;
		if (in_byte == '\n')
		{
			input_buffer[buf_i] = '\0';
			in_str = String(input_buffer);
			in_str.replace("\r", "");
			in_str.replace("\n", "");
			buf_i = 0;

			input_ready=true;
		}
		else if (buf_i == MAX_BYTES - 1)
		{
			warning_stream << "Warning input buffer overflow!\n"; // debug
			buf_i = 0;
		}
	}
}

bool Listener::has_input(){
	return input_ready;
}

void Listener::set_input(String input_str){
	in_str = input_str;
	input_set = true;
}

String Listener::get_input(){
	return in_str;
}

String Listener::await_user_string(){
	while(true){
		update_reception();
		if(has_input()) return get_input();
		delay(100);
	}
}

float Listener::await_user_float(const Parser::parse_arg<float> float_arg){
	while(true){
		update_reception();
		if(has_input()){
			Parser p(get_input());
			float res = p.parse<float>(float_arg);
			if(p.verify_input()){
				return res;
			}
		}
		delay(100);
	}
}

} // USerComm