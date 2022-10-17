#pragma once

#include <Arduino.h>
#include "Streaming.h"
#include "UserStream.h"

namespace UserInput{

class Parser {

public:
	enum parser_error_t {
		NO_ERROR = 0,
		PARSING_ERROR,
		LIMIT_ERROR,
		MISSING_ARG_ERROR,
		UNEXP_ARG_ERROR,
		PARSE_TYPE_ERROR
	};

private:
	const String in_str;
	const char * cstr;
	const char DELIMITER;
 
	parser_error_t input_err;

public:
    Parser(String str, char seperator=' ') : in_str(str), DELIMITER(seperator){
        cstr = in_str.c_str();
        input_err = NO_ERROR;
    };

	template <typename T>
	struct parse_arg {
		bool use_limits = false;
		T lim_min, lim_max;
		bool has_default = false;
		T default_val;

		parse_arg(){}
		parse_arg(T lmin, T lmax) : use_limits(true), lim_min(lmin), lim_max(lmax) {}
		parse_arg(T def) : has_default(true), default_val(def) {}
		parse_arg(T lmin, T lmax, T def) : use_limits(true), lim_min(lmin), lim_max(lmax), has_default(true), default_val(def) {}
	};
	
    bool match(String cmd);
    String rest_string();
	template <typename T>
	T parse(const parse_arg<T> parg = parse_arg<T>());
	String parse_string(String default_val="");
	bool verify_input(bool print_if_error=true);

	parser_error_t parsing_error(){return input_err;}

private:
	void end_parsing();
	void print_error();

};


} // USerComm