
#include "Parser.h"

namespace UserInput{

bool Parser::match(const String cmd){
  if(String(cstr).startsWith(cmd)){
	// cstr = in_str.c_str() + cmd.length();
	// cstr += cmd.length();
	const char * p = cstr + cmd.length();
	if(*p==' ' || *p=='\0'){
		while(*p==' '){p++;} // skip leading white spaces
		cstr=p;
		return true;
	}
  }
  return false;
}

String Parser::rest_string(){
	return String(cstr);
}

String Parser::parse_string(String default_val){
	if(input_err!=NO_ERROR) return 0;

	if (cstr[0] == '\0'){
		if (default_val!=""){
			return default_val;
		}else{
			input_err = MISSING_ARG_ERROR;
			return 0;
		}
	}
	size_t i,ii;
	for (i = 0; cstr[i]==' '; i++){} // Skip leading white spaces
	for (ii = i; cstr[ii]!=DELIMITER && cstr[ii]!='\0'; ii++){} // read till next seperator or end of string
	String res = String(cstr).substring(i,ii);
	cstr += ii;
	return res;
}

template <typename T>
T Parser::parse(const parse_arg<T> parg){
	char *digit_end;
	T v;

	if(input_err!=NO_ERROR) return 0;

	if (cstr[0] == '\0'){
		if (parg.has_default){
			return parg.default_val;
		}else{
			// debug_stream << "Too few arguments\n";
			input_err = MISSING_ARG_ERROR;
			return 0;
		}
	}

	// NOTE: static-if only valid in C++17
	if 		/*constexpr*/ (std::is_same<T, float>::value)    v = strtof(cstr, &digit_end);
	else if /*constexpr*/ (std::is_same<T, int>::value) 	 v = strtol(cstr, &digit_end, 10);
	else{
		input_err = PARSE_TYPE_ERROR;
		return 0;
	};

	if (cstr == digit_end || !(*digit_end == DELIMITER || *digit_end == '\0')){
		// debug_stream << "Parsing argument failed '" << String(cstr) << "'\n";
		input_err = PARSING_ERROR;
		return 0;
	}

	if (parg.use_limits && (v < parg.lim_min || v > parg.lim_max)){
		// debug_stream << "Argument out of limits ("<<v<<") ["<< parg.lim_min << " ; " << parg.lim_max <<"]\n";
		input_err = LIMIT_ERROR;
		return 0;
	}

	if(*digit_end == DELIMITER) digit_end++; // skip trailing delimiter
	cstr = digit_end;
	return v;
}
template int Parser::parse<int>(const parse_arg<int> parg);
template float Parser::parse<float>(const parse_arg<float> parg);

bool Parser::verify_input(bool print_if_error){
	end_parsing();
	if(input_err == NO_ERROR){
		return true;
	} 
	else if(print_if_error){
		print_error();
	}
	return false;
}

void Parser::end_parsing(){	
	if(input_err!=NO_ERROR) return;

	// Check if any arguments left to parse
	for (const char * p = cstr; *p != '\0'; p++)
	{
		if( !(*p==' ' || *p=='\n' || *p=='\r') ){
			input_err = UNEXP_ARG_ERROR;
			return;
		}		
	}	
}

void Parser::print_error(){
	Print &out = debug_stream;
	switch(input_err)
	{
	case NO_ERROR:
		out << "Success\n";
		break;

	case PARSING_ERROR:
		out << "Parsing argument failed";
		break;

	case MISSING_ARG_ERROR:
		out << "Too few arguments";
		out << "\n";
		return;

	case UNEXP_ARG_ERROR:
		out << "Too many arguments";
		break;

	case PARSE_TYPE_ERROR:
		out << "Parsing type not implemented";
		break;

	case LIMIT_ERROR:
		out << "Argument out of limits";
		break;
	}
	out << " ->"<< String(cstr) << "\n";
}

} // USerComm