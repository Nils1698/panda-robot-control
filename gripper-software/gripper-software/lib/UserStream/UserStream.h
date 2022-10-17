#pragma once

#include "Streaming.h"
#include "serial_definitions.h"

class UserStream : public Print
{
private:
    const String STREAM_TYPE;
    bool stream_start = true;
    bool prepend_streamtype = true;
public:
    bool active = true;

public:
    UserStream(String type_str, bool use_prefix=true);

    inline size_t write(uint8_t b) override {
        if(!active) return 0;
        if(stream_start){
            if(prepend_streamtype) UserSerial << STREAM_TYPE << ": ";
            stream_start=false;
        }
        stream_start = (b=='\n');
        return UserSerial.write(b);
    }
    inline void setActive(bool b){active=b;}
};

inline UserStream::UserStream(String type_str, bool use_prefix) : STREAM_TYPE(type_str){
    prepend_streamtype = use_prefix;
}

extern UserStream debug_stream;
extern UserStream debug_com_stream;
extern UserStream error_stream;
extern UserStream info_stream;
extern UserStream warning_stream;

extern UserStream signal_stream;