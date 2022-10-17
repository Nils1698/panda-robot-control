#pragma once

#include "RS485_HW.h"

#define USB_Port Serial
#define RS485_Port Serial_RS485

#ifndef UserSerial
  #define UserSerial USB_Port
#endif
#ifndef BAUD_RATE
    #define BAUD_RATE 500000
#endif

namespace user_serial {

    inline void begin(bool await_connection=false){
        UserSerial.begin(BAUD_RATE);
        while(await_connection && !UserSerial){delay(1);}  
    }

} // user_serial