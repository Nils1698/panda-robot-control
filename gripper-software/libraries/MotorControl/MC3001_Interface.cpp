
#include "MC3001_Interface.h"
#include "UserStream.h"

namespace MC3001 {

// Method copied from Communication Manual, page 16
uint8_t CalcCRCByte(uint8_t u8Byte, uint8_t u8CRC){
    uint8_t i;
    u8CRC = u8CRC ^ u8Byte;
    for (i = 0; i < 8; i++) {
        if (u8CRC & 0x01) {
            u8CRC = (u8CRC >> 1) ^ polynomial;
        }else {
            u8CRC >>= 1;
        }
    }
    return u8CRC;
}

uint8_t CRC8(uint8_t *data, uint8_t len){
    uint8_t u8CRC = 0xFF;
    for (int i = 0; i < len; i++)
        u8CRC = CalcCRCByte(*(data+i), u8CRC);
    return u8CRC;
}

// TODO delete method
int read_msg(uint8_t *in_buffer, const int max_length, const unsigned long timeout_ms){
  unsigned long timeStart = millis();
  int bytes_read = -1;
  int exp_len=-1;
  uint8_t input;
  uint8_t u8CRC = 0xFF;
  
  while(millis()-timeStart < timeout_ms){

    if(mot_serial.available() > 0){
      input = mot_serial.read();
      
      if(bytes_read==-1){
        if(input == START_BYTE) bytes_read=0;
      }
      else if(bytes_read==0){
        exp_len = input;
        if(exp_len < 0 || exp_len > max_length) return -2;
        *(in_buffer++) = input;
        u8CRC = CalcCRCByte(input, u8CRC);
        bytes_read++;
      }
      else
      {
        if(bytes_read == exp_len){
          if(input==END_BYTE) break;
          else return -3;
        }else{
          *(in_buffer++) = input;
          u8CRC = CalcCRCByte(input, u8CRC);
          bytes_read++;          
        }
      }
    }
  }
  
  if(u8CRC !=0){
    warning_stream << "read_msg() - CRC error\n";
    return -4;
  }
  
  return bytes_read;
}

void send_msg(uint8_t *msg, const int len){
  uint8_t crc = CRC8(msg, len);
  mot_serial.write(START_BYTE);
  mot_serial.write(msg, len);
  mot_serial.write(crc);
  mot_serial.write(END_BYTE);
}

void write_controlword(controlword_t ctrlw){
  constexpr uint8_t N = 5;
  uint8_t msg[N]; // CRC appended later
  
  msg[0] = N+1; // (+CRC)
  msg[1] = NODE_ID;
  msg[2] = Controlword;
  msg[3] = lowByte(ctrlw);
  msg[4] = highByte(ctrlw);
  send_msg(msg, N);  
}

String decode_state(statusword_t statusword){
  String res = "";
       if((statusword & 0b01001111) == 0b00000000) res += "Not ready to switch on";
  else if((statusword & 0b01001111) == 0b01000000) res += "Switch on disabled";
  else if((statusword & 0b01101111) == 0b00100001) res += "Ready to switch on";
  else if((statusword & 0b01101111) == 0b00100011) res += "Switched on";
  else if((statusword & 0b01101111) == 0b00100111) res += "Operation enabled";
  else if((statusword & 0b01101111) == 0b00000111) res += "Quickstop active";
  else if((statusword & 0b01101111) == 0b00001111) res += "Fault reaction active";
  else if((statusword & 0b01101111) == 0b00001000) res += "Fault";
  else res = "Unknown";
  return res;
}

String decode_fault_register(uint16_t fault_register){
  switch(fault_register){
    case 0x0000: return "No Error";
    case 0x0001: return "SpeedDeviationError";
    case 0x0002: return "FollowingError";
    case 0x0004: return "OverVoltageError";
    case 0x0008: return "UnderVoltageError";
    case 0x0010: return "TempWarning";
    case 0x0020: return "TempError";
    case 0x0040: return "EncoderError";
    case 0x0080: return "IntHW error";
    case 0x0100: return "ModuleError";
    case 0x0200: return "CurrentMeasError";
    case 0x0400: return "Memory error";
    case 0x0800: return "ComError";
    case 0x1000: return "Calculation error";
    case 0x2000: return "DynamicError";
  }
  return "Unknown";
}

// Com Manual, Tab 24, p.26
String decode_SDO_error(uint8_t err_class, uint8_t err_code, uint16_t err_add_code){
  uint32_t combined_code = err_class<<24 | err_code<<16 | err_add_code;
  switch (combined_code)
  {
    case 0x05040001: return "SDO command invalid or unknown";
    case 0x06010000: return "Access to this object is not supported";
    case 0x06010001: return "Attempt to read a write-only parameter";
    case 0x06010002: return "Attempt to write to a read-only parameter";
    case 0x06020000: return "Object not present in the object dictionary";
    case 0x06040043: return "General parameter incompatibility";
    case 0x06040047: return "General internal incompatibility error in the device";
    case 0x06070010: return "Data type or parameter length do not match or are unknown";
    case 0x06070012: return "Data types do not match, parameter length too long";
    case 0x06070013: return "Data types do not match, parameter length too short";
    case 0x06090030: return "General value range error";
    case 0x06090031: return "Value range error: Parameter value too large";
    case 0x06090032: return "Value range error: Parameter value too small";
    case 0x06090036: return "Value range error: Maximum value greater than minimum value";
    case 0x08000000: return "General SDO error";
    case 0x08000020: return "Cannot be accessed";
    case 0x08000022: return "Cannot be accessed at current device status";
    default:         return "Unknown error code";
  }
}

String decode_op_status(uint16_t statusword, uint8_t mode){
  String res = "";
  switch(mode){
    case ProfilePosition:
      if((statusword & 1<<10) > 0) res += "Target reached ";
      if((statusword & 1<<12) > 0) res += "New set-point ";
      if((statusword & 1<<13) > 0) res += "Follow error! ";    
      break;

    case ProfileVelocity:
      if((statusword & 1<<10) > 0) res += "Velocity reached ";
      if((statusword & 1<<12) > 0) res += "standstill";
      if((statusword & 1<<13) > 0) res += "Slippage error! ";    
      break;

    case Homing: {
      if((statusword & MASK_HOMING_STOPPED) > 0) res += "Speed=0 ";
      if((statusword & MASK_HOMING_REACHED) > 0) res += "Homing completed ";
      if((statusword & MASK_HOMING_ERROR) > 0) res += "Homing error! ";
      uint8_t state = (statusword & 1<<10)>>10 | (statusword & 1<<12)>>11 | (statusword & 1<<13)>>12;
      switch(state){
        case 0b000: res += "- Homing procedure active"; break;
        case 0b001: res += "- Homing procedure interrupted or not started"; break;
        case 0b010 : res += "- Homing procedure has been completed, the speed is not yet 0"; break;
        case 0b011 : res += "- Homing procedure has been successfully completed"; break;
        case 0b100 : res += "- A homing error has occurred, speed is not 0"; break;
        case 0b101 : res += "- A homing error has occurred, speed is 0"; break;
      }    
        
    } break;
    
  }

  return res;
}

// DEBUG
void print_msg(uint8_t *msg, int len){
  if(len<0) return;
  for(int i=0; i<len; ++i){
    debug_com_stream << _HEX(*(msg++)) << " ";
  }
  debug_com_stream<<"\n";
}

void read_SDO(const uint16_t index, const uint8_t sub_index)
{  
  constexpr uint8_t N = 6;
  uint8_t msg[N];
  
  msg[0] = N+1; // (+ CRC)
  msg[1] = NODE_ID;
  msg[2] = SDO_Read;
  msg[3] = lowByte(index);
  msg[4] = highByte(index);
  msg[5] = sub_index;
  
  send_msg(msg, N);
#if DEBUG_COMM
  debug_com_stream << "Read 0x"<<_HEX(index)<<"."<<_HEX(sub_index)<<" Send: ";
  print_msg(msg, N);
#endif
}


} // MC3001
