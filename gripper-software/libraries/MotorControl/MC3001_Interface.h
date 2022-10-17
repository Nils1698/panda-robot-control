
#pragma once 

#include "Arduino.h"
#include "Streaming.h"

#define mot_serial Serial1
#define polynomial 0xD5

#define DEBUG_COMM 0

namespace MC3001 {

  constexpr uint8_t NODE_ID = 1;
  constexpr uint8_t START_BYTE = 'S'; // 0x53
  constexpr uint8_t END_BYTE = 'E'; // 0x45
  
  constexpr uint8_t MSG_LEN_IDX = 0;
  constexpr uint8_t MSG_NODE_IDX = 1;
  constexpr uint8_t MSG_CMD_IDX = 2;
  constexpr uint8_t MSG_INDEX_IDX = 3;
  constexpr uint8_t MSG_SUBINDEX_IDX = 5;
  constexpr uint8_t MSG_DATA_IDX = 6;

  constexpr uint8_t SDO_MSG_LEN = 7;

  constexpr long DEFAULT_BAUD = 115200;

  typedef uint16_t controlword_t;
  typedef uint16_t statusword_t;

  constexpr statusword_t MASK_OP_STATE = (1<<10)|(1<<12)|(1<<13);

  // Drive functions, Tab. 60 (p. 134):
  constexpr statusword_t MASK_HOMING_STOPPED = 1<<10;
  constexpr statusword_t MASK_HOMING_REACHED = 1<<12;
  constexpr statusword_t MASK_HOMING_ERROR   = 1<<13;

  constexpr statusword_t MASK_PP_TARGET_REACHED  = 1<<10;
  constexpr statusword_t MASK_PP_TARGET_ACK      = 1<<12;
  constexpr statusword_t MASK_PP_FOLLOWING_ERROR = 1<<13;

  // Drive functions, Tab. 3, p. 21
  constexpr controlword_t MASK_CONTROL_SWITCH_ON    = 1<<0;
  constexpr controlword_t MASK_CONTROL_ENABLE_VOLT  = 1<<1;
  constexpr controlword_t MASK_CONTROL_QUICKSTOP    = 1<<2;
  constexpr controlword_t MASK_CONTROL_ENABLE_OP    = 1<<3;
  constexpr controlword_t MASK_CONTROL_HALT         = 1<<8;

  enum controller_state {
    NotReadyToSwitchOn = 0b0000000,
    SwitchOnDisabled   = 0b1000000,
    ReadyToSwitchOn    = 0b0100001,
    SwitchedOn         = 0b0100011,
    OperationEnabled   = 0b0100111,
    QuickStopActive    = 0b0000111,
    Fault              = 0b0001000
    //Halt,
    //FaultReactionActive
  };
  constexpr statusword_t STATE_MASK1 = 0b1011111;
  constexpr statusword_t STATE_MASK2 = 0b1001111;

  // Modes of Operation (Drive Function, p. 102)
  enum controller_mode : int8_t {
    Inactive = 0,

    ProfilePosition = 1,
    AnalogPositionControl = -2,
    CSPosition = 8,

    ProfileVelocity = 3,
    AnalogVelocityControl = -3,
    CSVelocity = 9,

    CSTorque = 10,

    Homing = 6
  };
  enum controller_command : uint8_t { // Communication Manual, Table 2 (page 16)
    Boot_Up     = 0x00,
    SDO_Read    = 0x01,
    SDO_Write   = 0x02,
    SDO_Error   = 0x03,
    Controlword = 0x04,
    Statusword  = 0x05,
    Trace_Log   = 0x06,
    EMCY        = 0x07,
    CMD_UNSET   = 0xFF
    // ...
  };
  enum save_configuration_options : uint8_t {
    SAVE_ALL_PARAMS = 0x01,
    SAVE_COM_PARAMS = 0x02, // 0x0000 to 0x1FFF (ComMan page 8)
    SAVE_APP_PARAMS = 0x03, // 0x2000 to 0x6FFF
    SAVE_TO_APP1 = 0x04,
    SAVE_TO_APP2 = 0x05
  };
  enum load_configuration_options : uint8_t {
    RESET_ALL_PARAMS = 0x01,
    RESET_COM_PARAMS = 0x02, // 0x0000 to 0x1FFF (ComMan page 8)
    RESET_APP_PARAMS = 0x03, // 0x2000 to 0x6FFF
    LOAD_APP_PARAMS = 0x04,
    LOAD_FROM_APP1 = 0x05,
    LOAD_FROM_APP2 = 0x06
  };
  
  int read_msg(uint8_t *in_buffer, const int max_length, const unsigned long timeout_ms);
  void send_msg(uint8_t *msg, const int len);
  uint8_t CalcCRCByte(uint8_t u8Byte, uint8_t u8CRC);
  uint8_t CRC8(uint8_t *data, uint8_t len);
  String decode_state(statusword_t statusword);
  String decode_op_status(uint16_t statusword, uint8_t mode);
  String decode_fault_register(uint16_t fault_register);
  String decode_SDO_error(uint8_t err_class, uint8_t err_code, uint16_t err_add_code);
  void print_msg(uint8_t *msg, int len);

  void write_controlword(controlword_t ctrlw);
  
  void read_SDO(const uint16_t index, const uint8_t sub_index);

  // inline static bool check_message(uint8_t *msg, const uint16_t index, const uint8_t sub_index){
  //   return (msg[3] == lowByte(index) && msg[4] == highByte(index) && msg[5] == sub_index);
  // }

  // TODO delte method
  // DEBUG
  inline static bool check_message(uint8_t *msg, const uint16_t index, const uint8_t sub_index){
    bool success = true;
    if(msg[3] != lowByte(index) || msg[4] != highByte(index)){
      Serial << "Wrong index, " << msg[3] << " != " << lowByte(index) << " || " << msg[4] << " != " << highByte(index) <<"\n";
      success = false;
    }
    if(msg[5] != sub_index){
      Serial << "Wrong subindex, " << msg[5] << " != " << sub_index <<"\n";
      success = false;
    }
    return success;
  }



inline void write_SDO(const uint16_t index, const uint8_t sub_index, const void* value_ptr, const size_t value_size){
  const uint8_t N = 6+value_size;
  uint8_t msg[N];
  
  msg[0] = N+1;
  msg[1] = NODE_ID;
  msg[2] = SDO_Write;
  msg[3] = lowByte(index);
  msg[4] = highByte(index);
  msg[5] = sub_index;
  memcpy(&msg[6], value_ptr, value_size);
  send_msg(msg, N);

#if DEBUG_COMM
  debug_stream << "Read 0x"<<_HEX(index)<<"."<<_HEX(sub_index)<<" Send: ";
  print_msg(msg, N);
#endif
}


  // template<typename T>
  // void write_SDO(const uint16_t index, const uint8_t sub_index, const T value){
  //   constexpr uint8_t N = 6+sizeof(T);
  //   uint8_t msg[N];
      
  //   msg[0] = N+1;
  //   msg[1] = NODE_ID;
  //   msg[2] = SDO_Write;
  //   msg[3] = lowByte(index);
  //   msg[4] = highByte(index);
  //   msg[5] = sub_index;
  //   memcpy(&msg[6], &value, sizeof(T));
  //   send_msg(msg, N);
  
  // #if DEBUG_COMM
  //   // DEBUG
  //   Serial.print("Write 0x");
  //   Serial.print(index,HEX);
  //   Serial.print(".");
  //   Serial.print(sub_index,HEX);
  //   Serial.print(": ");
  //   Serial.print(value);
  //   Serial.print("   Send: ");
  //   print_msg(msg, N); // DEBUG
  // #endif
  // }


} // MC3001
