#include "MotorCom.h"

#include "UserStream.h"

namespace motor_interface
{

MotorCom::MotorCom(){}

void MotorCom::init()
{
	mot_serial.begin(MC3001::DEFAULT_BAUD, SERIAL_8N1);
	thread_id = threads.addThread(MotorCom::receiver_thread, this);
	threads.delay(500);

	bool ok = false;
	for (int i = 0; i < 5; ++i) {
		if(sync_state()) {
			print_status(debug_com_stream);
			debug_com_stream << "Controlword: " << _BIN(controlword) << "\n";
			ok = true;
			break;
		}
		threads.delay(100);
	}

	if(!ok){
		error_stream << "No response from motor controller\n";
		connection_status = CONNECTION_DOWN;
	}else{
		connection_status = CONNECTION_OK;
	}
}

void MotorCom::receiver_thread(void *arg)
{
	uint8_t input_buffer[MotorCom::BUFFER_SIZE];
	MotorCom *motor = static_cast<MotorCom*>(arg);

	debug_com_stream << "Receiver thread started\n";
	while (1)
	{
		unsigned long timeStart = millis();
		int bytes_read = -1;
		int exp_len=-1;
		uint8_t input;
		uint8_t u8CRC = 0xFF;
		bool timed_out = false;
		
		while(millis()-timeStart < motor->incoming_timeout_ms){

			if(mot_serial.available() > 0){
				timed_out=true;
				input = mot_serial.read();				
				if(bytes_read==-1){ // First byte
					if(input == MC3001::START_BYTE){bytes_read=0;}
				}
				else
				{
					if(bytes_read == exp_len){ // Last byte
						if(input==MC3001::END_BYTE){
							motor->handle_incoming(input_buffer+1, bytes_read);
						}else{
							debug_com_stream << "Incoming - Uncexpected end byte\n";
						}
						timed_out=false;
						break;
					}else{
						if(bytes_read==0){ // Second byte (length)
							exp_len = input;
							if(exp_len < 0 || exp_len > BUFFER_SIZE){
								debug_com_stream << "Incoming - wrong length (" << bytes_read <<")n";
								timed_out=false;
								break;
							}
						}
						// 2nd to second last byte
						u8CRC = MC3001::CalcCRCByte(input, u8CRC);
						bytes_read++;          
						input_buffer[bytes_read] = input;
					}
				}
			}
			threads.yield();
		}
		if(timed_out) debug_com_stream << "Incoing - Timed out\n";
		threads.yield();
	}
}

void MotorCom::handle_incoming(uint8_t *input_buffer, int msg_len){
	if (input_buffer[MC3001::MSG_NODE_IDX] != MC3001::NODE_ID){
		debug_com_stream << "Handle Incoming - Wrong Node ID\n";
		return;
	}

	const MC3001::controller_command cmd = (MC3001::controller_command)input_buffer[MC3001::MSG_CMD_IDX];
	switch (cmd)
	{
	case MC3001::Boot_Up:
	{
		debug_stream << "Boot Up!\n";
		Threads::Scope scope(state_lock);
	}
	break;

	case MC3001::SDO_Write:
	{
		Threads::Scope scope(comm_lock);
		response_received = matches_sdo_request(input_buffer) ? RESPONSE_SUCCESS : RESPONSE_ERROR;
	}
	break;

	case MC3001::SDO_Read:
	{
		Threads::Scope scope(comm_lock);
		const uint8_t val_size = input_buffer[MC3001::MSG_LEN_IDX] - MC3001::SDO_MSG_LEN;
		memcpy(value_ptr, &input_buffer[MC3001::MSG_DATA_IDX], val_size);
		response_received = matches_sdo_request(input_buffer) ? RESPONSE_SUCCESS : RESPONSE_ERROR;
	}
	break;

	case MC3001::SDO_Error:
	{
		uint8_t error_code, error_class;
		uint16_t add_error_code;
		if (msg_len == 11)
		{
			memcpy(&add_error_code, &input_buffer[6], sizeof(add_error_code));
			error_code = input_buffer[8];
			error_class = input_buffer[9];
			warning_stream << "SDO Error: 0x"<<_HEX(error_class) << " | 0x"<<_HEX(error_code)<< " | 0x"<<_HEX(add_error_code) <<"\n";
			warning_stream << MC3001::decode_SDO_error(error_class, error_code, add_error_code) << "\n";
			response_received = RESPONSE_ERROR;
		}
		else
		{
			warning_stream <<"SDO Error - wrong length\n";
		}
	}
	break;

	case MC3001::Statusword:
	{
		MC3001::statusword_t stat;
		memcpy(&stat, &input_buffer[3], sizeof(MC3001::statusword_t));
		state_lock.lock();
		statusword = stat;
		debug_com_stream << "flag before:\t" << _BIN(operationStatus_flag) << "\n";
		debug_com_stream << "mask:\t" << _BIN(MC3001::MASK_OP_STATE) << "\n";
		debug_com_stream << "stat:\t" << _BIN(statusword) << "\n";
		operationStatus_flag |= (stat & MC3001::MASK_OP_STATE);
		debug_com_stream << "flag after:\t" << _BIN(operationStatus_flag) << "\n";
		state_lock.unlock();
		// print_status();
	}
	break;

	case MC3001::Controlword:
	{
		debug_com_stream << "Incoming Controlword\n";
		if (exp_command == MC3001::Controlword)
		{
			Threads::Scope scope(comm_lock);
			const uint8_t err = input_buffer[3];
			response_received = err==0 ? RESPONSE_SUCCESS : RESPONSE_ERROR;
		}
	}
	break;

	case MC3001::EMCY:
	{
		uint16_t error_code, faulhaber_error;
		uint8_t error_reg;
		if (msg_len == 12)
		{
			memcpy(&error_code, &input_buffer[3], sizeof(error_code));
			error_reg = input_buffer[5];
			memcpy(&faulhaber_error, &input_buffer[6], sizeof(faulhaber_error));
			warning_stream << "EMCY: 0x" << _HEX(error_reg) << " | 0x" <<_HEX(error_code) << " | 0x"<<_HEX(faulhaber_error) << "   -  " << MC3001::decode_fault_register(faulhaber_error) << "\n";
		}
		else
		{
			warning_stream << "EMCY - wrong length\n";
		}
	}
	break;

	case MC3001::Trace_Log:
	{
		debug_stream << "Trace Log - not handled\n";
	}
	break;

	case MC3001::CMD_UNSET:
	{
		debug_com_stream << "Handle Incoming - Received CMD_UNSET!\n";
	}
	break; // Not possible

	default:
		debug_com_stream << "Handle Incoming - Unknown command\n";
		break;
	}
}


bool MotorCom::await_response(unsigned int timeout)
{
	unsigned long wait_start = millis();
	bool success = false;
	while(millis() - wait_start < timeout)
	{
		comm_lock.lock();
		if(response_received != NO_RESPONSE)
		{
			success = (response_received==RESPONSE_SUCCESS);
			break;
		}
		comm_lock.unlock();
		threads.yield();
	}
	if(response_received==NO_RESPONSE) debug_com_stream << "response timed out after "<< millis() - wait_start << "ms\n";
	exp_command = MC3001::CMD_UNSET;
	comm_lock.unlock();
	
	return success;
}

MC3001::statusword_t MotorCom::getStatusword(){
	Threads::Scope scope(state_lock);
	return statusword;
}
MC3001::controlword_t MotorCom::getControlword(){
	Threads::Scope scope(state_lock);
	return controlword;
}

bool MotorCom::awaitStatus(MC3001::statusword_t mask, MC3001::statusword_t value, unsigned long timeout_ms){
  unsigned long wait_start = millis();
  while(millis() - wait_start < timeout_ms) {
    if(checkStatus(mask, value)){
      return true;
    }
    threads.delay(5);
  }
  return false;
}

bool MotorCom::sync_state() {
  bool success = true;
  success = success && get_parameter<MC3001::statusword_t>(0x6041, 0x00, statusword);
  success = success && get_parameter<MC3001::controlword_t>(0x6040, 0x00, controlword);
  success = success && get_parameter<int8_t>(  0x6060, 0x00, operating_mode);
  success = success && get_parameter<uint16_t>(0x233F, 0x00, opmode_options);
  return success;
}

bool MotorCom::checkStatus(MC3001::statusword_t mask, MC3001::statusword_t value){
	return (getStatusword() & mask) == (value & mask);
}
bool MotorCom::checkFlaggedStatus(MC3001::statusword_t mask, MC3001::statusword_t value){
	Threads::Scope scope(state_lock);
	return (operationStatus_flag & mask) == (value & mask);
}
void MotorCom::resetStatusFlag(){
	Threads::Scope scope(state_lock);
	operationStatus_flag=0;
}

bool MotorCom::change_state(MC3001::controlword_t mask, MC3001::controlword_t cmd) {
	cmd = (getControlword() & ~mask) | (cmd & mask);
	return change_state(cmd);
}
bool MotorCom::change_state(MC3001::controlword_t cmd) {
	comm_lock.lock();
	exp_command = MC3001::Controlword;
	response_received = NO_RESPONSE;
	comm_lock.unlock();

	MC3001::write_controlword(cmd);
	if(await_response(response_timeout)){
		Threads::Scope scope(state_lock);
		debug_com_stream << "Controlword: " << _BIN(controlword) << " -> " << _BIN(cmd) << "\n";
		controlword = cmd;
		return true;
	}else{
		return false;
	}
}

bool MotorCom::set_operation_mode(uint16_t mask, uint16_t value){
  auto options = opmode_options;
  if( (value & mask) == opmode_options) return true;

  options = (options & ~mask) | (value & mask);

  if(set_parameter<uint16_t>(0x233F,0x00, options)){
    opmode_options = options;
	debug_com_stream << "OpMode: " << _BIN(opmode_options) << "\n";
    return true;
  }else{
    return false;
  }
}

bool MotorCom::set_control_mode(MC3001::controller_mode mode) {
  if(!operation_allowed){
	  warning_stream << "MotorCom - operation not allowed!\n";
	  return  false;
  }

  if(operating_mode == mode) return true;

  if(set_parameter<uint8_t>(0x6060, 0x00, (uint8_t)mode)){
    operating_mode = mode;
    return true;
  }else{
    return false;
  }
}

bool MotorCom::begin_operation(){
  return begin_operation(controlword);
}

bool MotorCom::begin_operation(MC3001::controlword_t ctrlw){
  if(!operation_allowed){
	  warning_stream << "MotorCom - operation not allowed!\n";
	  return  false;
  }

  constexpr uint16_t BIT_4 = (1<<4);

  // Set bit 4 low (if not already)
  if((ctrlw & BIT_4) > 0){
    if(!change_state(ctrlw & ~BIT_4))
      return false;
  }

  // Set bit 4 high
  return change_state(ctrlw | BIT_4);
}

bool MotorCom::save_configuration(MC3001::save_configuration_options type){
  const char* signature = "save";
  constexpr int TIMEOUT = 5000;
  auto tmp = response_timeout;
  response_timeout = TIMEOUT;
  bool success = set_parameter<uint32_t>(0x1010,type, *((uint32_t*)signature));
  response_timeout = tmp;
  return success;
}

bool MotorCom::load_configuration(MC3001::load_configuration_options type){
  const char* signature = "load";
  constexpr int TIMEOUT = 5000;
  auto tmp = response_timeout;
  response_timeout = TIMEOUT;
  bool success = set_parameter<uint32_t>(0x1011,type, *((uint32_t*)signature));
  response_timeout = tmp;
  return success;
}


//////////// TESTING ////////////

time_t MotorCom::test_write_delay(){
	typedef int32_t T;
	const uint16_t index = 0x607A; // target pos
	const uint8_t sub_index = 0;
	const T value = 0;
	unsigned long t0,t1;
	response_received = NO_RESPONSE;
	exp_command = MC3001::SDO_Write;
	exp_index = index;
	exp_subindex = sub_index;
	t0 = micros();
	set_parameter<T>(index, sub_index, value, true);
	t1 = micros();
	return t1-t0;
}

time_t MotorCom::test_read_delay(){
	typedef int32_t T;
	T value;
	unsigned long t0,t1;
	t0 = micros();
	get_parameter<T>(0x607A,0,value); // target pos
	t1 = micros();
	return t1-t0;
}

// DEBUGGING //

void MotorCom::print_status(Print &p) {
	MC3001::statusword_t sw = getStatusword();
	p << "Status: "<< _BIN(sw) <<"\n";
	p << "'" << MC3001::decode_state(sw) << "'\n";
	p << "'" << MC3001::decode_op_status(sw, operating_mode) << "'\n";
	p << "operation " << (operation_allowed ? "allowed":"disallowed") << "\n";
}


} // motor_interface