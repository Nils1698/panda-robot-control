#pragma once

#include <Arduino.h>
#include "MC3001_Interface.h"
#include "TeensyThreads.h"

#include "UserStream.h"

namespace motor_interface
{

class MotorCom
{
public:
	enum connection_status_t {
		CONNECTION_OK = true,
		CONNECTION_DOWN = false
	};

protected:
	connection_status_t connection_status; // TODO implement throughtout class
	// TODO maybe turn private and add getter method

	bool operation_allowed = false;

private:
	
	// VARIABLES //
	static constexpr int BUFFER_SIZE = 100;
	unsigned int incoming_timeout_ms = 50;
	enum comm_response_t {
		NO_RESPONSE = 0,
		RESPONSE_SUCCESS = 1,
		RESPONSE_ERROR = -1,
	};

	int thread_id;
	Threads::Mutex comm_lock;
	// Shared with receiver thread:
	uint16_t exp_index;
	uint8_t exp_subindex;
	uint8_t exp_val_size;
	MC3001::controller_command exp_command = MC3001::CMD_UNSET;
	void *value_ptr;
	comm_response_t response_received;
	unsigned long response_timeout = 100; // ms

	// Shared state
	Threads::Mutex state_lock;
	MC3001::statusword_t statusword;
	MC3001::statusword_t operationStatus_flag;
	MC3001::controlword_t controlword;
    uint16_t opmode_options;
    int8_t operating_mode;

public:
	MotorCom();
	void init();

	MC3001::statusword_t getStatusword();
	MC3001::controlword_t getControlword();
	bool checkStatus(MC3001::statusword_t mask, MC3001::statusword_t value);
	bool awaitStatus(MC3001::statusword_t mask, MC3001::statusword_t value, unsigned long timeout_ms);

	bool save_configuration(MC3001::save_configuration_options type);
	bool load_configuration(MC3001::load_configuration_options type);

// TEST METHODS //
	time_t test_read_delay();
	time_t test_write_delay();

// DEBUGGING //
  void print_status(Print &p);

    bool sync_state(); // TODO move to protected

protected:
	bool change_state(MC3001::controlword_t cmd);
	bool change_state(MC3001::controlword_t mask, MC3001::controlword_t cmd);
	bool set_operation_mode(uint16_t mask, uint16_t value);
    bool set_control_mode(MC3001::controller_mode mode);
	bool begin_operation();
	bool begin_operation(MC3001::controlword_t ctrlw);

	bool checkFlaggedStatus(MC3001::statusword_t mask, MC3001::statusword_t value);
	void resetStatusFlag();

private:
	// FUNCTIONS //
	static void receiver_thread(void *arg);
	void handle_incoming(uint8_t *input_buffer, int buffer_size);
	bool await_response(unsigned int timeout);
	// virtual void statusword_changed(MC3001::statusword_t statusword){} // TODO unused

	// INLINES //

	inline bool matches_sdo_request(uint8_t *msg){
		// debug
		// debug_stream << msg[MC3001::MSG_NODE_IDX] 		<< " : " << MC3001::NODE_ID	<< "\n";
		// debug_stream << msg[MC3001::MSG_CMD_IDX]      << " : " <<  exp_command			<< "\n";
		// debug_stream << msg[MC3001::MSG_INDEX_IDX]    << " : " <<  lowByte(exp_index) 	<< "\n";
		// debug_stream << msg[MC3001::MSG_INDEX_IDX+1]  << " : " <<  highByte(exp_index)	<< "\n";
		// debug_stream << msg[MC3001::MSG_SUBINDEX_IDX] << " : " <<  exp_subindex	<< "\n";
		return
		msg[MC3001::MSG_NODE_IDX]     == MC3001::NODE_ID	&&
		msg[MC3001::MSG_CMD_IDX]      == exp_command			&&
		msg[MC3001::MSG_INDEX_IDX]    == lowByte(exp_index)  &&
		msg[MC3001::MSG_INDEX_IDX+1]  == highByte(exp_index) &&
		msg[MC3001::MSG_SUBINDEX_IDX] == exp_subindex;
	}


	// TEMPLATE METHDOS //
public:

	template <typename T>
	inline bool set_parameter(const uint16_t index, const uint8_t sub_index, const T &value, bool wait_for_response=true)
	{
		comm_lock.lock();
		exp_index = index;
		exp_subindex = sub_index;
		exp_command = MC3001::SDO_Write;
		exp_val_size = sizeof(T);
		response_received = NO_RESPONSE;
		comm_lock.unlock();

		MC3001::write_SDO(index, sub_index, &value, sizeof(T));
		return wait_for_response==false || await_response(response_timeout);
	}

	template <typename T>
	inline bool get_parameter(const uint16_t index, const uint8_t sub_index, T &value)
	{
		comm_lock.lock();
		exp_index = index;
		exp_subindex = sub_index;
		exp_command = MC3001::SDO_Read;
		exp_val_size = sizeof(T);
		response_received = NO_RESPONSE;
		value_ptr = &value;
		comm_lock.unlock();

		MC3001::read_SDO(index, sub_index);
		return await_response(response_timeout);
	}
};

} // motor_interface