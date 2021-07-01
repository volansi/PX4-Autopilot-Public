
#pragma once


#include <dummy_data_types/reg/small_1_0.h>
#include <dummy_data_types/reg/medium_1_0.h>
#include <dummy_data_types/reg/large_1_0.h>
#include <dummy_data_types/reg/vlarge_1_0.h>
#include <dummy_data_types/reg/vvlarge_1_0.h>

#include "DynamicPortSubscriber.hpp"
#define _N_ 100
class UavcanDummySmlSubscriber : public UavcanDynamicPortSubscriber
{
public:
	long _messages_received{0};
	UavcanDummySmlSubscriber(CanardInstance &ins, UavcanParamManager &pmgr, uint8_t instance = 0) :
		UavcanDynamicPortSubscriber(ins, pmgr, "dummy.sml", instance) { };

	void subscribe() override
	{
		// Subscribe to messages reg.drone.physics.kinematics.geodetic.Point.0.1
		canardRxSubscribe(&_canard_instance,
				  CanardTransferKindMessage,
				  _subj_sub._canard_sub.port_id,
				  dummy_data_types_reg_small_1_0_EXTENT_BYTES_,
				  CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
				  &_subj_sub._canard_sub);
	};

	void callback(const CanardTransfer &receive) override
	{
		// Test with Yakut:
		// export YAKUT_TRANSPORT="pyuavcan.transport.can.CANTransport(pyuavcan.transport.can.media.slcan.SLCANMedia('/dev/serial/by-id/usb-Zubax_Robotics_Zubax_Babel_23002B000E514E413431302000000000-if00', 8, 115200), 42)"
		// yakut pub 22.dummy_data_types.reg.small.1.0 '{counter: 1}'

		dummy_data_types_reg_small_1_0 geo {};
		size_t geo_size_in_bits = receive.payload_size;
		dummy_data_types_reg_small_1_0_deserialize_(&geo, (const uint8_t *)receive.payload, &geo_size_in_bits);
		// printf("S%d ",geo.counter);
		// fflush(stdout);

		// Every N messages print out current state, will match if
		// there are no dropped messages, otherwise it won't
		if(_messages_received++ %_N_ == 0){
			printf("{S%ld:%ld ",geo.counter,_messages_received);
			fflush(stdout);
		}
	};

};

class UavcanDummyMedSubscriber : public UavcanDynamicPortSubscriber
{
public:
	long _messages_received{0};
	UavcanDummyMedSubscriber(CanardInstance &ins, UavcanParamManager &pmgr, uint8_t instance = 0) :
		UavcanDynamicPortSubscriber(ins, pmgr, "dummy.med", instance) { };

	void subscribe() override
	{
		// Subscribe to messages reg.drone.physics.kinematics.geodetic.Point.0.1
		canardRxSubscribe(&_canard_instance,
				  CanardTransferKindMessage,
				  _subj_sub._canard_sub.port_id,
				  dummy_data_types_reg_medium_1_0_EXTENT_BYTES_,
				  CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
				  &_subj_sub._canard_sub);
	};

	void callback(const CanardTransfer &receive) override
	{

		dummy_data_types_reg_medium_1_0 geo {};
		size_t geo_size_in_bits = receive.payload_size;
		dummy_data_types_reg_medium_1_0_deserialize_(&geo, (const uint8_t *)receive.payload, &geo_size_in_bits);
		printf("M%d ",geo.counter);
		fflush(stdout);

		// Every N messages print out current state, will match if
		// there are no dropped messages, otherwise it won't
		if(_messages_received++ %_N_ == 0){
			printf("{M%ld:%ld ",geo.counter,_messages_received);
			fflush(stdout);
		}
	};

};

class UavcanDummyLrgSubscriber : public UavcanDynamicPortSubscriber
{
public:
	long _messages_received{0};
	UavcanDummyLrgSubscriber(CanardInstance &ins, UavcanParamManager &pmgr, uint8_t instance = 0) :
		UavcanDynamicPortSubscriber(ins, pmgr, "dummy.lrg", instance) { };

	void subscribe() override
	{
		// Subscribe to messages reg.drone.physics.kinematics.geodetic.Point.0.1
		canardRxSubscribe(&_canard_instance,
				  CanardTransferKindMessage,
				  _subj_sub._canard_sub.port_id,
				  dummy_data_types_reg_vvlarge_1_0_EXTENT_BYTES_,
				  CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
				  &_subj_sub._canard_sub);
	};

	void callback(const CanardTransfer &receive) override
	{

		dummy_data_types_reg_vvlarge_1_0 geo {};
		size_t geo_size_in_bits = receive.payload_size;
		dummy_data_types_reg_vvlarge_1_0_deserialize_(&geo, (const uint8_t *)receive.payload, &geo_size_in_bits);

		printf("L%d ",geo.counter);
		fflush(stdout);

		// Every N messages print out current state, will match if
		// there are no dropped messages, otherwise it won't
		if(_messages_received++ %_N_ == 0){
			printf("{L%ld:%ld ",geo.counter,_messages_received);
			fflush(stdout);
		}
	};

};
