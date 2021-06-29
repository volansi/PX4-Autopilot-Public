/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file Gyro.hpp
 *
 * Defines basic functionality of UAVCAN v1 Gyro publisher
 *
 * @author Shaun Cosgrove <shaun@flyvoly.com>
 */

#pragma once

// DS-15 Specification Messages
#include <reg/drone/physics/kinematics/rotation/Planar_0_1.h>

#include "Publisher.hpp"

#include <uORB/topics/sensor_gyro.h>

class UavcanGyroPublisher : public UavcanPublisher
{
public:
	UavcanGyroPublisher(CanardInstance &ins, UavcanParamManager &pmgr, uint8_t instance = 0) :
		UavcanPublisher(ins, pmgr, "gyro", instance)
	{

	};

	// Update the uORB Subscription and broadcast a UAVCAN message
	virtual void update() override
	{
		if (_gyro_sub.updated() && _port_id != CANARD_PORT_ID_UNSET
		    && _port_id != 0) { //FIXME either make default param UNSET or handle 0 in base class
			sensor_gyro_s gyro {};
			_gyro_sub.update(&gyro);

			reg_drone_physics_kinematics_rotation_Planar_0_1 rs{};
			rs.angular_acceleration.radian_per_second_per_second = gyro.x;

			uint8_t gyro_payload_buffer[reg_drone_physics_kinematics_rotation_Planar_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

			CanardTransfer transfer = {
				.timestamp_usec = hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
				.priority       = CanardPriorityNominal,
				.transfer_kind  = CanardTransferKindMessage,
				.port_id        = _port_id, // This is the subject-ID.
				.remote_node_id = CANARD_NODE_ID_UNSET,
				.transfer_id    = _transfer_id,
				.payload_size   = reg_drone_physics_kinematics_rotation_Planar_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_,
				.payload        = &gyro_payload_buffer,
			};

			int32_t result = reg_drone_physics_kinematics_rotation_Planar_0_1_serialize_(&rs, gyro_payload_buffer,
					 &transfer.payload_size);

			if (result == 0) {
				// set the data ready in the buffer and chop if needed
				++_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
				result = canardTxPush(&_canard_instance, &transfer);
			}
		}
	};

private:

	uORB::Subscription _gyro_sub{ORB_ID(sensor_gyro)};
	CanardTransferID _transfer_id_2 {0};
};
