/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include <drivers/drv_hrt.h>

#include <parameters/param.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/gpio_rpm.h>
#include <uORB/topics/actuator_outputs.h>

using namespace time_literals;

static constexpr int MAX_MOTORS = GPIO_INPUT_RPM_MAX_MOTORS;
static constexpr uint64_t MOTOR_TIMEOUT_US =  500000; // If a full revolution does not occur within this time period, the motor is flagged as "timed_out"

class GpioRpm : public ModuleBase<GpioRpm>, public px4::ScheduledWorkItem
{
public:
	GpioRpm();
	void start();

	static int isr_callback1(int irq, void *context, void *arg);
	static int isr_callback2(int irq, void *context, void *arg);

	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	static int task_spawn(int argc, char *argv[]);

private:
	void Run() override;
	void isr_callback_handler(int motor_number);

	void gpio_init(void);

	struct MotorData
	{
		int rpm {};
		int pulse_count {};
		uint64_t start_time {};
		bool timed_out {};
	};

	MotorData _motor_data_array[MAX_MOTORS] {};

	int _pulses_per_revolution {};

	uORB::PublicationData<gpio_rpm_s> _gpio_rpm_pub{ORB_ID(gpio_rpm)};

	// TESTING: remove when done testing
	uORB::PublicationData<actuator_outputs_s> _actuator_outputs_pub{ORB_ID(actuator_outputs)};
};
