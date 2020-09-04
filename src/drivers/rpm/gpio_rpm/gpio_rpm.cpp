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

#include "gpio_rpm.h"

int
GpioRpm::task_spawn(int argc, char *argv[])
{
	auto *gpioRpm = new GpioRpm();

	if (!gpioRpm) {
		PX4_ERR("driver allocation failed");
		return PX4_ERROR;
	}

	_object.store(gpioRpm);
	_task_id = task_id_is_work_queue;

	gpioRpm->start();

	return PX4_OK;
}

GpioRpm::GpioRpm() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	param_get(param_find("GPIO_RPM_PULSES"), &_pulses_per_revolution);

	for (size_t i = 0; i < MAX_MOTORS; i++) {
		_motor_data_array[i].timed_out = true;
	}
}

void
GpioRpm::start()
{
	// Initialize the GPIO rising edge ISRs and callback
	gpio_init();

	ScheduleOnInterval(100_ms);
}

void
GpioRpm::gpio_init()
{
	irqstate_t state = px4_enter_critical_section();

	/* * * * * * * * * * * * * * * * * *
	*
	* Add more GPIO inputs here and define them in board_config.h if you'd like more.
	*
	* GPIO_RPM_1 = PWM CH5
	* GPIO_RPM_2 = PWM CH6
	*
	*/

	// Configure GPIO
	px4_arch_configgpio(GPIO_INPUT_RPM_1);
	px4_arch_configgpio(GPIO_INPUT_RPM_2);

	// Setup data ready on rising edge
	px4_arch_gpiosetevent(GPIO_INPUT_RPM_1, true, false, true, &GpioRpm::isr_callback1, this);
	px4_arch_gpiosetevent(GPIO_INPUT_RPM_2, true, false, true, &GpioRpm::isr_callback2, this);

	px4_leave_critical_section(state);
}

void
GpioRpm::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	// TESTING: remove when done testing
	actuator_outputs_s actuators {};
	actuators.output[0] = 200;
	_actuator_outputs_pub.publish(actuators);

	// Check if any of the motors have timed out -- this data is shared with ISR so we need to use a critical section
	// Alternatively we could ping pong buffer, but this should have minimal overhead
	irqstate_t state = px4_enter_critical_section();

	uint64_t now = hrt_absolute_time();

	gpio_rpm_s report {};
	report.timestamp = now;

	for (size_t i = 0; i < MAX_MOTORS; i++) {

		uint64_t time_since_last_revolution =  now - _motor_data_array[i].start_time;

		if (time_since_last_revolution > MOTOR_TIMEOUT_US) {

			if (!_motor_data_array[i].timed_out) {
				PX4_INFO("motor %d timed out", i + 1);
			}

			_motor_data_array[i].timed_out = true;
			_motor_data_array[i].pulse_count = 0;
			_motor_data_array[i].rpm = 0;
		}

		// We make a copy so we can leave the critical section to publish
		report.rpm[i] = _motor_data_array[i].rpm;
	}

	px4_leave_critical_section(state);

	_gpio_rpm_pub.publish(report);
}

int
GpioRpm::isr_callback1(int irq, void *context, void *arg)
{
	auto obj = static_cast<GpioRpm *>(arg);
	obj->isr_callback_handler(1);
	return PX4_OK;
}

int
GpioRpm::isr_callback2(int irq, void *context, void *arg)
{
	auto obj = static_cast<GpioRpm *>(arg);
	obj->isr_callback_handler(2);
	return PX4_OK;
}

void
GpioRpm::isr_callback_handler(int motor_number)
{
	MotorData *motor = &_motor_data_array[motor_number - 1]; // Motors are labelled 1-4 but indexed 0-3

	if (motor->timed_out) {
		PX4_INFO("ISR: motor: %d timed out", motor_number);

		motor->start_time = hrt_absolute_time();
		motor->timed_out = false;
	}

	motor->pulse_count++;

	// Check if we've completed a full revolution
	if (motor->pulse_count == _pulses_per_revolution) {

		uint64_t time_now = hrt_absolute_time();
		uint64_t time_delta_us = time_now - motor->start_time;
		motor->start_time = time_now;

		float rpm_instantaneous = 60 * 1e6 / time_delta_us;

		// TODO: apply a low pass or complimentary filter?
		motor->rpm = rpm_instantaneous;

		// Reset the counter
		motor->pulse_count = 0;
	}
}

int
GpioRpm::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Measures the RPM of a connected motor by counting the number of input pulses per revolution.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("gpio_rpm", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("info", "prints gpio_rpm capture info.");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return PX4_OK;
}

int
GpioRpm::custom_command(int argc, char *argv[])
{
	const char *input = argv[0];
	auto *obj = get_instance();

	if (!is_running() || !obj) {
		PX4_ERR("not running");
		return PX4_ERROR;
	}

	// We don't have any commands
	(void)input;

	print_usage();
	return PX4_ERROR;
}

extern "C" __EXPORT int gpio_rpm_main(int argc, char *argv[])
{
	return GpioRpm::main(argc, argv);
}
