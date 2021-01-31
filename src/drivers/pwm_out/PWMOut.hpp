/****************************************************************************
 *
 *   Copyright (c) 2012-2020 PX4 Development Team. All rights reserved.
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

#pragma once

#include <float.h>
#include <math.h>

#include <board_config.h>
#include <drivers/device/device.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_input_capture.h>
#include <drivers/drv_mixer.h>
#include <drivers/drv_pwm_output.h>
#include <lib/cdev/CDev.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/output_control.h>
#include <uORB/topics/output_feedback.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/safety.h>

using namespace time_literals;

/** Mode given via CLI */
enum PortMode {
	PORT_MODE_UNSET = 0,
	PORT_FULL_GPIO,
	PORT_FULL_PWM,
	PORT_PWM8,
	PORT_PWM6,
	PORT_PWM5,
	PORT_PWM4,
	PORT_PWM3,
	PORT_PWM2,
	PORT_PWM1,
	PORT_PWM3CAP1,
	PORT_PWM4CAP1,
	PORT_PWM4CAP2,
	PORT_PWM5CAP1,
	PORT_PWM2CAP2,
	PORT_CAPTURE,
};

#if !defined(BOARD_HAS_PWM)
#  error "board_config.h needs to define BOARD_HAS_PWM"
#endif

// TODO: keep in sync with drivers/camera_capture
#define PX4FMU_DEVICE_PATH	"/dev/px4fmu"

class PWMOut : public cdev::CDev, public ModuleBase<PWMOut>, public OutputModuleInterface
{
public:
	enum Mode {
		MODE_NONE,
		MODE_1PWM,
		MODE_2PWM,
		MODE_2PWM2CAP,
		MODE_3PWM,
		MODE_3PWM1CAP,
		MODE_4PWM,
		MODE_4PWM1CAP,
		MODE_4PWM2CAP,
		MODE_5PWM,
		MODE_5PWM1CAP,
		MODE_6PWM,
		MODE_8PWM,
		MODE_14PWM,
		MODE_4CAP,
		MODE_5CAP,
		MODE_6CAP,
	};
	PWMOut();
	virtual ~PWMOut();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	static char* get_param_prefix() { return "PWM_AUX"; }

	void Run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	/** change the FMU mode of the running module */
	static int fmu_new_mode(PortMode new_mode);

	static int test();

	virtual int	ioctl(file *filp, int cmd, unsigned long arg);

	virtual int	init();

	int		set_mode(Mode mode);
	Mode		get_mode() { return _mode; }

	static int	set_i2c_bus_clock(unsigned bus, unsigned clock_hz);

	static void	capture_trampoline(void *context, uint32_t chan_index,
					   hrt_abstime edge_time, uint32_t edge_state,
					   uint32_t overflow);

	void update_pwm_trims();

	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

private:
	static constexpr int FMU_MAX_ACTUATORS = DIRECT_PWM_OUTPUT_CHANNELS;
	static_assert(FMU_MAX_ACTUATORS <= MAX_ACTUATORS, "Increase MAX_ACTUATORS if this fails");

	MixingOutput _mixing_output{FMU_MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, true};

	Mode		_mode{MODE_NONE};

	uint32_t	_backup_schedule_interval_us{1_s};

	unsigned	_pwm_default_rate{50};
	unsigned	_pwm_alt_rate{50};
	uint32_t	_pwm_alt_rate_channels{0};

	unsigned	_current_update_rate{0};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionData<actuator_armed_s> _armed_sub{ORB_ID(actuator_armed)};
	uORB::SubscriptionData<safety_s> _safety_sub{ORB_ID(safety)};

	uORB::SubscriptionMultiArray<output_control_s> _output_control_subs{ORB_ID::output_control};
	uint16_t _assigned_functions[FMU_MAX_ACTUATORS]{};

	unsigned	_num_outputs{0};
	int		_class_instance{-1};

	bool		_pwm_on{false};
	uint32_t	_pwm_mask{0};
	bool		_pwm_initialized{false};
	bool		_test_mode{false};

	unsigned	_num_disarmed_set{0};

	bool		_legacy_mixer_mode{true};

	uint16_t	_reverse_pwm_mask{0}; // Local version of legacy mixer variable
	int16_t		_pwm_trim_values[FMU_MAX_ACTUATORS]{}; // Local version of legacy mixer variable
	uint16_t	_output_values[FMU_MAX_ACTUATORS]; // The actual values output to the pins
	void update_outputs();

	perf_counter_t	_cycle_perf;
	perf_counter_t	_interval_perf;

	void		capture_callback(uint32_t chan_index,
					 hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow);
	void		update_current_rate();
	int			set_pwm_rate(unsigned rate_map, unsigned default_rate, unsigned alt_rate);
	int			pwm_ioctl(file *filp, int cmd, unsigned long arg);
	void		update_pwm_rev_mask();
	void		update_pwm_out_state(bool on);
	uint16_t	get_pwm_rev_mask();
	void		get_pwm_trim_values(int16_t values[]);
	void		update_function_map();

	void		update_params();

	static void		sensor_reset(int ms);
	static void		peripheral_reset(int ms);

	int		capture_ioctl(file *filp, int cmd, unsigned long arg);

	PWMOut(const PWMOut &) = delete;
	PWMOut operator=(const PWMOut &) = delete;


	DEFINE_PARAMETERS_CUSTOM_PARENT(OutputModuleInterface,
		(ParamInt<px4::params::PWM_AUX_MODE>)    _p_pwm_aux_mode,
		(ParamInt<px4::params::PWM_AUX_FUNC1>)    _p_pwm_aux_func1,
		(ParamInt<px4::params::PWM_AUX_FUNC2>)    _p_pwm_aux_func2,
		(ParamInt<px4::params::PWM_AUX_FUNC3>)    _p_pwm_aux_func3,
		(ParamInt<px4::params::PWM_AUX_FUNC4>)    _p_pwm_aux_func4,
		(ParamInt<px4::params::PWM_AUX_FUNC5>)    _p_pwm_aux_func5,
		(ParamInt<px4::params::PWM_AUX_FUNC6>)    _p_pwm_aux_func6,
		(ParamInt<px4::params::PWM_AUX_FUNC7>)    _p_pwm_aux_func7,
		(ParamInt<px4::params::PWM_AUX_FUNC8>)    _p_pwm_aux_func8,
		(ParamInt<px4::params::PWM_AUX_MIN1>)    _p_pwm_aux_min1,
		(ParamInt<px4::params::PWM_AUX_MIN2>)    _p_pwm_aux_min2,
		(ParamInt<px4::params::PWM_AUX_MIN3>)    _p_pwm_aux_min3,
		(ParamInt<px4::params::PWM_AUX_MIN4>)    _p_pwm_aux_min4,
		(ParamInt<px4::params::PWM_AUX_MIN5>)    _p_pwm_aux_min5,
		(ParamInt<px4::params::PWM_AUX_MIN6>)    _p_pwm_aux_min6,
		(ParamInt<px4::params::PWM_AUX_MIN7>)    _p_pwm_aux_min7,
		(ParamInt<px4::params::PWM_AUX_MIN8>)    _p_pwm_aux_min8,
		(ParamInt<px4::params::PWM_AUX_MAX1>)    _p_pwm_aux_max1,
		(ParamInt<px4::params::PWM_AUX_MAX2>)    _p_pwm_aux_max2,
		(ParamInt<px4::params::PWM_AUX_MAX3>)    _p_pwm_aux_max3,
		(ParamInt<px4::params::PWM_AUX_MAX4>)    _p_pwm_aux_max4,
		(ParamInt<px4::params::PWM_AUX_MAX5>)    _p_pwm_aux_max5,
		(ParamInt<px4::params::PWM_AUX_MAX6>)    _p_pwm_aux_max6,
		(ParamInt<px4::params::PWM_AUX_MAX7>)    _p_pwm_aux_max7,
		(ParamInt<px4::params::PWM_AUX_MAX8>)    _p_pwm_aux_max8,
		(ParamFloat<px4::params::PWM_AUX_TRIM1>)    _p_pwm_aux_trim1,
		(ParamFloat<px4::params::PWM_AUX_TRIM2>)    _p_pwm_aux_trim2,
		(ParamFloat<px4::params::PWM_AUX_TRIM3>)    _p_pwm_aux_trim3,
		(ParamFloat<px4::params::PWM_AUX_TRIM4>)    _p_pwm_aux_trim4,
		(ParamFloat<px4::params::PWM_AUX_TRIM5>)    _p_pwm_aux_trim5,
		(ParamFloat<px4::params::PWM_AUX_TRIM6>)    _p_pwm_aux_trim6,
		(ParamFloat<px4::params::PWM_AUX_TRIM7>)    _p_pwm_aux_trim7,
		(ParamFloat<px4::params::PWM_AUX_TRIM8>)    _p_pwm_aux_trim8,
		(ParamInt<px4::params::PWM_AUX_DIS1>)    _p_pwm_aux_dis1,
		(ParamInt<px4::params::PWM_AUX_DIS2>)    _p_pwm_aux_dis2,
		(ParamInt<px4::params::PWM_AUX_DIS3>)    _p_pwm_aux_dis3,
		(ParamInt<px4::params::PWM_AUX_DIS4>)    _p_pwm_aux_dis4,
		(ParamInt<px4::params::PWM_AUX_DIS5>)    _p_pwm_aux_dis5,
		(ParamInt<px4::params::PWM_AUX_DIS6>)    _p_pwm_aux_dis6,
		(ParamInt<px4::params::PWM_AUX_DIS7>)    _p_pwm_aux_dis7,
		(ParamInt<px4::params::PWM_AUX_DIS8>)    _p_pwm_aux_dis8,
		(ParamInt<px4::params::PWM_AUX_REV1>)    _p_pwm_aux_rev1,
		(ParamInt<px4::params::PWM_AUX_REV2>)    _p_pwm_aux_rev2,
		(ParamInt<px4::params::PWM_AUX_REV3>)    _p_pwm_aux_rev3,
		(ParamInt<px4::params::PWM_AUX_REV4>)    _p_pwm_aux_rev4,
		(ParamInt<px4::params::PWM_AUX_REV5>)    _p_pwm_aux_rev5,
		(ParamInt<px4::params::PWM_AUX_REV6>)    _p_pwm_aux_rev6,
		(ParamInt<px4::params::PWM_AUX_REV7>)    _p_pwm_aux_rev7,
		(ParamInt<px4::params::PWM_AUX_REV8>)    _p_pwm_aux_rev8
	);
};
