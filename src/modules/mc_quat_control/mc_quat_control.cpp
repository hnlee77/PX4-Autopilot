#include "mc_quat_control.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

MulticopterQuatControl::MulticopterQuatControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_actuators_0_pub(vtol ? ORB_ID(actuator_controls_virtual_mc) : ORB_ID(actuator_controls_0)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
	_controller_status_pub.advertise();
}

MulticopterQuatControl::~MulticopterQuatControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterQuatControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("vehicle_angular_velocity callback registration failed!");
		return false;
	}

	return true;
}

void
MulticopterQuatControl::parameters_updated()
{
	// my controller gain parameter using _rate_control
}

void
MulticopterQuatControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		// grab corresponding vehicle_angular_acceleration immediately after vehicle_angular_velocity copy
		vehicle_angular_acceleration_s v_angular_acceleration{};
		_vehicle_angular_acceleration_sub.copy(&v_angular_acceleration);

		const hrt_abstime now = angular_velocity.timestamp_sample;

		// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		// const Vector3f angular_accel{v_angular_acceleration.xyz};
		const Vector3f rates{angular_velocity.xyz};  // current state

		/* check for updates in other topics */
		_v_control_mode_sub.update(&_v_control_mode);

		// if (_vehicle_land_detected_sub.updated()) {
		// 	vehicle_land_detected_s vehicle_land_detected;

		// 	if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
		// 		_landed = vehicle_land_detected.landed;
		// 		_maybe_landed = vehicle_land_detected.maybe_landed;
		// 	}
		// }

		// _vehicle_status_sub.update(&_vehicle_status);

		// if (_landing_gear_sub.updated()) {
		// 	landing_gear_s landing_gear;

		// 	if (_landing_gear_sub.copy(&landing_gear)) {
		// 		if (landing_gear.landing_gear != landing_gear_s::GEAR_KEEP) {
		// 			if (landing_gear.landing_gear == landing_gear_s::GEAR_UP && (_landed || _maybe_landed)) {
		// 				mavlink_log_critical(&_mavlink_log_pub, "Landed, unable to retract landing gear\t");
		// 				events::send(events::ID("mc_rate_control_not_retract_landing_gear_landed"),
		// 				{events::Log::Error, events::LogInternal::Info},
		// 				"Landed, unable to retract landing gear");

		// 			} else {
		// 				_landing_gear = landing_gear.landing_gear;
		// 			}
		// 		}
		// 	}
		// }
		if (_v_control_mode.flag_control_rates_enabled && _v_control_mode.flag_control_attitude_enabled) {
			vehicle_attitude_s v_att;
			vehicle_attitude_setpoint_s v_att_s;
			Vector3f att_control;
			// vehicle_rates_setpoint_s vehicle_rates_setpoint{};

			if (_vehicle_attitude_sub.update(&v_att) && _vehicle_attitude_setpoint_sub.copy(&v_att_s)){
				att_controllerModelClass::ExtU_att_controller_T in_to_controller = {
					.q = {v_att.q[0], v_att.q[1], v_att.q[2], v_att.q[3]},
					.w = {angular_velocity.xyz[0], angular_velocity.xyz[1], angular_velocity.xyz[2]},
					.q_d = {v_att_s.q_d[0], v_att_s.q_d[1], v_att_s.q_d[2], v_att_s.q_d[3]}
				};

				_att_control.setExternalInputs(&in_to_controller);
				_att_control.step();

				att_controllerModelClass::ExtY_att_controller_T out_from_controller = _att_control.getExternalOutputs();

				att_control = {
					(float)out_from_controller.tau[0],
					(float)out_from_controller.tau[1],
					(float)out_from_controller.tau[2]};

				_thrust_sp = -v_att_s.thrust_body[2];
				// PX4_INFO("Thrust: %.2f", (double)_thrust_sp);
				// _thrust_sp = math::constrain(-v_att_s.thrust_body[2], 0.0f, 1.0f);
				// _thrust_sp = 1.0;
				// _thrust_sp = vehicle_rates_setpoint.thrust_body[2];
			}

			// publish actuator controls
			actuator_controls_s actuators{};
			actuators.control[actuator_controls_s::INDEX_ROLL] = PX4_ISFINITE(att_control(0)) ? att_control(0) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_PITCH] = PX4_ISFINITE(att_control(1)) ? att_control(1) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_YAW] = PX4_ISFINITE(att_control(2)) ? att_control(2) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_THROTTLE] = PX4_ISFINITE(_thrust_sp) ? _thrust_sp : 0.0f;
			actuators.control[actuator_controls_s::INDEX_LANDING_GEAR] = _landing_gear;
			actuators.timestamp_sample = angular_velocity.timestamp_sample;

			if (!_vehicle_status.is_vtol) {
				publishTorqueSetpoint(att_control, angular_velocity.timestamp_sample);
				publishThrustSetpoint(_thrust_sp, angular_velocity.timestamp_sample);
			}

			actuators.timestamp = hrt_absolute_time();
			_actuators_0_pub.publish(actuators);

			updateActuatorControlsStatus(actuators, dt);

		} else if (_v_control_mode.flag_control_termination_enabled) {
			if (!_vehicle_status.is_vtol) {
				// publish actuator controls
				actuator_controls_s actuators{};
				actuators.timestamp = hrt_absolute_time();
				_actuators_0_pub.publish(actuators);
			}

		}

	}

	perf_end(_loop_perf);
}

void MulticopterQuatControl::publishTorqueSetpoint(const Vector3f &torque_sp, const hrt_abstime &timestamp_sample)
{
	vehicle_torque_setpoint_s v_torque_sp = {};
	v_torque_sp.timestamp = hrt_absolute_time();
	v_torque_sp.timestamp_sample = timestamp_sample;
	v_torque_sp.xyz[0] = (PX4_ISFINITE(torque_sp(0))) ? torque_sp(0) : 0.0f;
	v_torque_sp.xyz[1] = (PX4_ISFINITE(torque_sp(1))) ? torque_sp(1) : 0.0f;
	v_torque_sp.xyz[2] = (PX4_ISFINITE(torque_sp(2))) ? torque_sp(2) : 0.0f;

	_vehicle_torque_setpoint_pub.publish(v_torque_sp);
}

void MulticopterQuatControl::publishThrustSetpoint(const float thrust_sp, const hrt_abstime &timestamp_sample)
{
	vehicle_thrust_setpoint_s v_thrust_sp = {};
	v_thrust_sp.timestamp = hrt_absolute_time();
	v_thrust_sp.timestamp_sample = timestamp_sample;
	v_thrust_sp.xyz[0] = 0.0f;
	v_thrust_sp.xyz[1] = 0.0f;
	v_thrust_sp.xyz[2] = PX4_ISFINITE(thrust_sp) ? -thrust_sp : 0.0f; // Z is Down

	_vehicle_thrust_setpoint_pub.publish(v_thrust_sp);
}

void MulticopterQuatControl::updateActuatorControlsStatus(const actuator_controls_s &actuators, float dt)
{
	for (int i = 0; i < 4; i++) {
		_control_energy[i] += actuators.control[i] * actuators.control[i] * dt;
	}

	_energy_integration_time += dt;

	if (_energy_integration_time > 500e-3f) {

		actuator_controls_status_s status;
		status.timestamp = actuators.timestamp;

		for (int i = 0; i < 4; i++) {
			status.control_power[i] = _control_energy[i] / _energy_integration_time;
			_control_energy[i] = 0.f;
		}

		_actuator_controls_status_0_pub.publish(status);
		_energy_integration_time = 0.f;
	}
}

int MulticopterQuatControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterQuatControl *instance = new MulticopterQuatControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MulticopterQuatControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterQuatControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter rate controller. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has a PID loop for angular rate error.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_quat_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_quat_control_main(int argc, char *argv[])
{
	return MulticopterQuatControl::main(argc, argv);
}
