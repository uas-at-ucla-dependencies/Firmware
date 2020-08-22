/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "EKF2Selector.hpp"

using namespace time_literals;
using matrix::Quatf;
using matrix::Vector2f;

EKF2Selector::EKF2Selector() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	updateParams();
}

EKF2Selector::~EKF2Selector()
{
	ScheduleClear();
}

bool EKF2Selector::init()
{
	ScheduleNow();
	return true;
}

bool EKF2Selector::SelectInstance(uint8_t ekf_instance, bool force_reselect)
{
	if (ekf_instance != _selected_instance || force_reselect) {

		// switch callback registration
		if (_selected_instance != UINT8_MAX) {
			_instance[_selected_instance].estimator_attitude_sub.unregisterCallback();
		}

		_instance[ekf_instance].estimator_attitude_sub.registerCallback();

		if (_selected_instance != UINT8_MAX) {
			PX4_WARN("primary EKF changed %d -> %d", _selected_instance, ekf_instance);
		}

		_selected_instance = ekf_instance;
		_instance_changed_count++;
		_last_instance_change = hrt_absolute_time();

		// handle resets on change

		// vehicle_attitude: quat_reset_counter
		vehicle_attitude_s attitude_new;

		if (_instance[_selected_instance].estimator_attitude_sub.copy(&attitude_new)) {
			++_quat_reset_counter;
			_delta_q_reset = Quatf{attitude_new.q} - Quatf{_attitude_last.q};

			// save new estimator_attitude
			_attitude_last = attitude_new;

			attitude_new.quat_reset_counter = _quat_reset_counter;
			_delta_q_reset.copyTo(attitude_new.delta_q_reset);

			// publish new vehicle_attitude immediately
			_vehicle_attitude_pub.publish(attitude_new);
		}

		// vehicle_local_position: xy_reset_counter, z_reset_counter, vxy_reset_counter, vz_reset_counter
		vehicle_local_position_s local_position_new;

		if (_instance[_selected_instance].estimator_local_position_sub.copy(&local_position_new)) {

			// update sensor_selection immediately
			{
				sensor_selection_s sensor_selection{};
				sensor_selection.accel_device_id = _instance[_selected_instance].estimator_status.accel_device_id;
				sensor_selection.gyro_device_id = _instance[_selected_instance].estimator_status.gyro_device_id;
				sensor_selection.timestamp = hrt_absolute_time();
				_sensor_selection_pub.publish(sensor_selection);
			}

			++_xy_reset_counter;
			++_z_reset_counter;
			++_vxy_reset_counter;
			++_vz_reset_counter;
			++_heading_reset_counter;

			_delta_xy = Vector2f{local_position_new.x, local_position_new.y} - Vector2f{_local_position_last.x, _local_position_last.y};
			_delta_z = local_position_new.z - _local_position_last.z;
			_delta_vxy = Vector2f{local_position_new.vx, local_position_new.vy} - Vector2f{_local_position_last.vx, _local_position_last.vy};
			_delta_vz = local_position_new.vz - _local_position_last.vz;
			_delta_heading = matrix::wrap_2pi(local_position_new.heading - _local_position_last.heading);

			// save new estimator_local_position
			_local_position_last = local_position_new;

			local_position_new.xy_reset_counter = _xy_reset_counter;
			local_position_new.z_reset_counter = _z_reset_counter;
			local_position_new.vxy_reset_counter = _vxy_reset_counter;
			local_position_new.vz_reset_counter = _vz_reset_counter;
			_delta_xy.copyTo(local_position_new.delta_xy);
			_delta_vxy.copyTo(local_position_new.delta_vxy);
			local_position_new.delta_z = _delta_z;
			local_position_new.delta_vz = _delta_vz;
			local_position_new.delta_heading = _delta_heading;

			// publish new vehicle_local_position immediately
			_vehicle_local_position_pub.publish(local_position_new);
		}

		// vehicle_global_position: lat_lon_reset_counter, alt_reset_counter
		vehicle_global_position_s global_position_new;

		if (_instance[_selected_instance].estimator_global_position_sub.copy(&global_position_new)) {
			++_alt_reset_counter;
			++_lat_lon_reset_counter;

			_delta_lat = global_position_new.lat - _global_position_last.lat;
			_delta_lon = global_position_new.lon - _global_position_last.lon;
			_delta_alt = global_position_new.delta_alt - _global_position_last.delta_alt;

			// save new estimator_global_position
			_global_position_last = global_position_new;

			global_position_new.alt_reset_counter = _alt_reset_counter;
			global_position_new.delta_alt = _delta_alt;

			// publish new vehicle_global_position immediately
			_vehicle_global_position_pub.publish(_global_position_last);
		}

		return true;
	}

	return false;
}

void EKF2Selector::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	// re-schedule as watchdog timeout
	ScheduleDelayed(10_ms);

	// update combined test ratio for all estimators
	for (int i = 0; i < MAX_INSTANCES; i++) {
		if (_instance[i].estimator_status_sub.update(&_instance[i].estimator_status)) {
			const estimator_status_s &status = _instance[i].estimator_status;

			_instance[i].timestamp = status.timestamp;

			_instance[i].combined_test_ratio = status.vel_test_ratio
							   + status.pos_test_ratio
							   + status.hgt_test_ratio;

			_instance[i].tilt_align = status.control_mode_flags & (1 << estimator_status_s::CS_TILT_ALIGN);
			_instance[i].yaw_align = status.control_mode_flags & (1 << estimator_status_s::CS_YAW_ALIGN);

			_instance[i].filter_fault_flags = status.filter_fault_flags;

			if ((i + 1) > _available_instances) {
				_available_instances = i + 1;
			}
		}
	}

	// TODO: handle switchover
	//   - don't switch until absolutely necessary
	//   - handle reset counter and deltas
	//   - check all fault flags
	//   - publish sensor_selection

	// find highest test ratio and register callback
	uint8_t best_ekf_instance = UINT8_MAX;
	float best_test_ratio = FLT_MAX;

	for (int i = 0; i < MAX_INSTANCES; i++) {
		if (hrt_elapsed_time(&_instance[i].timestamp) < 20_ms) {
			if (_instance[i].tilt_align && _instance[i].yaw_align) {
				if (_instance[i].combined_test_ratio < best_test_ratio) {
					best_test_ratio = _instance[i].combined_test_ratio;
					best_ekf_instance = i;
				}
			}
		}
	}

	const uint8_t requested_instance = _requested_instance.load();

	if (requested_instance != _selected_instance) {
		if (requested_instance < _available_instances) {
			PX4_WARN("manually selecting instance %d", requested_instance);
			SelectInstance(requested_instance, true); // force reselect
		}

		_requested_instance.store(_selected_instance);

	} else {
		if (best_ekf_instance != _selected_instance) {

			bool force_reselect = false;

			// critical error if primary has timed out
			if ((_selected_instance == UINT8_MAX) || (hrt_elapsed_time(&_instance[_selected_instance].timestamp) > 50_ms)) {
				force_reselect = true;
			}

			// only switch if current primary has a fault and current best does not
			if (force_reselect ||
			    (_instance[_selected_instance].filter_fault_flags && !_instance[best_ekf_instance].filter_fault_flags)
			   ) {
				SelectInstance(best_ekf_instance, force_reselect);
			}

		}
	}

	if (_selected_instance != UINT8_MAX) {

		// vehicle_attitude
		vehicle_attitude_s attitude;

		if (_instance[_selected_instance].estimator_attitude_sub.update(&attitude)) {

			if (attitude.quat_reset_counter > _attitude_last.quat_reset_counter) {
				++_quat_reset_counter;
				_delta_q_reset = Quatf{attitude.delta_q_reset};
			}

			// save latest estimator_attitude
			_attitude_last = attitude;

			// update with total estimator resets
			attitude.quat_reset_counter = _quat_reset_counter;
			_delta_q_reset.copyTo(attitude.delta_q_reset);

			_vehicle_attitude_pub.publish(attitude);
		}

		// vehicle_local_position
		vehicle_local_position_s local_position;

		if (_instance[_selected_instance].estimator_local_position_sub.update(&local_position)) {

			// XY reset
			if (local_position.xy_reset_counter > _local_position_last.xy_reset_counter) {
				++_xy_reset_counter;
				_delta_xy = Vector2f{local_position.delta_xy};
			}

			// Z reset
			if (local_position.z_reset_counter > _local_position_last.z_reset_counter) {
				++_z_reset_counter;
				_delta_z = local_position.delta_z;
			}

			// VXY reset
			if (local_position.vxy_reset_counter > _local_position_last.vxy_reset_counter) {
				++_vxy_reset_counter;
				_delta_vxy = Vector2f{local_position.delta_vxy};
			}

			// VZ reset
			if (local_position.vz_reset_counter > _local_position_last.vz_reset_counter) {
				++_vz_reset_counter;
				_delta_z = local_position.delta_vz;
			}

			// heading reset
			if (local_position.heading_reset_counter > _local_position_last.heading_reset_counter) {
				++_heading_reset_counter;
				_delta_heading = local_position.delta_heading;
			}

			// save new estimator_local_position
			_local_position_last = local_position;

			// update with total estimator resets
			local_position.xy_reset_counter = _xy_reset_counter;
			local_position.z_reset_counter = _z_reset_counter;
			local_position.vxy_reset_counter = _vxy_reset_counter;
			local_position.vz_reset_counter = _vz_reset_counter;
			local_position.heading_reset_counter = _heading_reset_counter;

			_delta_xy.copyTo(local_position.delta_xy);
			_delta_vxy.copyTo(local_position.delta_vxy);
			local_position.delta_z = _delta_z;
			local_position.delta_vz = _delta_vz;
			local_position.delta_heading = _delta_heading;

			_vehicle_local_position_pub.publish(local_position);
		}

		// vehicle_global_position
		vehicle_global_position_s global_position;

		if (_instance[_selected_instance].estimator_global_position_sub.update(&global_position)) {

			// Z reset
			if (global_position.alt_reset_counter > _global_position_last.alt_reset_counter) {
				++_alt_reset_counter;
				_delta_alt = global_position.delta_alt;
			}

			// save new estimator_global_position
			_global_position_last = global_position;

			// update with total estimator resets
			global_position.alt_reset_counter = _alt_reset_counter;
			global_position.delta_alt = _delta_alt;

			_vehicle_global_position_pub.publish(global_position);
		}

		estimator_selector_status_s selector_status{};
		selector_status.primary_instance = _selected_instance;
		selector_status.instances_available = _available_instances;
		selector_status.instance_changed_count = _instance_changed_count;
		selector_status.last_instance_change = _last_instance_change;

		for (int i = 0; i < MAX_INSTANCES; i++) {
			selector_status.combined_error[i] = _instance[i].combined_test_ratio;
		}

		selector_status.timestamp = hrt_absolute_time();
		_estimator_selector_status_pub.publish(selector_status);
	}
}

int EKF2Selector::print_status()
{
	PX4_INFO("available instances: %d", _available_instances);

	if (_selected_instance != UINT8_MAX) {
		PX4_INFO("selected instance: %d", _selected_instance);

	} else {
		PX4_WARN("selected instance: None");
	}

	for (const auto &inst : _instance) {
		if (inst.timestamp > 0) {
			PX4_INFO("%d: ACC: %d, GYRO: %d, MAG: %d, tilt align: %d, yaw align: %d, combined test ratio: %.6f",
				 inst.instance, inst.estimator_status.accel_device_id, inst.estimator_status.gyro_device_id,
				 inst.estimator_status.mag_device_id,
				 inst.tilt_align, inst.yaw_align, (double)inst.combined_test_ratio);
		}
	}

	return 0;
}

int EKF2Selector::task_spawn(int argc, char *argv[])
{
	EKF2Selector *instance = new EKF2Selector();

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

int EKF2Selector::custom_command(int argc, char *argv[])
{
	// TODO: manually select different instance

	const char *verb = argv[0];

	if (!strcmp(verb, "select")) {
		if (is_running()) {
			int instance = atoi(argv[1]);
			get_instance()->RequestInstanceChange(instance);
		}

		return 0;
	}


	return print_usage("unknown command");
}

int EKF2Selector::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description


)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ekf2_selector", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	PRINT_MODULE_USAGE_COMMAND("switch");

	return 0;
}

extern "C" __EXPORT int ekf2_selector_main(int argc, char *argv[])
{
	return EKF2Selector::main(argc, argv);
}
