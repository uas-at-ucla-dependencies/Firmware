#include "AviataMixerManager.hpp"
#include "aviata_mixers.h"
#include <px4_platform_common/defines.h>
#include <systemlib/mavlink_log.h>
#include <uORB/topics/mavlink_log.h>
#include <stdlib.h>
int cmpfunc (const void * a, const void * b) {
   return ( *(int*)a - *(int*)b );
}

// This doesn't make any sense but is required for some reason.
// constexpr float AviataMixerManager::STANDALONE_SENS_BOARD_Z_OFF;
constexpr AviataMixerManager::DualParamByName AviataMixerManager::DUAL_PARAMS_BY_NAME[];
constexpr size_t AviataMixerManager::DUAL_PARAMS_LEN;

AviataMixerManager::AviataMixerManager() : WorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	for (size_t i = 0; i < DUAL_PARAMS_LEN; i++) {
		_dual_params[i].param = param_find(DUAL_PARAMS_BY_NAME[i].param_name);
		_dual_params[i].standalone_val = DUAL_PARAMS_BY_NAME[i].standalone_val;
		_dual_params[i].aviata_val = DUAL_PARAMS_BY_NAME[i].aviata_val;
	}

	// Avoid unused variable warnings
	(void)_config_aviata_key;
	(void)_config_aviata_rotor_count;
}

void AviataMixerManager::init(MultirotorMixer* m) {
	_aviata_finalize_docking_sub.unregisterCallback();
	_aviata_set_configuration_sub.unregisterCallback();
	_aviata_set_standalone_sub.unregisterCallback();

	if (m == nullptr || m->get_multirotor_count() != AVIATA_NUM_ROTORS) {
		return; // Incompatible rotor count, so do nothing
	}

	_mixer = m;
	_standalone_rotors = m->get_rotors();
	_docked = false;

	mavlink_log_info(&_mavlink_log_pub, "AVIATA: Initializing AviataMixerManager");
	PX4_INFO("AVIATA: Initializing AviataMixerManager");

	set_standalone_params();

	_aviata_finalize_docking_sub.registerCallback();
	_aviata_set_configuration_sub.registerCallback();
	_aviata_set_standalone_sub.registerCallback();
}

void AviataMixerManager::Run() {
	bool aviata_finalize_docking_received = _aviata_finalize_docking_sub.update(&_aviata_finalize_docking_cmd);
	bool aviata_set_configuration_received = _aviata_set_configuration_sub.update(&_aviata_set_configuration_cmd);
	bool aviata_set_standalone_received = _aviata_set_standalone_sub.update(&_aviata_set_standalone_cmd);

	if (aviata_set_standalone_received) {
		set_standalone();
	} else {
		if (aviata_finalize_docking_received){
			finalize_docking(_aviata_finalize_docking_cmd.docking_slot,
			                 _aviata_finalize_docking_cmd.missing_drones,
			                 _aviata_finalize_docking_cmd.n_missing);
		}
		if (aviata_set_configuration_received) {
			set_configuration(_aviata_set_configuration_cmd.missing_drones,
			                  _aviata_set_configuration_cmd.n_missing);
		}
	}
}

void AviataMixerManager::finalize_docking(uint8_t docking_slot, uint8_t* missing_drones, uint8_t n_missing) {
	if (!_docked && validate_configuration(docking_slot, missing_drones, n_missing)) {
		_docked = true;
		_docking_slot = docking_slot;
		mavlink_log_info(&_mavlink_log_pub, "AVIATA DOCKED IN SLOT %u", docking_slot);
		PX4_INFO("AVIATA DOCKED IN SLOT %u", docking_slot);

		// float fc_yaw_rotation = STANDALONE_SENS_BOARD_Z_OFF - _config_aviata_drone_angle[docking_slot]; // Subtract due to opposite CW/CCW conventions
		// param_set_no_notification(_handle_SENS_BOARD_Z_OFF, &fc_yaw_rotation);

		// TODO Test without param setting
		/*
		for (size_t i = 0; i < DUAL_PARAMS_LEN; i++) {
			param_set_no_notification(_dual_params[i].param, &_dual_params[i].aviata_val);
		}
		param_notify_changes();
		*/

		set_configuration(missing_drones, n_missing);
		_mixer->set_aviata_rotor_index(AVIATA_NUM_ROTORS * docking_slot); // TODO set angle offset in _mixer
	} else {
		// AVIATA TODO print warning?
	}
}

void AviataMixerManager::set_configuration(uint8_t* missing_drones, uint8_t n_missing) {
	if (_docked && validate_configuration(_docking_slot, missing_drones, n_missing)) {
		qsort(missing_drones, n_missing, sizeof(uint8_t), cmpfunc);

		// Calculate index of aviata configuration, based on the predictable order in which combinations are generated.
		MultirotorGeometryUnderlyingType mixer_index = 0;
		int8_t missing_drone_prev = -1;
		for (uint8_t i = 0; i < n_missing; i++) {
			mixer_index += _n_choose_k[AVIATA_NUM_DRONES][i];
			for (uint8_t j = missing_drone_prev+1; j < missing_drones[i]; j++) {
				mixer_index += _n_choose_k[AVIATA_NUM_DRONES-j-1][n_missing-i-1];
			}
			missing_drone_prev = missing_drones[i];
		}

		_mixer->set_rotors(_config_aviata_index[mixer_index], AVIATA_NUM_DRONES * AVIATA_NUM_ROTORS);
		mavlink_log_info(&_mavlink_log_pub, "SELECTED AVIATA MIXER: %s", _config_aviata_key[mixer_index]);
		PX4_INFO("SELECTED AVIATA MIXER: %s", _config_aviata_key[mixer_index]);
	} else {
		// AVIATA TODO print warning?
	}
}

void AviataMixerManager::set_standalone() {
	if (_docked) {
		set_standalone_params();
		_mixer->set_aviata_rotor_index(0); // TODO set angle offset in _mixer
		_mixer->set_rotors(_standalone_rotors, AVIATA_NUM_ROTORS);
		_docked = false;
		mavlink_log_info(&_mavlink_log_pub, "AVIATA UNDOCKED");
		PX4_INFO("AVIATA UNDOCKED");
	} else {
		// AVIATA TODO print warning?
	}
}

void AviataMixerManager::set_standalone_params() {
	// param_set_no_notification(_handle_SENS_BOARD_Z_OFF, &STANDALONE_SENS_BOARD_Z_OFF);

	// TODO Test without param setting
	/*
	for (size_t i = 0; i < DUAL_PARAMS_LEN; i++) {
		param_set_no_notification(_dual_params[i].param, &_dual_params[i].standalone_val);
	}
	param_notify_changes();
	*/
}

bool AviataMixerManager::validate_configuration(uint8_t docking_slot, uint8_t* missing_drones, uint8_t n_missing) {
	if (n_missing > AVIATA_MAX_MISSING_DRONES) {
		return false;
	}
	for (uint8_t i = 0; i < n_missing; i++) {
		if (missing_drones[i] >= AVIATA_NUM_DRONES || missing_drones[i] == docking_slot) {
			return false;
		}
	}
	return true;
}

// From https://stackoverflow.com/a/11032879
template <int N>
int** AviataMixerManager::populate_n_choose_k_matrix(int C[N][N], int* C_rows[N]) {
	for (int k = 1; k < N; k++) C[0][k] = 0;
	for (int n = 0; n < N; n++) {
		C[n][0] = 1;
		C_rows[n] = C[n];
	}

	for (int n = 1; n < N; n++)
		for (int k = 1; k < N; k++)
			C[n][k] = C[n-1][k-1] + C[n-1][k];

	return C_rows;
}

static int n_choose_k_matrix[AVIATA_NUM_DRONES+1][AVIATA_NUM_DRONES+1];
static int* n_choose_k_rows[AVIATA_NUM_DRONES+1];
int** AviataMixerManager::_n_choose_k = populate_n_choose_k_matrix(n_choose_k_matrix, n_choose_k_rows);
