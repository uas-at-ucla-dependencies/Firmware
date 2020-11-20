#include "aviata_mixer_manager.hpp"
#include "aviata_mixers.h"
#include <iostream>
#include <algorithm>

static bool initialized = false;
static bool docked = false;
static MultirotorMixer* mixer = nullptr;
static const MultirotorMixer::Rotor* standalone_rotors = nullptr;

static void aviata_mixer_manager_set_standalone_params() {
	float fc_yaw_rotation = STANDALONE_SENS_BOARD_Z_OFF;
	param_set_no_notification(aviata_handle_SENS_BOARD_Z_OFF, &fc_yaw_rotation);

	for (size_t i = 0; i < aviata_shared_params_len; i++) {
		param_set_no_notification(aviata_shared_params[i].param, &aviata_shared_params[i].standalone_val);
	}
	param_notify_changes();
}

void aviata_mixer_manager_init(MultirotorMixer* m) {
	if (m->get_multirotor_count() == AVIATA_NUM_ROTORS) {
		aviata_mixer_manager_set_standalone_params();
		mixer = m;
		standalone_rotors = m->get_rotors();
		initialized = true;
	} else {
		initialized = false;
	}
	docked = false;

	// Avoid unused variable warnings
	(void)_config_aviata_key;
	(void)_config_aviata_rotor_count;
}

void aviata_mixer_manager_set_standalone() {
	if (docked) {
		aviata_mixer_manager_set_standalone_params();
		mixer->set_aviata_rotor_index(0);  // AVIATA TODO Might need to consider thread safety for set_aviata_rotor_index() and set_rotors()
		mixer->set_rotors(standalone_rotors, AVIATA_NUM_ROTORS);
		docked = false;
	} else {
		// AVIATA TODO print warning?
	}
}

void aviata_mixer_manager_finalize_docking(uint8_t docking_slot, uint8_t* missing_drones, uint8_t n_missing) {
	if (initialized && !docked) {
		docked = true;

		float fc_yaw_rotation = STANDALONE_SENS_BOARD_Z_OFF - _config_aviata_drone_angle[docking_slot]; // Subtract due to opposite CW/CCW conventions
		param_set_no_notification(aviata_handle_SENS_BOARD_Z_OFF, &fc_yaw_rotation);

		for (size_t i = 0; i < aviata_shared_params_len; i++) {
			param_set_no_notification(aviata_shared_params[i].param, &aviata_shared_params[i].aviata_val);
		}
		param_notify_changes();

		aviata_mixer_manager_set_configuration(missing_drones, n_missing);
		mixer->set_aviata_rotor_index(AVIATA_NUM_ROTORS * docking_slot);
	} else {
		// AVIATA TODO print warning?
	}
}

// From https://stackoverflow.com/a/11032879
template <int N>
static int populate_n_choose_k_matrix(int C[N][N]) {
	for (int k = 1; k <= N; k++) C[0][k] = 0;
	for (int n = 0; n <= N; n++) C[n][0] = 1;

	for (int n = 1; n <= N; n++)
		for (int k = 1; k <= N; k++)
			C[n][k] = C[n-1][k-1] + C[n-1][k];

	return 0;
}

static int n_choose_k[AVIATA_NUM_DRONES+1][AVIATA_NUM_DRONES+1]; // Used to calculate mixer index from missing_drones list
static int temp = populate_n_choose_k_matrix(n_choose_k);

void aviata_mixer_manager_set_configuration(uint8_t* missing_drones, uint8_t n_missing) {
	if (docked) {
		std::sort(missing_drones, missing_drones + n_missing);

		// Calculate index of aviata configuration, based on the predictable order in which combinations are generated.
		MultirotorGeometryUnderlyingType mixer_index = 0;
		int8_t missing_drone_prev = -1;
		for (uint8_t i = 0; i < n_missing; i++) {
			mixer_index += n_choose_k[AVIATA_NUM_DRONES][i];
			for (uint8_t j = missing_drone_prev+1; j < missing_drones[i]; j++) {
				mixer_index += n_choose_k[AVIATA_NUM_DRONES-j-1][n_missing-i-1];
			}
			missing_drone_prev = missing_drones[i];
		}

		mixer->set_rotors(_config_aviata_index[mixer_index], AVIATA_NUM_DRONES * AVIATA_NUM_ROTORS);
		// std::cout << "SELECTED AVIATA MIXER: " << _config_aviata_key[mixer_index] << std::endl;
	} else {
		// AVIATA TODO print warning?
	}
}
