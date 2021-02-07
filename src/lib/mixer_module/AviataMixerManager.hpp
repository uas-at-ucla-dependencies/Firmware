#pragma once

#include <px4_platform_common/module_params.h>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/aviata_finalize_docking.h>
#include <uORB/topics/aviata_set_configuration.h>
#include <uORB/topics/aviata_set_standalone.h>
#include <lib/mixer/MultirotorMixer/MultirotorMixer.hpp>

class AviataMixerManager: public px4::WorkItem
{
public:
	AviataMixerManager();
	void init(MultirotorMixer* m);

private:
	// static constexpr float STANDALONE_SENS_BOARD_Z_OFF = 0.0f;
	// const param_t _handle_SENS_BOARD_Z_OFF = param_find("SENS_BOARD_Z_OFF");

	struct DualParamByName {
		const char* param_name;
		float standalone_val;
		float aviata_val;
	};

	struct DualParam {
		param_t param;
		float standalone_val;
		float aviata_val;
	};

	// Define different parameter values for docked and undocked drones. Eventually, these should be made into PX4 parameters themselves.
	static constexpr DualParamByName DUAL_PARAMS_BY_NAME[] = {
		// Multicopter Attitude Control
		{ "MC_PITCHRATE_MAX" , 220.0f , 220.0f },
		{ "MC_ROLLRATE_MAX"  , 220.0f , 220.0f },
		{ "MC_YAWRATE_MAX"   , 200.0f , 45.0f  },
		{ "MPC_YAWRAUTO_MAX" , 45.0f  , 45.0f  },

		{ "MC_PITCH_P"       , 6.5f   , 6.5f   },
		{ "MC_ROLL_P"        , 6.5f   , 6.5f   },
		{ "MC_YAW_P"         , 2.8f   , 2.8f   },

		// Multicopter Position Control
		{ "MPC_ACC_HOR"      , 3.0f   , 3.0f   },
		{ "MPC_ACC_HOR_MAX"  , 5.0f   , 3.0f   },
		{ "MPC_ACC_DOWN_MAX" , 3.0f   , 3.0f   },
		{ "MPC_ACC_UP_MAX"   , 4.0f   , 4.0f   },

		{ "MPC_TKO_SPEED"    , 1.5f   , 1.5f   },
		{ "MPC_XY_CRUISE"    , 5.0f   , 5.0f   },
		{ "MPC_XY_VEL_MAX"   , 12.0f  , 5.0f   },
		{ "MPC_Z_VEL_MAX_DN" , 1.0f   , 1.0f   },
		{ "MPC_Z_VEL_MAX_UP" , 3.0f   , 3.0f   },

		{ "MPC_XY_P"         , 0.95f  , 0.95f  },
		{ "MPC_XY_TRAJ_P"    , 0.5f   , 0.5f   },
		{ "MPC_Z_P"          , 1.0f   , 1.0f   },

		{ "MPC_XY_VEL_P_ACC" , 1.8f   , 1.8f   },
		{ "MPC_XY_VEL_I_ACC" , 0.4f   , 0.4f   },
		{ "MPC_XY_VEL_D_ACC" , 0.2f   , 0.2f   },

		{ "MPC_Z_VEL_P_ACC"  , 4.0f   , 4.0f   },
		{ "MPC_Z_VEL_I_ACC"  , 2.0f   , 2.0f   },
		{ "MPC_Z_VEL_D_ACC"  , 0.0f   , 0.0f   },

		// Multicopter Rate Control
		{ "MC_PITCHRATE_P"   , 0.15f  , 0.15f  },
		{ "MC_PITCHRATE_I"   , 0.2f   , 0.2f   },
		{ "MC_PITCHRATE_D"   , 0.003f , 0.003f },
		{ "MC_PITCHRATE_FF"  , 0.0f   , 0.0f   },
		{ "MC_PR_INT_LIM"    , 0.30f  , 0.30f  },

		{ "MC_ROLLRATE_P"    , 0.15f  , 0.15f  },
		{ "MC_ROLLRATE_I"    , 0.2f   , 0.2f   },
		{ "MC_ROLLRATE_D"    , 0.003f , 0.003f },
		{ "MC_ROLLRATE_FF"   , 0.0f   , 0.0f   },
		{ "MC_RR_INT_LIM"    , 0.30f  , 0.30f  },

		{ "MC_YAWRATE_P"     , 0.2f   , 0.2f   },
		{ "MC_YAWRATE_I"     , 0.1f   , 0.1f   },
		{ "MC_YAWRATE_D"     , 0.0f   , 0.0f   },
		{ "MC_YAWRATE_FF"    , 0.0f   , 0.0f   },
		{ "MC_YR_INT_LIM"    , 0.30f  , 0.30f  }
	};

	static constexpr size_t DUAL_PARAMS_LEN = sizeof(DUAL_PARAMS_BY_NAME) / sizeof(DUAL_PARAMS_BY_NAME[0]);

	DualParam _dual_params[DUAL_PARAMS_LEN];

	uORB::SubscriptionCallbackWorkItem _aviata_finalize_docking_sub{this, ORB_ID(aviata_finalize_docking)};
	uORB::SubscriptionCallbackWorkItem _aviata_set_configuration_sub{this, ORB_ID(aviata_set_configuration)};
	uORB::SubscriptionCallbackWorkItem _aviata_set_standalone_sub{this, ORB_ID(aviata_set_standalone)};
	orb_advert_t _mavlink_log_pub{nullptr};

	aviata_finalize_docking_s _aviata_finalize_docking_cmd;
	aviata_set_configuration_s _aviata_set_configuration_cmd;
	aviata_set_standalone_s _aviata_set_standalone_cmd;

	MultirotorMixer* _mixer;
	const MultirotorMixer::Rotor* _standalone_rotors;
	bool _docked;
	uint8_t _docking_slot;

	void Run() override;

	// Configure the mixer once docked based on the docking slot and the missing drones.
	// Also update PID values, flight controller orientation, etc.
	void finalize_docking(uint8_t docking_slot, uint8_t* missing_drones, uint8_t n_missing);

	// Update the mixer according to which drones are missing
	void set_configuration(uint8_t* missing_drones, uint8_t n_missing);

	// Set mixer to its original state (a standard hexacopter)
	void set_standalone();

	void set_standalone_params();

	bool validate_configuration(uint8_t docking_slot, uint8_t* missing_drones, uint8_t n_missing);

	template<int N>
	static int** populate_n_choose_k_matrix(int C[N][N], int* C_rows[N]);

	static int** _n_choose_k; // Used to calculate mixer index from missing_drones list
};
