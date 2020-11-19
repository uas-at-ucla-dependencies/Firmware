#include "MultirotorMixer.hpp"
#include <parameters/param.h>

#define STANDALONE_SENS_BOARD_Z_OFF 0.0f

const param_t aviata_handle_SENS_BOARD_Z_OFF = param_find("SENS_BOARD_Z_OFF");

struct AviataSharedParam {
	param_t param;
	float standalone_val;
	float aviata_val;
};

// Define different parameter values for docked and undocked drones. Eventually, these should be made into PX4 parameters themselves.
const AviataSharedParam aviata_shared_params[] = {
	// Multicopter Attitude Control
	{ param_find("MC_PITCHRATE_MAX") , 220.0f , 220.0f },
	{ param_find("MC_ROLLRATE_MAX")  , 220.0f , 220.0f },
	{ param_find("MC_YAWRATE_MAX")   , 200.0f , 200.0f },
	{ param_find("MPC_YAWRAUTO_MAX") , 45.0f  , 45.0f  },

	{ param_find("MC_PITCH_P")       , 6.5f   , 6.5f   },
	{ param_find("MC_ROLL_P")        , 6.5f   , 6.5f   },
	{ param_find("MC_YAW_P")         , 2.8f   , 2.8f   },

	// Multicopter Position Control
	{ param_find("MPC_ACC_HOR")      , 3.0f   , 3.0f   },
	{ param_find("MPC_ACC_HOR_MAX")  , 5.0f   , 5.0f   },
	{ param_find("MPC_ACC_DOWN_MAX") , 3.0f   , 3.0f   },
	{ param_find("MPC_ACC_UP_MAX")   , 4.0f   , 4.0f   },

	{ param_find("MPC_TKO_SPEED")    , 1.5f   , 1.5f   },
	{ param_find("MPC_XY_CRUISE")    , 5.0f   , 5.0f   },
	{ param_find("MPC_XY_VEL_MAX")   , 12.0f  , 12.0f  },
	{ param_find("MPC_Z_VEL_MAX_DN") , 1.0f   , 1.0f   },
	{ param_find("MPC_Z_VEL_MAX_UP") , 3.0f   , 3.0f   },

	{ param_find("MPC_XY_P")         , 0.95f  , 0.95f  },
	{ param_find("MPC_XY_TRAJ_P")    , 0.5f   , 0.5f   },
	{ param_find("MPC_Z_P")          , 1.0f   , 1.0f   },

	{ param_find("MPC_XY_VEL_P_ACC") , 1.8f   , 1.8f   },
	{ param_find("MPC_XY_VEL_I_ACC") , 0.4f   , 0.4f   },
	{ param_find("MPC_XY_VEL_D_ACC") , 0.2f   , 0.2f   },

	{ param_find("MPC_Z_VEL_P_ACC")  , 4.0f   , 4.0f   },
	{ param_find("MPC_Z_VEL_I_ACC")  , 2.0f   , 2.0f   },
	{ param_find("MPC_Z_VEL_D_ACC")  , 0.0f   , 0.0f   },

	// Multicopter Rate Control
	{ param_find("MC_PITCHRATE_P")   , 0.15f  , 0.15f  },
	{ param_find("MC_PITCHRATE_I")   , 0.2f   , 0.2f   },
	{ param_find("MC_PITCHRATE_D")   , 0.003f , 0.003f },
	{ param_find("MC_PITCHRATE_FF")  , 0.0f   , 0.0f   },
	{ param_find("MC_PR_INT_LIM")    , 0.30f  , 0.30f  },

	{ param_find("MC_ROLLRATE_P")    , 0.15f  , 0.15f  },
	{ param_find("MC_ROLLRATE_I")    , 0.2f   , 0.2f   },
	{ param_find("MC_ROLLRATE_D")    , 0.003f , 0.003f },
	{ param_find("MC_ROLLRATE_FF")   , 0.0f   , 0.0f   },
	{ param_find("MC_RR_INT_LIM")    , 0.30f  , 0.30f  },

	{ param_find("MC_YAWRATE_P")     , 0.2f   , 0.2f   },
	{ param_find("MC_YAWRATE_I")     , 0.1f   , 0.1f   },
	{ param_find("MC_YAWRATE_D")     , 0.0f   , 0.0f   },
	{ param_find("MC_YAWRATE_FF")    , 0.0f   , 0.0f   },
	{ param_find("MC_YR_INT_LIM")    , 0.30f  , 0.30f  }
};

const size_t aviata_shared_params_len = sizeof(aviata_shared_params) / sizeof(aviata_shared_params[0]);

// Receive pointer to multirotor mixer
void aviata_mixer_manager_init(MultirotorMixer* m);

// Set mixer to its original state (a standard hexacopter)
void aviata_mixer_manager_set_standalone();

// Configure the mixer once docked based on the docking slot and the missing drones.
// Also update PID values, flight controller orientation, etc.
void aviata_mixer_manager_finalize_docking(uint8_t docking_slot, uint8_t* missing_drones, uint8_t n_missing);

// Update the mixer according to which drones are missing
void aviata_mixer_manager_set_configuration(uint8_t* missing_drones, uint8_t n_missing);
