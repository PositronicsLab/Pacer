#include <string>
#include <vector>

bool
       WALK                = false,   //  "Activate Walking?"
         TRACK_FOOTHOLDS     = false,//    EXPERIMENTAL -- "Locate and use footholds?"
       CONTROL_IDYN        = false,   //  "Activate IDYN?"
         WORKSPACE_IDYN      = false,//    EXPERIMENTAL -- "Activate WIDYN?"
         USE_LAST_CFS        = false,//    EXPERIMENTAL -- "Use last detected contact forces?"
       FRICTION_EST        = false,  //  EXPERIMENTAL
       TRUNK_STABILIZATION = false,   //  Balance Pitch (D) and Roll (PD) or robot base with compressive forces
       ERROR_FEEDBACK      = false,   //  "Use error-feedback control?"
         FEEDBACK_ACCEL      = false, //    "Apply error-feedback as accelerations?"
         JOINT_FEEDBACK      = false, //    "Apply error-feedback as forces?"
         WORKSPACE_FEEDBACK  = false,//    "Use error-feedback in workspace frame?"
       HOLONOMIC = false;

// -- LOCOMOTION OPTIONS --
double
       gait_time   = 0.0,//,"Gait Duration over one cycle."),
       step_height = 0.0,//,""),
       SIM_MU_COULOMB = 0,
       SIM_MU_VISCOSE = 0,
       SIM_PENALTY_KV = 0,
       SIM_PENALTY_KP = 0;

// Assign Gait to the locomotion controller
std::string
       gait_type   = ""; //,"Gait type [trot,walk,pace,bount,rgallop,tgallop]");

std::vector<double>
       duty_factor    = std::vector<double>(),
       goto_command   = std::vector<double>(),
       goto_point     = std::vector<double>(),
       patrol_points  = std::vector<double>(),
// INIT Vectors
       base_start     = std::vector<double>(),
       torque_limits  = std::vector<double>(),
       joints_start   = std::vector<double>(),
       other_gait     = std::vector<double>(),
       eefs_start     = std::vector<double>();

std::vector<std::string>
        joint_names = std::vector<std::string>(),
        eef_names = std::vector<std::string>();

// -- IDYN OPTIONS --
double STEP_SIZE = 0.005;
