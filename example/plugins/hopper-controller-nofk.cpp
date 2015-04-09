#include <Pacer/controller.h>
std::string plugin_namespace;

using boost::shared_ptr;
using std::string;
using std::vector;
using namespace Pacer;
using namespace Ravelin;

// these will be set to the indices of the joint DoFs
unsigned ROLL_JOINT;
unsigned PITCH_JOINT;
unsigned PISTON_JOINT;

// all possible hopper states
enum State { eFlight, eLoading, eCompression, eThrust, eUnloading };

// hopper state, set initially to flight
State hopper_state = eFlight;

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double time){

  const unsigned ROLL = 0, PITCH = 1, YAW = 2, VERT_DIM = 2;
  const double NEAR_MAX_PISTON_LEN = 0.97;
  const double COMPRESSION_PISTON_LEN = 0.95;
  const double THRUST_PISTON_LEN = 0.8;

  // TODO: look up what this gain does
  const double AKv = 1e-1;

  // get the linear velocity of the robot base
  // NOTE: you could get this using ctrl->get_data<VectorNd>
  Vector3d lvel = ctrl->get_data<Vector3d>("base.xd");

  // get roll/pitch/yaw
  Origin3d roll_pitch_yaw = ctrl->get_data<Origin3d>("roll_pitch_yaw");

  // get contacts for the foot
  vector<shared_ptr<const Robot::contact_t> > foot_contacts; 
  ctrl->get_link_contacts("FOOT", foot_contacts);
  const unsigned NUM_CONTACTS = foot_contacts.size();

  // get joint positions
  static vector<string>& joint_names = Utility::get_variable<vector<string> >("init.joint.id");
  VectorNd q(joint_names.size()), qd(joint_names.size());
  VectorNd q_des(joint_names.size()), qd_des(joint_names.size());
  for (unsigned i=0; i< joint_names.size(); i++)
  {
    // get the current position and velocity
    q[i] = ctrl->get_joint_value(joint_names[i], Robot::position, 0);
    qd[i] = ctrl->get_joint_value(joint_names[i], Robot::velocity, 0);

    // initialize the desired position and velocity to the current values
    // (we'll overwrite individual elements as necessary later on)
    q_des[i] = q[i];
    qd_des[i] = qd[i]; 
  }

  // setup joint mapping so that we can refer to joints using their natural
  // names
  std::map<string, unsigned> idx;
  for (unsigned i=0; i< joint_names.size(); i++)
    idx[joint_names[i]] = i;
  ROLL_JOINT = idx["ROLL_JOINT"];
  PITCH_JOINT = idx["PITCH_JOINT"];
  PISTON_JOINT = idx["PISTON_JOINT"];

  // setup u (directly applied forces)
  double u[3] = { 0, 0, 0 };

  // do controller transitions 
  if (hopper_state == eFlight && NUM_CONTACTS != 0 && lvel[VERT_DIM] < 0.0)
  {
    // foot is now contacting ground -> move to loading phase
    hopper_state = eLoading;
  }
  else if (hopper_state == eLoading && q[PISTON_JOINT] < COMPRESSION_PISTON_LEN)
  {
    // foot is sufficiently shortened; now in compression phase
    hopper_state = eCompression;
  }
  else if (hopper_state == eCompression && (q[PISTON_JOINT] < THRUST_PISTON_LEN || qd[PISTON_JOINT] < 0.0))
  {
    // foot is really shortened; add thrust
    hopper_state = eThrust;
    u[PISTON_JOINT] = 5.0;
  }
  else if (hopper_state == eThrust && q[PISTON_JOINT] > NEAR_MAX_PISTON_LEN)
  {
    // leg near full length
    hopper_state = eUnloading;
  }
  else if (hopper_state == eUnloading && NUM_CONTACTS == 0)
  {
    // foot not in contact
    hopper_state = eFlight;
  }

  // Perform controller action based on mode
  switch(hopper_state){
  case eLoading:
    //// STOP EXHAUSTING LEG & ZERO HIP TORQUE
    q_des[ROLL_JOINT] = q[ROLL_JOINT];
    q_des[PITCH_JOINT] = q[PITCH_JOINT];
    q_des[PISTON_JOINT] = q[PISTON_JOINT]; // try to keep same piston position
    qd_des[ROLL_JOINT] = 0;
    qd_des[PITCH_JOINT] = 0;
    qd_des[PISTON_JOINT] = qd[PISTON_JOINT]; // try to keep same piston velocity
    break;
  case eCompression:
    //// UPPER LEG CHAMBER SEALED & SERVO BODY ATTITUDE WITH HIP
    q_des[ROLL_JOINT] = q[ROLL_JOINT];     // we will not apply torques...
    q_des[PITCH_JOINT] = q[PITCH_JOINT];   // ...due to positional error...
    q_des[PISTON_JOINT] = q[PISTON_JOINT]; // ...to these joints
    qd_des[ROLL_JOINT] = AKv*roll_pitch_yaw[ROLL]/dt;
    qd_des[PITCH_JOINT] = AKv*roll_pitch_yaw[PITCH]/dt;
    qd_des[PISTON_JOINT] = 0; // start moving to zero velocity 
    break;
  case eThrust:
    //// PRESSURIZE LEG & SERVO BODY ATTITUDE WITH HIP
    q_des[ROLL_JOINT] = q[ROLL_JOINT];     // we will not apply torques...
    q_des[PITCH_JOINT] = q[PITCH_JOINT];   // ...due to positional error...
    q_des[PISTON_JOINT] = q[PISTON_JOINT]; // ...to these joints
    qd_des[ROLL_JOINT] = AKv*roll_pitch_yaw[ROLL]/dt;
    qd_des[PITCH_JOINT] = AKv*roll_pitch_yaw[PITCH]/dt;
    qd_des[PISTON_JOINT] = qd[PISTON_JOINT]; // force applied directly
    break;
  case eUnloading:
    q_des[ROLL_JOINT] = q[ROLL_JOINT];     // we will not apply torques...
    q_des[PITCH_JOINT] = q[PITCH_JOINT];   // ...due to positional error...
    q_des[PISTON_JOINT] = q[PISTON_JOINT]; // ...to these joints
    qd_des[ROLL_JOINT] = 0;                // we will not apply torques...
    qd_des[PITCH_JOINT] = 0;               // ...due to velocity error...
    qd_des[PISTON_JOINT] = 0;              // ...to these joints
    break;
  case eFlight:
    //// EXHAUST LEG TO LOW PRESSURE & POSITION LEG FOR LANDING

    // don't try to extend the foot position outward (just extend the piston
    // downward)
    q_des[ROLL_JOINT] = 0.0;
    q_des[PITCH_JOINT] = 0.0;
    q_des[PISTON_JOINT] = 1.0;   // set the piston to fully extended
    break;
  } 

  // setup the PD gains
  const double KP[3] = { 2e3, 2e3, 2e3 };
  const double KV[3] = { 5e2, 5e2, 5e2 };

  // the PID controller will apply torques using the desired commands 
  for (unsigned i=0; i< joint_names.size(); i++)
  {
    // setup tau
    if (u[i] == 0.0)
      u[i] = KP[i] * (q_des[i] - q[i]) + KV[i] * (qd_des[i] - qd[i]);
  
    // get the current position and velocity
    ctrl->set_joint_value(joint_names[i], Robot::load_goal, 0, u[i]);
  }
}

// calls *my* update function
void update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
  static int ITER = 0;
  int RTF = (int) Utility::get_variable<double>(plugin_namespace+"real-time-factor");
  if(ITER%RTF == 0)
    Update(ctrl,t);
}

extern "C" {
  void init(const boost::shared_ptr<Pacer::Controller> ctrl, const char* name){
    plugin_namespace = std::string(std::string(name)+".");

    // get the desired priority
    int priority = Utility::get_variable<double>(plugin_namespace+"priority");

    // set the hook to my plugin
    ctrl->add_plugin_update(priority,name,&update);

    // set the mode for the hopper 
    
  }
}

