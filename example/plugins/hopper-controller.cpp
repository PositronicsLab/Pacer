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

enum State { eFlight, eLoading, eCompression, eThrust, eUnloading };

// hopper state
State hopper_state = eFlight;

// hopper IK
void footIK(const Vector3d& foot_pos, Origin3d& joint_pos){
  // x = z
  // y = y
  // z = -x

  //r = sqrt(x^2 + y^2 + z^2)
  joint_pos[PISTON_JOINT] = std::sqrt(foot_pos[0]*foot_pos[0] + foot_pos[1]*foot_pos[1] + foot_pos[2]*foot_pos[2]);
  
  //theta = acos(x/r)
  joint_pos[PITCH_JOINT] = std::acos(foot_pos[0]/joint_pos[PISTON_JOINT]) - M_PI_2;
  
  // Phi  = atan(y/z)
  joint_pos[ROLL_JOINT] = std::atan2(foot_pos[1],-foot_pos[2]);
}

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double time){

  const unsigned ROLL = 0, PITCH = 1, YAW = 2, VERT_DIM = 2;
  const double NEAR_MAX_PISTON_LEN = 0.97;
  const double COMPRESSION_PISTON_LEN = 0.95;
  const double THRUST_PISTON_LEN = 0.8;

  // define the global frame (null pointer)
  boost::shared_ptr<Ravelin::Pose3d> GLOBAL;

  // set the down vector
  Ravelin::Vector3d DOWN(0.0,0.0,-1.0,GLOBAL);

  // flight phase start time
  static double t = time;

  // set last t to (hopefully) some reasonable value, update last_t, compute dt
  static double last_t = 1e-4;
  double dt = time - last_t;
  last_t = time;

  // set AKv
  const double AKv = 2.5e-1;

  // robot "publishes" its base data
  shared_ptr<Pose3d> base_frame(new Pose3d(ctrl->get_data<Pose3d>("base_link_frame"))); 

  // get the linear velocity of the robot base
  // NOTE: you could get this using ctrl->get_data<VectorNd>
  Vector3d lvel = ctrl->get_data<Vector3d>("base.xd");

  // get roll/pitch/yaw
  Origin3d roll_pitch_yaw = ctrl->get_data<Origin3d>("roll_pitch_yaw");
  OUT_LOG(logERROR) << "roll/pitch/yaw " << roll_pitch_yaw;

  // check whether the value is there
  Ravelin::Origin3d xd_des;
  if (!ctrl->get_data<Ravelin::Origin3d>("SE2_command",xd_des))
     xd_des = Ravelin::Origin3d(0,0,0);

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

  // setup joint mapping
  std::map<string, unsigned> idx;
  for (unsigned i=0; i< joint_names.size(); i++)
    idx[joint_names[i]] = i;
  ROLL_JOINT = idx["ROLL_JOINT"];
  PITCH_JOINT = idx["PITCH_JOINT"];
  PISTON_JOINT = idx["PISTON_JOINT"];

  // setup u (directly applied forces)
  double u[3] = { 0, 0, 0 };

  // do controller transitions 
  OUT_LOG(logERROR) << "NC = " << NUM_CONTACTS;
  OUT_LOG(logERROR) << "z_vel = " << lvel[VERT_DIM];

  if (hopper_state == eFlight && NUM_CONTACTS != 0 && lvel[VERT_DIM] < 0.0)
  {
    OUT_LOG(logERROR) << "FLIGHT ==> LOADING PHASE";
    // foot is now contacting ground -> move to loading phase
    hopper_state = eLoading;
  }
  else if (hopper_state == eLoading && q[PISTON_JOINT] < COMPRESSION_PISTON_LEN)
  {
    OUT_LOG(logERROR) << "LOADING ==> COMPRESSION PHASE";
    // foot is sufficiently shortened; now in compression phase
    hopper_state = eCompression;
//    Vz = ?;
  }
  else if (hopper_state == eCompression && (q[PISTON_JOINT] < THRUST_PISTON_LEN || qd[PISTON_JOINT] < 0.0))
  {
    OUT_LOG(logERROR) << "COMPRESSION ==> THRUST PHASE";
    // foot is really shortened; add thrust
    hopper_state = eThrust;
  }
  else if (hopper_state == eThrust && q[PISTON_JOINT] > NEAR_MAX_PISTON_LEN) //(lvel.norm() >= -Vz) || )
  {
    OUT_LOG(logERROR) << "THRUST ==> UNLOADING PHASE";
    // leg near full length
    hopper_state = eUnloading;
  }
  else if (hopper_state == eUnloading && NUM_CONTACTS == 0)
  {
    OUT_LOG(logERROR) << "UNLOADING ==> FLIGHT PHASE";
    // foot not in contact
    hopper_state = eFlight;

    // reset t
    t = time; 
  }

  // get the linear velocity in the global frame
  const Vector3d lvel0 = Pose3d::transform_vector(GLOBAL, lvel);

  // if the foot hops along these axes:
  // speed will change and heading will remain unaffected: neutral_heading_locus
  // heading will change and speed will remain unaffected: neutral_speed_locus
  Vector3d neutral_heading_locus;
  Vector3d neutral_speed_locus = Vector3d::cross(lvel0,DOWN);
  if(neutral_speed_locus.norm() > std::sqrt(std::numeric_limits<double>::epsilon()))
  {
    neutral_speed_locus.normalize();
    neutral_heading_locus = Vector3d::cross(DOWN,neutral_speed_locus);

    // log
    OUTLOG(neutral_speed_locus,"neutral_speed_locus", logDEBUG1);
    OUTLOG(neutral_heading_locus,"neutral_heading_locus", logDEBUG1);
  }
 
  // Params of hop
  double duration_of_stance = 0.4,
         k_xd = 5e-2, // useless constant
         duration_of_swing = 0.5;

  // project the base frame to the ground plane 
  Vector3d neutral_foot_position = Vector3d(base_frame->x[0],base_frame->x[1],0,GLOBAL) + lvel0*duration_of_stance/2.0; // neutral_foot_position
  neutral_foot_position[VERT_DIM] = 0;

  // Perform controller action based on mode
  switch(hopper_state){
  case eLoading:
    OUT_LOG(logERROR) << "LOADING PHASE";
    //// STOP EXHAUSTING LEG & ZERO HIP TORQUE
    qd_des[ROLL_JOINT] = 0;
    qd_des[PITCH_JOINT] = 0;
    break;
  case eCompression:
    OUT_LOG(logERROR) << "COMPRESSION PHASE";
    //// UPPER LEG CHAMBER SEALED & SERVO BODY ATTITUDE WITH HIP
    qd_des[ROLL_JOINT] = 0.0;
    qd_des[PITCH_JOINT] = 0.0;
    qd_des[PISTON_JOINT] = 0; // NOTE: this is not used
    break;
  case eThrust:
    OUT_LOG(logERROR) << "THRUST PHASE";
    //// PRESSURIZE LEG & SERVO BODY ATTITUDE WITH HIP
    qd_des[ROLL_JOINT] = 0.0;
    qd_des[PITCH_JOINT] = 0.0;
    qd_des[PISTON_JOINT] = .2; 
    q_des[PISTON_JOINT] = qd_des[PISTON_JOINT]*dt;
    break;
  case eUnloading:
    OUT_LOG(logERROR) << "UNLOADING PHASE";
    qd_des[ROLL_JOINT] = 0;
    qd_des[PITCH_JOINT] = 0;
    qd_des[PISTON_JOINT] = 0;
    break;
  case eFlight:
    OUT_LOG(logERROR) << "FLIGHT PHASE";
    //// EXHAUST LEG TO LOW PRESSURE & POSITION LEG FOR LANDING

    // get the forward foot position from neutral
    Ravelin::Vector3d forward_foot_position = neutral_foot_position +
                                              k_xd*(lvel0 - xd_des);
    forward_foot_position[VERT_DIM] = neutral_foot_position[VERT_DIM];
    // get the desired foot position 
    Vector3d foot_pos = base_frame->inverse_transform_point(forward_foot_position);

    OUTLOG(foot_pos,"foot_pos", logDEBUG1);
    Utility::visualize.push_back( Pacer::VisualizablePtr( new Pacer::Ray(forward_foot_position,Vector3d(base_frame->x.data()),   Ravelin::Vector3d(0,1,0),0.1)));
    // do IK to get the joint angle for the desired foot position
    Origin3d joint_pos;
    footIK(foot_pos,joint_pos);
    // set the piston to fully extended in direction of goal point
    joint_pos[PISTON_JOINT] = 1.0;   

    // log foot position 
    OUTLOG(joint_pos,"joint_pos", logDEBUG1);

    for(int i=0;i<joint_names.size();i++)
    {
      q_des[i] = joint_pos[i];
      qd_des[i] = 0.0;
    }

/*
    // get elapsed t since start of flight phase
    double t_elapsed = time - t;

    // TODO: fix this?
    for(int i=0;i<joint_names.size();i++){
      // if movement has gone beyond duration, set goal position to endpoint 
      if( t< duration_of_swing)
        q_des[i] = (t_elapsed/duration_of_swing)*joint_pos[i] + (1.0 - t_elapsed/duration_of_swing)*q_des[i];
      else
        q_des[i] = joint_pos[i];
      qd_des[i] = 0;
    }
*/
    break;
  } 

  // setup the PD gains for on-ground control
  const double KP[3] = { 2e1, 2e1, 2e1 };
  const double KV[3] = { 5e0, 5e0, 5e0 };
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
  ITER += 1;
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

