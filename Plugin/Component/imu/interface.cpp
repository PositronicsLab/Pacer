#include "driver.h"
#include "interface.h"

using namespace microstrain_3dm_gx3_35;
using namespace std;
using namespace boost;

#include "../plugin.h"

IMUInterface::IMUInterface(int rate){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);

   // set some reasonable values 
  _port = ctrl->get_data<std::string>(plugin_namespace+".port"); 
  _baud_rate = 115200;
  _rate  = rate; 
  assert(rate <= 1000); 
 
  _linear_acceleration_stdev = 0.098;
  _orientation_stdev = 0.035;
  _angular_velocity_stdev = 0.012;

   // create the pointer 
  _imu = boost::shared_ptr<IMU>(new IMU((int)floor(_rate), microstrain_3dm_gx3_35::IMU::GX3_35));

  // indicate that IMU is neither started nor inited  
  _started = false;
  _inited = false;
  
  _gps_fix_available = false;
}

/// Initializes the IMU
bool IMUInterface::init() 
{  
  if (!_imu->openPort(_port,(unsigned int)_baud_rate)) 
  {  
    std::cerr << "Can't open port." << std::endl;
    return false; 
  }
  
  _started = false;
 
  std::cout << "Disabling all streams" << std::endl;
  if (!_imu->disAllStreams()) 
  {
    std::cerr << "Disabling all streams FAILED" << std::endl;
    return false;
  }
  
  std::cout << "Pinging device" << std::endl;
  _imu->setTimeout(posix_time::seconds(0.5));

  if (!_imu->ping()) 
  {
    std::cerr << "Pinging device FAILED" << std::endl;
    return false;
  }
  
  std::cout << "Setting to idle" << std::endl;
  if (!_imu->setToIdle()) 
  {
    std::cerr << "Setting to idle FAILED" << std::endl;
    return false;
  }
  
  std::cout << "Checking status" << std::endl;
  if (!_imu->devStatus()) 
  {
    std::cerr << "Checking status FAILED" << std::endl;
    return false;
  }
  
 
  std::cout << "Device self test" << std::endl;
  if (!_imu->selfTest()) 
  {
    std::cerr << "Device self test FAILED" << std::endl;
    return false;
  }

  std::cout << "Setting dynamics mode" << std::endl;
  if (!_imu->setDynamicsMode()) 
  {
    std::cerr << "Setting dynamics mode FAILED" << std::endl;
    return false;
  }

  std::cout << "Setting signal conditioning" << std::endl;
  if (!_imu->setAHRSSignalCond()) 
  {
    std::cerr << "Setting AHRS signal conditioning FAILED" << std::endl;
    return false;
  }

  std::cout << "Setting AHRS msg format" << std::endl;
  if (!_imu->setAHRSMsgFormat()) 
  {
    std::cerr << "Setting AHRS msg format FAILED" << std::endl;
    return false;
  }

  std::cout << "Setting GPS msg format" << std::endl;
  if (!_imu->setGPSMsgFormat()) 
  {
    std::cerr << "Setting GPS msg format FAILED" << std::endl;
    return false;
  }
 
  // start the IMU 
  start();
  
  _inited = true;
  return true;
}

bool IMUInterface::start() {
  
  if (!_imu->resume()) 
  {
    std::cerr << "Resuming" << std::endl;
    return false;
  }
  
  // reset IMU state
  for (int i=0; i< 3; i++)
  {
    _imu_state.linear_velocity[i] = 0.0f;
    _imu_state.position[i] = 0.0f;
  }

  // enable AHRS stream
  if (!_imu->enableAHRSStream()) 
  {
    std::cerr << "Enabling AHRS streaming" << std::endl;
    return false;
  }

  // begin publishing IMU and pose
  _publish_imu = _publish_pose = true;

  _started = true;
  return true;
}

bool IMUInterface::stop() {
  
  if (!_imu->setToIdle()) 
  {
    std::cerr << "To idle" << std::endl;
    return false;
  }
  
  _started = false;
  return true;
  
}

/*
void IMUInterface::update() {

  const double DT = 1;//1.0/_rate;
  const float G = 9.80655;

  if (!_imu->isOpen()) 
  {
    std::cerr << "Port is not opened. Can't continue." << std::endl;
    return;
  }
  
  int gps_msg_cnt = 0;
  
  if (_publish_imu || _publish_pose) {
    
    if (!_imu->pollAHRS()) 
      std::cerr << "AHRS polling failed!" << std::endl;
  }
  
  if (_publish_imu) {
    
    const tahrs& q = _imu->getAHRS();
    _imu_state.time = q.time;
    
    _imu_state.linear_acceleration[0] = q.ax;
    _imu_state.linear_acceleration[1] = q.ay;
    _imu_state.linear_acceleration[2] = q.az;
    
    _imu_state.angular_velocity[0] = -q.gx;
    _imu_state.angular_velocity[1] = q.gy;
    _imu_state.angular_velocity[2] = -q.gz;

    // setup the orientation
    float yaw = q.y;
    
    // TODO is this needed?
    yaw+=M_PI;
    if (yaw > M_PI) yaw-=2*M_PI;

    // set the orientation    
    _imu_state.orientation = Ravelin::Quatd::rpy(-q.r, q.p, -yaw);

    // update the acceleration to the global frame
    _imu_state.linear_acceleration = _imu_state.orientation * _imu_state.linear_acceleration;

    // unbias linear acceleration
    _imu_state.linear_acceleration[2] += G;    

    // determine velocity
    _imu_state.linear_velocity[0] = q.ax*DT;
    _imu_state.linear_velocity[1] = q.ay*DT;
    _imu_state.linear_velocity[2] = q.az*DT;
    _imu_state.linear_velocity[0] = q.vx;
    _imu_state.linear_velocity[1] = q.vy;
    _imu_state.linear_velocity[2] = q.vz;
    _imu_state.linear_velocity = _imu_state.orientation * _imu_state.linear_velocity;

    // determine position (using OSG)
    _imu_state.position[0] += _imu_state.linear_velocity[0]*DT;
    _imu_state.position[1] += _imu_state.linear_velocity[1]*DT;
    _imu_state.position[2] += _imu_state.linear_velocity[2]*DT;
    std::cout << _imu_state.position << std::endl;
  }
  
  if (_publish_pose) {
    
    const tahrs& q = _imu->getAHRS();
    
    _ps.time = (q.time);
    
    float yaw = q.y;
    yaw+=M_PI;
    if (yaw > M_PI) yaw-=2*M_PI;
    
    _ps.orientation = Ravelin::Quatd::rpy(-q.r, q.p, -yaw);

    // setup position
    _ps.position = _imu_state.position; 
  }
  
  {
    
    if (!_imu->pollGPS()) 
      std::cerr << "GPS" << std::endl;
    
    // get GPS data
    const tgps& g = _imu->getGPS();
    
    if (g.lat_lon_valid && !_gps_fix_available) 
    {
      std::cout << "GPS fix available." << std::endl;
      _gps_fix_available = true;
    }
    
    if (!g.lat_lon_valid && _gps_fix_available) 
    {
      std::cout << "GPS fix lost." << std::endl;
      _gps_fix_available = false;
    }
    
    if (_gps_fix_available) 
    {
      if (gps_msg_cnt++==6*_rate) 
      {
        gps_msg_cnt = 0;
        
        if (!g.lat_lon_valid) std::cout << "LAT/LON not valid." << std::endl;
        if (!g.hor_acc_valid) std::cout << "Horizontal accuracy not valid." << std::endl;
        else 
          std::cout << "GPS horizontal accuracy: " << g.horizontal_accuracy << std::endl;
      }
    }
  }
}
*/

IMUInterface::~IMUInterface() {
  _imu->closePort();
}

/// TODO: enable this for testing calcFletcher(.) and checkFletcher(.)
/*
int main(int argc, char* argv[])
{
  std::vector<char> msg;

  // setup the message
  msg.push_back(2);
  msg.push_back(17);
  msg.push_back(52);
  msg.push_back(9);
  msg.push_back(121);
  msg.push_back(42);
  msg.push_back(110);
  msg.push_back(12);
  msg.push_back(62);
  msg.push_back(36);

  IMU::calcFletcher(msg);
  std::cout << "Fletcher checksum: " << ((int) msg[msg.size()-2]) << " " << ((int) msg.back()) << std::endl;

  // construct an example return packet (protocol documentation PDF, p. 13)
  msg.clear();
  msg.push_back(0x75);
  msg.push_back(0x65);
  msg.push_back(0x01);
  msg.push_back(0x04);
  msg.push_back(0x04);
  msg.push_back(0xF1);
  msg.push_back(0x01);
  msg.push_back(0x00);
  msg.push_back(0xD5);
  msg.push_back(0x6A);
  std::cout << "Fletcher checks out? " << IMU::checkFletcher(msg) << std::endl; 
}
*/

/// Gets the pose
Pose IMUInterface::getPose()
{
  Pose p;
  p.position = _imu->get_position();
  p.orientation = _imu->get_orientation(); 
  return p;
}

/// TODO: do something here... 
void IMUInterface::update()
{
  // get IMU data if it is available 
  Pose ps = getPose();
}


IMUInterface *node;

Ravelin::Transform3d imu_transform;
// called by the plugin
void loop(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
 // node->update();
  Pose ps = node->getPose();
  Ravelin::Pose3d imu_pose(ps.orientation,ps.position,Pacer::GLOBAL);
  
  Ravelin::Pose3d robot_pose = imu_transform.inverse_transform(imu_pose);
  
  static Ravelin::Origin3d p1 = robot_pose.x,p2;
  static Ravelin::Matrix3d R1 = Ravelin::Matrix3d(robot_pose.q), R2;
  
  p2 = robot_pose.x;
  R2 = Ravelin::Matrix3d(robot_pose.q);
  
  {
    Ravelin::Origin3d dp = p2 - p1;
    Ravelin::Matrix3d dR = R2 - R1;
    Ravelin::Origin3d dr(dR(2,1),dR(0,2),dR(1,0));
    Ravelin::VectorNd control = Ravelin::VectorNd::zero(6);
    control.segment(0,3) = dp * 10000.0;
    control.segment(3,6) = dr * 500.0;
//    ctrl->set_data<Ravelin::VectorNd>("base-control",control);
  }
  {  
    Ravelin::VectorNd state = Ravelin::VectorNd::zero(7);
    //state.segment(0,3) = p2;
    state[3] = robot_pose.q.x;
    state[4] = robot_pose.q.y;
    state[5] = robot_pose.q.z;
    state[6] = robot_pose.q.w;
  //  ctrl->set_data<Ravelin::VectorNd>("base-state",state);
    bool apply_state = false;
    if(apply_state){
    std::cerr << t << " " << state << std::endl;
    } else {
      ctrl->set_base_value(Pacer::Robot::position,state);
    }
  }
  p1 = p2;
  R1 = R2;
  
}

// called by the plugin
void setup()
{
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  static std::vector<double> pose_vec(6);
  ctrl->get_data<std::vector<double> >(plugin_namespace+".pose",pose_vec);
  imu_transform = Ravelin::Transform3d
     (Ravelin::Quatd::rpy(pose_vec[3],pose_vec[4],pose_vec[5]),
      Ravelin::Origin3d(pose_vec[0],pose_vec[1],pose_vec[2]));


  // setup the polling frequency (Hz)
  const int UPDATE_FREQ =  ctrl->get_data<int>(plugin_namespace+".rate"); 

  // create the IMU interface
  node = new IMUInterface(UPDATE_FREQ);

  fprintf(stdout,"Initializing.");
  if (!node->init()) {
    fprintf(stderr,"Initialization failed. Please check logs.");
    throw std::runtime_error("Initialization of IMU failed.");
  }
  
  fprintf(stdout,"Initialization completed.");
}

