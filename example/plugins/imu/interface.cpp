#include "driver.h"
#include "interface.h"

using namespace microstrain_3dm_gx3_35;
using namespace std;
using namespace boost;

/*
 * TODOs
 * add some services etc.
 */

#include "../plugin.h"

imuInterface::imuInterface(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);

  std::string port_ = ctrl->get_data<std::string>(plugin_namespace+".port");
  int baud_rate_ = ctrl->get_data<int>(plugin_namespace+".baud-rate");
  float declination_ = 3.8; // http://www.ngdc.noaa.gov/geomag-web/#declination
  float rate_  = ctrl->get_data<double>(plugin_namespace+".rate");;
  
  bool zero_height_ = true;
  
  float linear_acceleration_stdev_ = 0.098;
  float orientation_stdev_ = 0.035;
  float angular_velocity_stdev_ = 0.012;
  
  imu_.reset(new IMU((int)floor(rate_),microstrain_3dm_gx3_35::IMU::GX3_35));
  
  started_ = false;
  inited_ = false;
  
  gps_fix_available_ = false;
}

bool imuInterface::reset_kalman_filter() {
  
  fprintf(stdout,"Resetting KF.");
  
  if (!imu_->setToIdle()) fprintf(stderr,"%s",imu_->getLastError().c_str());
  if (!imu_->initKalmanFilter(declination_)) fprintf(stderr,"%s",imu_->getLastError().c_str());
  if (!imu_->resume()) fprintf(stderr,"%s",imu_->getLastError().c_str());
  
  return true;
  
}

bool imuInterface::init() {
  
  
  if (!imu_->openPort(port_,(unsigned int)baud_rate_)) {
    
    fprintf(stderr,"Can't open port.");
    return false;
    
  }
  
  started_ = false;
  
  fprintf(stdout,"Pinging device");
  imu_->setTimeout(posix_time::seconds(0.5));
  if (!imu_->ping()) {
    
    fprintf(stderr,"Pinging device");
    return false;
    
  }
  
  fprintf(stdout,"Setting to idle");
  if (!imu_->setToIdle()) {
    
    fprintf(stderr,"Setting to idle");
    return false;
    
  }
  
  fprintf(stdout,"Checking status");
  if (!imu_->devStatus()) {
    
    fprintf(stderr,"Checking status");
    return false;
    
  }
  
  fprintf(stdout,"Disabling all streams");
  if (!imu_->disAllStreams()) {
    
    fprintf(stderr,"Disabling all streams");
    return false;
    
  }
  
  fprintf(stdout,"Device self test");
  if (!imu_->selfTest()) {
    
    fprintf(stderr,"Device self test");
    return false;
  }
  
  fprintf(stdout,"Setting AHRS msg format");
  if (!imu_->setAHRSMsgFormat()) {
    
    fprintf(stderr,"Setting AHRS msg format");
    return false;
    
  }
  
  fprintf(stdout,"Setting GPS msg format");
  if (!imu_->setGPSMsgFormat()) {
    
    fprintf(stderr,"Setting GPS msg format");
    return false;
    
		}
  
  if(imu_->model_ == IMU::GX3_45)
  {
    fprintf(stdout,"Setting NAV msg format");
    if (!imu_->setNAVMsgFormat()) {
      
      fprintf(stderr,"Setting NAV msg format");
      return false;
      
    }
  }
  
  start();
  
  if(imu_->model_ == IMU::GX3_45)
  {
    fprintf(stdout,"KF initialization");
    if (!imu_->initKalmanFilter(declination_)) {
      
      fprintf(stderr,"KF initialization");
      return false;
      
    }
  }
  
  inited_ = true;
  return true;
  
}

bool imuInterface::start() {
  
  if (!imu_->resume()) {
    
    fprintf(stderr,"Resuming");
    return false;
    
		}
  
  started_ = true;
  return true;
  
}

bool imuInterface::stop() {
  
  if (!imu_->setToIdle()) {
    
    fprintf(stderr,"To idle");
    return false;
    
  }
  
  started_ = false;
  return true;
  
}

void imuInterface::update() {
  
  if (!imu_->isOpen()) {
    fprintf(stderr,"Port is not opened. Can't continue.");
    return;
  }
  
  double angular_velocity_covariance = angular_velocity_stdev_ * angular_velocity_stdev_;
  double orientation_covariance = orientation_stdev_ * orientation_stdev_;
  double linear_acceleration_covariance = linear_acceleration_stdev_ * linear_acceleration_stdev_;
  
  fprintf(stdout,"Start polling device.");
  
  int gps_msg_cnt = 0;
  
  if (publish_nav_odom_ || publish_nav_pose_ || publish_nav_fix_) {
    
    if (!imu_->pollNAV()) {
      
      fprintf(stderr,"NAV");
      
    }
    
    // TODO check nav filter status
  }
  
  if (publish_imu_ || publish_pose_) {
    
    if (!imu_->pollAHRS()) {
      
      fprintf(stderr,"AHRS");
      
    }
    
  }
  
  
  if (publish_nav_pose_) {
    
    fprintf(stdout,"Publishing NAV as Pose.");
    
    tnav n = imu_->getNAV();
    
    nav_pose.time = n.time;
    
    float yaw = n.est_y;
    
    yaw+=M_PI;
    if (yaw > M_PI) yaw-=2*M_PI;
    
    nav_pose.P.q = Ravelin::Quatd::rpy(-n.est_r, n.est_p, -yaw);
  }
  
  if (publish_imu_) {
    
    fprintf(stdout,"Publishing IMU data.");
    
    tahrs q = imu_->getAHRS();
    
    imu.time = q.time;
    
    imu.linear_acceleration[0] = -q.ax;
    imu.linear_acceleration[1] = q.ay;
    imu.linear_acceleration[2] = -q.az;
    
    imu.angular_velocity[0] = -q.gx;
    imu.angular_velocity[1] = q.gy;
    imu.angular_velocity[2] = -q.gz;
    
    float yaw = q.y;
    
    // TODO is this needed?
    yaw+=M_PI;
    if (yaw > M_PI) yaw-=2*M_PI;
    
    imu.orientation = Ravelin::Quatd::rpy(-q.r, q.p, -yaw);
  }
  
  if (publish_pose_) {
    
    fprintf(stdout,"Publishing IMU data as PoseStamped.");
    
    tahrs q = imu_->getAHRS();
    
    ps.time = (q.time);
    
    float yaw = q.y;
    yaw+=M_PI;
    if (yaw > M_PI) yaw-=2*M_PI;
    
    ps.orientation = Ravelin::Quatd::rpy(-q.r, q.p, -yaw);
  }
  
  {
    
    if (!imu_->pollGPS()) {
      
      fprintf(stderr,"GPS");
      
    }
    
    tgps g;
    g = imu_->getGPS();
    
    if (!g.lat_lon_valid) fprintf(stdout,"GPS fix not available.");
    
    if (g.lat_lon_valid && !gps_fix_available_) {
      
      fprintf(stdout,"GPS fix available.");
      gps_fix_available_ = true;
      
    }
    
    if (!g.lat_lon_valid && gps_fix_available_) {
      
      fprintf(stdout,"GPS fix lost.");
      gps_fix_available_ = false;
      
    }
    
    
    if (gps_fix_available_) {
      
      if (gps_msg_cnt++==6*rate_) {
        
        gps_msg_cnt = 0;
        
        if (!g.lat_lon_valid) fprintf(stdout,"LAT/LON not valid.");
        if (!g.hor_acc_valid) fprintf(stdout,"Horizontal accuracy not valid.");
        else fprintf(stdout,"GPS horizontal accuracy: %f",g.horizontal_accuracy);
        
      }
      
    }
    
  }
  
  if (publish_nav_fix_) {
    
    fprintf(stdout,"Publishing NAV as NavSatFix.");
    
    tnav n = imu_->getNAV();
    
    nav_fix.time = (n.time);
    
    nav_fix.latitude = n.est_latitude;
    nav_fix.longitude = n.est_longtitude;
    if (!zero_height_) nav_fix.altitude = n.est_height;
    else nav_fix.altitude = 0.0;
    
    if (n.est_llh_valid) nav_fix.fix_status = true;
    else  nav_fix.fix_status = false;
    
    if (n.est_pos_unc_valid) {
      nav_fix.covariance_status = true;
      nav_fix.position_covariance(0,0) = pow(n.est_north_pos_unc,2);
      nav_fix.position_covariance(1,1) = pow(n.est_east_pos_unc,2);
      nav_fix.position_covariance(2,2) = pow(n.est_down_pos_unc,2);
    } else {
      nav_fix.covariance_status = false;
    }
  }
}

imuInterface::~imuInterface() {
  
  imu_->closePort();
  
}

imuInterface * node;

void loop(){
  node->update();
}

void setup()
{
  node = new imuInterface();

  fprintf(stdout,"Initializing.");
  if (!node->init()) {
    fprintf(stderr,"Initialization failed. Please check logs.");
    throw std::runtime_error("Initialization of IMU failed.");
  }
  
  fprintf(stdout,"Initialization completed.");
}
