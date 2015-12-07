#ifndef INTERFACE_H_
#define INTERFACE_H_

#include <Ravelin/Pose3d.h>

namespace microstrain_3dm_gx3_35 {
  
  class imuInterface {
    
  public:
    
    imuInterface();
    ~imuInterface();
    
    void update();
    bool init();
    
    bool start();
    bool stop();
        
    bool reset_kalman_filter();
  protected:
    
    boost::shared_ptr<IMU> imu_;
    
    struct NavPose {
      double time;
      Ravelin::Pose3d P;
    } nav_pose;
    
    
    struct IMUState {
      double time;
      Ravelin::Origin3d
        linear_acceleration,
        angular_velocity;
      Ravelin::Quatd orientation;
    } imu;
    
    struct Pose {
      double time;
      Ravelin::Quatd orientation;
    } ps;
    
    struct NavFix {
      double time;
      double latitude,longitude,altitude;
      bool fix_status;
      Ravelin::Matrix3d position_covariance;
      bool covariance_status;

    } nav_fix;
    
    std::string port_;
    int baud_rate_;
    float declination_;
    
    bool started_;
    bool inited_;
    
    double linear_acceleration_stdev_;
    double orientation_stdev_;
    double angular_velocity_stdev_;
    
    float rate_;
    
    bool publish_pose_;
    bool publish_imu_;
    bool publish_gps_;
    bool publish_gps_as_odom_;
    
    bool publish_nav_odom_;
    bool publish_nav_pose_;
    bool publish_nav_fix_;
    
    bool zero_height_;
    
    bool gps_fix_available_;
    
    //bool nav_odom_rel_;
    
  private:
    
  };
  
  
  
} // ns


#endif /* INTERFACE_H_ */
