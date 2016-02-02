#ifndef IMU_INTERFACE_H_
#define IMU_INTERFACE_H_

#include <Ravelin/Pose3d.h>

namespace microstrain_3dm_gx3_35 {
 
    struct Pose {
      Ravelin::Quatd orientation;
      Ravelin::Origin3d position;
    };
  
  class IMUInterface {
    
  public:
    
    IMUInterface(int rate);
    ~IMUInterface();
    
    Pose getPose();
    void update();
    bool init();
    
    bool start();
    bool stop();
        
    bool reset_kalman_filter();
  protected:
    
    boost::shared_ptr<IMU> _imu;
    
   
    struct IMUState {
      double time;
      Ravelin::Origin3d
        linear_acceleration,
        linear_velocity,
        position,
        angular_velocity;
      Ravelin::Quatd orientation;
    } _imu_state;

    std::string _port;
    int _baud_rate;
    
    bool _started;
    bool _inited;
    
    double _linear_acceleration_stdev;
    double _orientation_stdev;
    double _angular_velocity_stdev;
    
    float _rate;
    
    bool _publish_pose;
    bool _publish_imu;
    bool _publish_gps;
    bool _publish_gps_as_odom;
    
    bool _gps_fix_available;
    bool isDataAvailable() const { return !_imu->isAHRSBufferEmpty(); }
  };
} // ns


#endif /* INTERFACE_H_ */

