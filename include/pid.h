#ifndef PID_H
#define PID_H

#include <project_common.h>

struct Gains
{
  double perr_sum;
  double kp;
  double kv;
  double ki;
};

class PID{
public:
  PID();
/// Controls the robot
///
  static void control(const Ravelin::VectorNd& q_des,const Ravelin::VectorNd& qd_des,
                   const Ravelin::VectorNd& q,    const Ravelin::VectorNd& qd,
                   const std::vector<std::string> joint_names,
                   std::map<std::string, Gains> gains, Ravelin::VectorNd& ufb);
};
#endif // PID_H
