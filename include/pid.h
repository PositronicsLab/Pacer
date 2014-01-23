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
/// Controls the robot
static void control_PID(const Ravelin::VectorNd& q_des,const Ravelin::VectorNd& qd_des,
                 const Ravelin::VectorNd& q,    const Ravelin::VectorNd& qd,
                 const std::vector<std::string> joint_names,
                 std::map<std::string, Gains> gains, Ravelin::VectorNd& ufb)
{
  unsigned num_joints = joint_names.size();
  // clear and set motor torques
  for (unsigned i=0; i< num_joints; i++)
  {
    // get the two gains
    std::string joint_name = joint_names[i];
    const double KP = gains[joint_name].kp;
    const double KV = gains[joint_name].kv;
    const double KI = gains[joint_name].ki;

    // add feedback torque to joints
    double perr = q_des[i] - q[i];
    gains[joint_name].perr_sum += perr;
    double ierr = gains[joint_name].perr_sum;
    double derr = qd_des[i] - qd[i];

    ufb[i] = perr*KP + derr*KV + ierr*KI;
  }
}
#endif // PID_H
