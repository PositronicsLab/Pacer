#ifndef PID_H
#define PID_H

#include <project_common.h>

struct Gains
{
  double kp;
  double kv;
  double ki;
};

// map of gains
std::map<std::string, Gains> gains;

// integration error
std::map<std::string, double> perr_sum;

template <typename T, typename U>
const U& get(const std::map<T, U>& m, const T& key)
{
  typename std::map<T, U>::const_iterator i = m.find(key);
  if (i == m.end())
    throw std::runtime_error("Unexpectedly couldn't find key in map!");
  return i->second;
}

/// Controls the robot
void control_PID(const Ravelin::VectorNd& q_des,const Ravelin::VectorNd& qd_des,
                 const Ravelin::VectorNd& q,    const Ravelin::VectorNd& qd,
                 const std::vector<std::string> joint_names,
                 const std::map<std::string, Gains> gains,
                 double time, Ravelin::VectorNd& ufb)
{
  unsigned num_joints = joint_names.size();
  // clear and set motor torques
  for (unsigned i=0; i< num_joints; i++)
  {
    // get the two gains
    std::string joint_name = joint_names[i];
    const double KP = get(gains,joint_name).kp;
    const double KV = get(gains,joint_name).kv;
    const double KI = get(gains,joint_name).ki;

    // add feedback torque to joints
    double perr = q_des[i] - q[i];
    perr_sum[joint_name] += perr;
    double ierr = perr_sum[joint_name];
    double derr = qd_des[i] - qd[i];

    ufb[i] = perr*KP + derr*KV + ierr*KI;
  }
}
#endif // PID_H
