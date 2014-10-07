#include <project_common.h>
#include <utilities.h>
#include <pid.h>

Ravelin::Vector3d& Utility::quat2TaitBryanZ(const Ravelin::Quatd& q_, Ravelin::Vector3d& rpy){

  Ravelin::VectorNd q(4);
  q[0] = q_.w;
  q[1] = q_.x;
  q[2] = q_.y;
  q[3] = q_.z;

  // Singularity Condition 2(q1q3 + q0q2) == +/- 1
  assert(fabs(2*(q[1]*q[3] + q[0]*q[2])) < (1.0 - Moby::NEAR_ZERO) || fabs(2*(q[1]*q[3] + q[0]*q[2])) > (1.0 + Moby::NEAR_ZERO));

  // (3-2-1) z-y-x Tait-Bryan rotation
  rpy[0] = atan2((q[2]*q[3] + q[0]*q[1]),0.5-(q[1]*q[1] + q[2]*q[2]));
  rpy[1] = -asin(-2*(q[1]*q[3] + q[0]*q[2]));
  rpy[2] = atan2((q[1]*q[2] + q[0]*q[3]),0.5-(q[2]*q[2] + q[3]*q[3]));
  return rpy;
}

Ravelin::Vector3d& Utility::quat2TaitBryanX(const Ravelin::Quatd& q_, Ravelin::Vector3d& rpy){

  Ravelin::VectorNd q(4);
  q[0] = q_.w;
  q[1] = q_.x;
  q[2] = q_.y;
  q[3] = q_.z;

  // Singularity Condition 2(q1q3 + q0q2) == +/- 1
  assert(fabs(2*(q[2]*q[3] + q[0]*q[1])) < (1.0 - Moby::NEAR_ZERO) || fabs(2*(q[2]*q[3] + q[0]*q[1])) > (1.0 + Moby::NEAR_ZERO));

  // (3-2-1) z-y-x Tait-Bryan rotation
  rpy[0] = atan2((q[1]*q[3] + q[0]*q[2]),0.5-(q[3]*q[3] + q[2]*q[2]));
  rpy[1] = -asin(-2*(q[2]*q[3] + q[0]*q[1]));
  rpy[2] = atan2((q[1]*q[2] + q[0]*q[3]),0.5-(q[2]*q[2] + q[3]*q[3]));
  return rpy;
}


Ravelin::Vector3d& Utility::R2rpy(const Ravelin::Matrix3d& R, Ravelin::Vector3d& rpy){
  if(R(0,0) == 0 || R(2,2) == 0)
    return (rpy = Ravelin::Vector3d::zero());

  rpy[0] = atan2(R(1,0),R(0,0));
  rpy[1] = atan2(-R(2,0),sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2)));
  rpy[2] = atan2(R(2,1),R(2,2));
  return rpy;
}

Ravelin::Vector3d& Utility::quat2rpy(const Ravelin::Quatd& q, Ravelin::Vector3d& rpy){

  double rotateXa0 = 2.0*(q.y*q.z + q.w*q.x);
  double rotateXa1 = q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;
  double rotateX = 0.0;
  if (rotateXa0 != 0.0 && rotateXa1 != 0.0)
    rotateX = atan2(rotateXa0, rotateXa1);

  double rotateYa0 = -2.0*(q.x*q.z - q.w*q.y);
  double rotateY = 0.0;
  if( rotateYa0 >= 1.0 )
    rotateY = M_PI/2.0;
  else if( rotateYa0 <= -1.0 )
    rotateY = -M_PI/2.0;
  else rotateY = asin(rotateYa0);

  double rotateZa0 = 2.0*(q.x*q.y + q.w*q.z);
  double rotateZa1 = q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z;
  double rotateZ = 0.0;
  if (rotateZa0 != 0.0 && rotateZa1 != 0.0)
    rotateZ = atan2(rotateZa0, rotateZa1);

  return (rpy = Ravelin::Vector3d(rotateX,rotateY,rotateZ));
}

Ravelin::Quatd& Utility::aa2quat(const Ravelin::VectorNd& aa,Ravelin::Quatd& q){
  q[0] = cos(aa[3]*0.5);
  q[1] = sin(aa[3]*0.5)*cos(aa[0]);
  q[2] = sin(aa[3]*0.5)*cos(aa[1]);
  q[3] = sin(aa[3]*0.5)*cos(aa[2]);
  return q;
}
Ravelin::Matrix3d& Utility::Rz(double x,Ravelin::Matrix3d& R){
  R = Ravelin::Matrix3d( cos(x), sin(x), 0,
                        -sin(x), cos(x), 0,
                              0,      0, 1);
  return R;
}

Ravelin::Matrix3d& Utility::Ry(double x,Ravelin::Matrix3d& R){
  R = Ravelin::Matrix3d( cos(x), 0,-sin(x),
                              0, 1,      0,
                         sin(x), 0, cos(x));
  return R;
}

Ravelin::Matrix3d& Utility::Rx(double x,Ravelin::Matrix3d& R){
  R = Ravelin::Matrix3d(1,      0,      0,
                        0, cos(x), sin(x),
                        0,-sin(x), cos(x));
  return R;
}

/// Calculates The null pace for matrix M and places it in Vk
/// returns the number of columns in Vk
unsigned Utility::kernal( Ravelin::MatrixNd& M,Ravelin::MatrixNd& null_M){
  unsigned size_null_space = 0;
  Ravelin::MatrixNd U,V;
  Ravelin::VectorNd S;

  // SVD decomp to retrieve nullspace of Z'AZ
  LA_.svd(M,U,S,V);
  if(S.rows() != 0){
    // Calculate the tolerance for ruling that a singular value is supposed to be zero
    double ZERO_TOL = std::numeric_limits<double>::epsilon() * M.rows() * S[0];
    // Count Zero singular values
    for(int i = S.rows()-1;i>=0;i--,size_null_space++)
      if(S[i] > ZERO_TOL)
        break;
    // get the nullspace
    null_M.set_zero(V.rows(),size_null_space);
    V.get_sub_mat(0,V.rows(),V.columns()-size_null_space,V.columns(),null_M);
  }
  return size_null_space;
}

void Utility::check_finite(Ravelin::VectorNd& v){
  for(int i=0;i<v.rows();i++)
    if(!std::isfinite(v[i]))
      v[i] = 0;
}

double Utility::distance_from_plane(const Ravelin::Vector3d& normal,const Ravelin::Vector3d& point, const Ravelin::Vector3d& x){
  Ravelin::Vector3d v;
  v = x - point;
  return normal.dot(v);
}

//double pow(double x,int n){
//  if(x <= std::numeric_limits<double>::epsilon()) return 0;
//  double temp_x = x;
//  if(n == 0) x = 1;
//  if(n<0) n = abs(n);
//  temp_x = 1/temp_x;
//  x = temp_x;
//  for(int i=1;i<n;i++)
//    x *= temp_x;
//  return x;
//}

/// 4 Control Point Bezier curve EEF calculation
// Calculate the next bezier point.

Ravelin::Vector3d Utility::logistic(double x,double a, double b){
  return Ravelin::Vector3d( 1/(1+exp(-a*(x-b))),
                           (a*exp(a*(b+x)))/((exp(a*b)+exp(a*x))*(exp(a*b)+exp(a*x))),
                           (a*a*exp(a*(b+x))*(exp(a*b)-exp(a*x)))/((exp(a*b)+exp(a*x))*(exp(a*b)+exp(a*x))*(exp(a*b)+exp(a*x)))
                            );
}

Ravelin::Vector3d Utility::slerp( const Ravelin::Vector3d& v0,const Ravelin::Vector3d& v1,double t){
  double a = v0.dot(v1);
  return (v0*sin((1-t)*a)/sin(a) + v1*sin(t*a)/sin(a));
}

//Ravelin::Quatd slerp( const Ravelin::Quatd& q0,const Ravelin::Quatd& q1,double t){
////  double a = q0.dot(q1);
////  return (q0*sin((1-t)*a)/sin(a) + q1*sin(t*a)/sin(a));
//  pow(q1.dot(q0.inverse()),t)*q0;
//  return q0.normalize();
//}

Ravelin::Vector3d Utility::lerp( const Ravelin::Vector3d& v0,const Ravelin::Vector3d& v1,double t){
  return (v0*(1-t) + v1*t);
}


void PID::control(const Ravelin::VectorNd& q_des,const Ravelin::VectorNd& qd_des,
                 const Ravelin::VectorNd& q,    const Ravelin::VectorNd& qd,
                 const std::vector<std::string> joint_names,
                 std::map<std::string, Gains> gains, Ravelin::VectorNd& ufb){
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

    ufb[i] += perr*KP + derr*KV + ierr*KI;
  }
}
