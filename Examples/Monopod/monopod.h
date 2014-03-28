#ifndef CONTROL_H
#define CONTROL_H

#include <robot.h>

class Monopod : public Robot{
  public:
    Monopod();
    Monopod(Moby::RCArticulatedBodyPtr abrobot){
      abrobot_ = abrobot;
      init();
    }
    Ravelin::VectorNd& control(double dt,
                               const Ravelin::VectorNd& q,
                               const Ravelin::VectorNd& qd,
                               Ravelin::VectorNd& q_des,
                               Ravelin::VectorNd& qd_des,
                               Ravelin::VectorNd& u);

    void raibert_hopper(const Ravelin::Vector3d& xd_des,double dt,
                        Ravelin::VectorNd& q_des,
                        Ravelin::VectorNd& qd_des,
                        Ravelin::VectorNd& u);

    void init();

    enum Mode{
      LOADING = 0,
      // LEG SHORTENS ->
      // UPPER LEG CHAMBER SEALED & SERVO BODY ATTITUDE WITH HIP
      COMPRESSION,

      // LEG LENGTHENS ->
      // PRESSURIZE LEG & SERVO BODY ATTITUDE WITH HIP
      THRUST,
      // FOOT NOT IN CONTACT ->
      // EXHAUST LEG TO LOW PRESSURE & POSITION LEG FOR LANDING
      UNLOADING,
      FLIGHT
    } MODE;
};

#endif // CONTROL_H
