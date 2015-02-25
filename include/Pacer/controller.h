/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#ifndef CONTROL_H
#define CONTROL_H

#include <Pacer/robot.h>
#include <CVars/CVar.h>
#include <Pacer/Module.h>
#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <cassert>

namespace Pacer{
class Controller;

typedef void (*init_t)(boost::shared_ptr<Controller>, const char*);
typedef void (*update_t)(boost::shared_ptr<Controller>, double);

class Controller : public Robot, public boost::enable_shared_from_this<Controller>{
  public:
    boost::shared_ptr<Controller> ptr()
    {
      return shared_from_this();
    }
    /**
     * @brief Controller constructor
     * @see Robot()
     */
    Controller();
    ~Controller();

    bool close_plugins();
    bool init_plugins();
    bool update_plugins(double t);

    void control(double dt,
                               const Ravelin::VectorNd& generalized_q,
                               const Ravelin::VectorNd& generalized_qd,
                               const Ravelin::VectorNd& generalized_qdd,
                               const Ravelin::VectorNd& generalized_fext,
                               Ravelin::VectorNd& q_des,
                               Ravelin::VectorNd& qd_des,
                               Ravelin::VectorNd& qdd_des,
                               Ravelin::VectorNd& u);

  private:
    std::vector<update_t> UPDATE;
    std::vector<init_t>   INIT;
    
    void trajectory_ik(const std::vector<Ravelin::Vector3d>& foot_pos,
                       const std::vector<Ravelin::Vector3d>& foot_vel,
                       const std::vector<Ravelin::Vector3d>& foot_acc,
                       const Ravelin::VectorNd& q, 
                       Ravelin::VectorNd& q_des,
                       Ravelin::VectorNd& qd_des,
                       Ravelin::VectorNd& qdd_des);
};
}
#endif // CONTROL_H
