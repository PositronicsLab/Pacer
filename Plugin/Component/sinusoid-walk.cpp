/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <Pacer/utilities.h>

#include <time.h>

#include "plugin.h"
using namespace Ravelin;
/// Gets the current time (as a floating-point number)
double get_current_time()
{
  const double MICROSEC = 1.0/1000000;
  timeval t;
  gettimeofday(&t, NULL);
  return (double) t.tv_sec + (double) t.tv_usec * MICROSEC;
}

void loop(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  
  double f = ctrl->get_data<double>(plugin_namespace+".frequency");
  double a = ctrl->get_data<double>(plugin_namespace+".amplitude");

  double time = t;
  
  std::map<std::string, std::vector<double> > q_goal = ctrl->make_id_value_map< double >();
  static std::map<std::string, std::vector<double> > q_init;
  if(q_init.empty())
    ctrl->get_joint_value(Pacer::Controller::position,q_init);
  
  std::map<std::string, std::vector<double> > qd_goal = ctrl->make_id_value_map< double >();
  std::map<std::string, std::vector<double> > qdd_goal = ctrl->make_id_value_map< double >();

  double s1 = sin( time * f ) * M_PI * a;  
  double c1 = cos( time * f ) * M_PI * a;

  double c0 = cos( time * f - M_PI_2) * M_PI * a;
  double s0 = sin( time * f - M_PI_2 ) * M_PI * a;

  // Hip Coronal
  q_goal  ["LF_X_1"][0]     = q_init["LF_X_1"][0];// +           s1;
  qd_goal ["LF_X_1"][0]     = 0;//                    f *      c1;
  qdd_goal["LF_X_1"][0]     = 0;//                    f * f * -s1;
  
  q_goal  ["RF_X_1"][0]     = q_init["RF_X_1"][0];// +           s1;
  qd_goal ["RF_X_1"][0]     = 0;//                    f *      c1;
  qdd_goal["RF_X_1"][0]     = 0;//                    f * f * -s1;
  
  q_goal  ["LH_X_1"][0]     = q_init["LH_X_1"][0];// +           s1;
  qd_goal ["LH_X_1"][0]     = 0;//                   f *      c1;
  qdd_goal["LH_X_1"][0]     = 0;//                   f * f * -s1;

  q_goal  ["RH_X_1"][0]     = q_init["RH_X_1"][0];// +           s1;
  qd_goal ["RH_X_1"][0]     = 0;//                    f *      c1;
  qdd_goal["RH_X_1"][0]     = 0;//                    f * f * -s1;
/*
  // Hip Sagittal
  q_goal  ["LF_Y_2"][0]     = q_init["LF_Y_2"][0] +           s1;
  qd_goal ["LF_Y_2"][0]     =                        f *      c1;
  qdd_goal["LF_Y_2"][0]     =                        f * f * -s1;
  
  q_goal  ["RF_Y_2"][0]     = q_init["RF_Y_2"][0] +           s1;
  qd_goal ["RF_Y_2"][0]     =                        f *      c1;
  qdd_goal["RF_Y_2"][0]     =                        f * f * -s1;

  q_goal  ["LH_Y_2"][0]     = q_init["LH_Y_2"][0] +           s1;
  qd_goal ["LH_Y_2"][0]     =                        f *      c1;
  qdd_goal["LH_Y_2"][0]     =                        f * f * -s1;

  q_goal  ["RH_Y_2"][0]     = q_init["RH_Y_2"][0] +           s1;
  qd_goal ["RH_Y_2"][0]     =                        f *      c1;
  qdd_goal["RH_Y_2"][0]     =                        f * f * -s1;
  
  // Knees
  q_goal  ["LF_Y_3"][0]     = q_init["LF_Y_3"][0] +           s1;
  qd_goal ["LF_Y_3"][0]     =                        f *      c1;
  qdd_goal["LF_Y_3"][0]     =                        f * f * -s1;

  q_goal  ["RF_Y_3"][0]     = q_init["RF_Y_3"][0] +           s1;
  qd_goal ["RF_Y_3"][0]     =                        f *      c1;
  qdd_goal["RF_Y_3"][0]     =                        f * f * -s1;

  q_goal  ["LH_Y_3"][0]     = q_init["LH_Y_3"][0] +           s1;
  qd_goal ["LH_Y_3"][0]     =                        f *      c1;
  qdd_goal["LH_Y_3"][0]     =                        f * f * -s1;

  q_goal  ["RH_Y_3"][0]     = q_init["RH_Y_3"][0] +           s1;
  qd_goal ["RH_Y_3"][0]     =                        f *      c1;
  qdd_goal["RH_Y_3"][0]     =                        f * f * -s1;
*/
  // Hip Sagittal
  q_goal  ["LF_Y_2"][0]     = q_init["LF_Y_2"][0] +           c0;
  qd_goal ["LF_Y_2"][0]     =                        f *     -s0;
  qdd_goal["LF_Y_2"][0]     =                        f * f * -c0;
  
  q_goal  ["RF_Y_2"][0]     = q_init["RF_Y_2"][0] +           c0;
  qd_goal ["RF_Y_2"][0]     =                        f *     -s0;
  qdd_goal["RF_Y_2"][0]     =                        f * f * -c0;

  q_goal  ["LH_Y_2"][0]     = q_init["LH_Y_2"][0] +           c0;
  qd_goal ["LH_Y_2"][0]     =                        f *     -s0;
  qdd_goal["LH_Y_2"][0]     =                        f * f * -c0;

  q_goal  ["RH_Y_2"][0]     = q_init["RH_Y_2"][0] +           c0;
  qd_goal ["RH_Y_2"][0]     =                        f *     -s0;
  qdd_goal["RH_Y_2"][0]     =                        f * f * -c0;
  
  // Knees
  q_goal  ["LF_Y_3"][0]     = q_init["LF_Y_3"][0] +  -         c0;
  qd_goal ["LF_Y_3"][0]     =                        -f *     -s0;
  qdd_goal["LF_Y_3"][0]     =                        -f * f * -c0;

  q_goal  ["RF_Y_3"][0]     = q_init["RF_Y_3"][0] +  -         c0;
  qd_goal ["RF_Y_3"][0]     =                        -f *     -s0;
  qdd_goal["RF_Y_3"][0]     =                        -f * f * -c0;

  q_goal  ["LH_Y_3"][0]     = q_init["LH_Y_3"][0] +  -         c0;
  qd_goal ["LH_Y_3"][0]     =                        -f *     -s0;
  qdd_goal["LH_Y_3"][0]     =                        -f * f * -c0;

  q_goal  ["RH_Y_3"][0]     = q_init["RH_Y_3"][0] +  -         c0;
  qd_goal ["RH_Y_3"][0]     =                        -f *     -s0;
  qdd_goal["RH_Y_3"][0]     =                        -f * f * -c0;

  ctrl->set_joint_value(Pacer::Controller::position_goal,q_goal);
  ctrl->set_joint_value(Pacer::Controller::velocity_goal,qd_goal);
  ctrl->set_joint_value(Pacer::Controller::acceleration_goal,qdd_goal);

  OUT_LOG(logINFO) << "Time: " << time;

  OUT_LOG(logINFO) << "New q = " << q_goal;
  OUT_LOG(logINFO) << "New qd = " << qd_goal;
  OUT_LOG(logINFO) << "New qdd = " << qdd_goal;
}

void setup(){
  OUT_LOG(logINFO) << "INITING SINUSOID";
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
}
