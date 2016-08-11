/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <time.h>

using Pacer::Controller;

namespace Pacer {
static boost::shared_ptr<Controller> robot_ptr;

static double sleep_duration(double duration){
  timespec req,rem;
  int seconds = duration;
  req.tv_nsec = (duration - (double)seconds) * 1.0e+9;
  req.tv_sec = seconds;
  nanosleep(&req,&rem);
  return 0;//( ( (double) rem.tv_nsec / 1.0e+9 ) + (double) rem.tv_sec);
}
/// Gets the current time (as a floating-point number)
static double get_current_time()
{
  const double MICROSEC = 1.0/1000000;
  timeval t;
  gettimeofday(&t, NULL);
  return (double) t.tv_sec + (double) t.tv_usec * MICROSEC;
}

  bool REAL_TIME = false;
  double STEP_SIZE = 0.001;
  double TIME = 0;

void init(){
  OUT_LOG(logDEBUG2) << "STARTING PACER" << std::endl;
  
  robot_ptr = boost::shared_ptr<Controller>(new Controller());
  
  robot_ptr->init();
  OUT_LOG(logDEBUG2) << "ROBOT INITED" << std::endl;
  
  robot_ptr->get_data("main.realtime",REAL_TIME); 
  robot_ptr->get_data("main.stepsize",STEP_SIZE);
  
  robot_ptr->set_generalized_value(Pacer::Robot::position_goal,robot_ptr->get_generalized_value(Pacer::Robot::position));
  robot_ptr->set_generalized_value(Pacer::Robot::velocity_goal,robot_ptr->get_generalized_value(Pacer::Robot::velocity));
  robot_ptr->set_generalized_value(Pacer::Robot::acceleration_goal,robot_ptr->get_generalized_value(Pacer::Robot::acceleration));
  robot_ptr->control( TIME );
}

double step(double step_size = 0){
    double step = STEP_SIZE;
    if(step_size>0)
      step = step_size;
  
  
  
    if(REAL_TIME){
      TIME = get_current_time();
      std::cerr << "Realtime : " <<  std::endl;

    } else {
      TIME += step;
    }
    static double FIRST_TIME = TIME-step;

    std::cerr << "time = " << (TIME-FIRST_TIME) << std::endl;
  robot_ptr->control( TIME-FIRST_TIME );
  return TIME;
}
}


