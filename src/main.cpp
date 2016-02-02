/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <time.h>

using Pacer::Controller;

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

static void init(){
  OUT_LOG(logDEBUG2) << "STARTING PACER" << std::endl;
  
  robot_ptr = boost::shared_ptr<Controller>(new Controller());
  
  robot_ptr->init();
  OUT_LOG(logDEBUG2) << "ROBOT INITED" << std::endl;
  
  robot_ptr->set_generalized_value(Pacer::Robot::position_goal,robot_ptr->get_generalized_value(Pacer::Robot::position));
  robot_ptr->set_generalized_value(Pacer::Robot::velocity_goal,robot_ptr->get_generalized_value(Pacer::Robot::velocity));
  robot_ptr->set_generalized_value(Pacer::Robot::acceleration_goal,robot_ptr->get_generalized_value(Pacer::Robot::acceleration));
}

int main(int argc, char* argv[])
{

  init();
 
  double TIME = 0;
  bool REAL_TIME = false;
  double STEP_SIZE = 0.001;
  robot_ptr->get_data("main.realtime",REAL_TIME); 
  robot_ptr->get_data("main.stepsize",STEP_SIZE); 
  do {
    if(REAL_TIME){
      TIME = get_current_time();
    } else {
      TIME += STEP_SIZE;
    }
    robot_ptr->control( TIME );
  } while(1);
}

