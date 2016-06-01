/*****************************************************************************
 * "Controller" for constrained pendulum example 
 ****************************************************************************/
#include <Moby/ConstraintSimulator.h>
#include <Moby/RCArticulatedBody.h>
#include <Moby/GravityForce.h>
#include <Ravelin/Pose3d.h>
#include <Ravelin/Vector3d.h>
#include <Ravelin/VectorNd.h>
#include <fstream>
#include <stdlib.h>
#include <stdexcept>

#ifndef USE_GSL
#error Failing build: This plugin should be built with randomization support from GSL. Set 'USE_GSL' to ON to correct build.
#endif

#include <Pacer/Random.h>

using boost::shared_ptr;
using namespace Ravelin;
using namespace Moby;

boost::shared_ptr<ConstraintSimulator> sim;
boost::shared_ptr<RCArticulatedBody> piston,pendulum;
boost::shared_ptr<GravityForce> grav;

std::string outfile_name;
std::string init_file_name;

Ravelin::VectorNd q_initial(1),qd_initial(1);

double init_time = 0.0;
double maxtime = 1.0;
int total_events = 0;
int total_trials = 0;
int event_detected = 0;

void output_final_state(std::string& outfile,bool append = true){
  if(!append){
    std::ofstream out(outfile.c_str());
    out.close();
    return;
  }
  std::ofstream out(outfile.c_str(), std::ostream::app);
  out << event_detected << " " << q_initial[0] << " " <<  qd_initial[0] << std::endl;
  out.close();
  event_detected = 0;
}

void output_state(std::string& outfile,bool append = true){
  if(!append){
    std::ofstream out(outfile.c_str());
    out.close();
    return;
  }
  std::ofstream out(outfile.c_str(), std::ostream::app);
  Ravelin::VectorNd q_piston,q_pendulum,qd_piston,qd_pendulum;
  piston->get_generalized_coordinates_euler(q_piston);
  pendulum->get_generalized_coordinates_euler(q_pendulum);
  piston->get_generalized_velocity(Ravelin::DynamicBodyd::eSpatial,qd_piston);
  pendulum->get_generalized_velocity(Ravelin::DynamicBodyd::eSpatial,qd_pendulum);

  out << sim->current_time-init_time << " " << q_piston[0] << " " <<  qd_piston[0] << " "  << q_pendulum[0] << " " <<  qd_pendulum[0] << std::endl;
  out.close();
}
  
void reset(){
  if(total_trials != 0){
    std::cout << "Trials: " << total_trials << ", Events detected: " << total_events << ", event probability: " << ( (double) total_events / (double) total_trials ) << std::endl;
    // output initial state and if there was an event detected
    output_final_state(init_file_name); 
  }
  
// perturb initial state
#define GRID_SEARCH
//#define MONTE_CARLO  

#ifdef GRID_SEARCH
const int RESOLUTION = 100;
static int i = 0;
static int j = 0;
double max_angle = M_PI*2.0;
double max_speed = 15.0;
//for(int i=0;i<RESOLUTION;i++)
double q_init = (M_PI*2.0 / (double) RESOLUTION) * (double) i;
//for(int j=0;j<RESOLUTION;j++)
double qd_init = max_speed - (2.0 * max_speed / (double) RESOLUTION) * (double) i;
j += 1;
if(j>=RESOLUTION){
  j = 0;
  i += 1;
}
if(i>=RESOLUTION){
  throw std::runtime_error("Experiment complete");
}

#endif

#ifdef MONTE_CARLO
double q_mean = -0.2;
double qd_mean = 0.1;
static Random::Generator q_init_generator, qd_init_generator;
if(total_trials == 0){
  q_init_generator.set_gaussian(q_mean, M_PI/20.0, -M_PI, M_PI);
  qd_init_generator.set_gaussian(qd_mean, max_speed/20.0, -max_speed, max_speed);
}
double q_init = q_init_generator.generate(); 
double qd_init = qd`_init_generator.generate(); 
#endif
q_initial[0] = q_init;
qd_initial[0] = qd_init;

pendulum->get_generalized_coordinates_euler(q_initial);
pendulum->get_generalized_velocity(Ravelin::DynamicBodyd::eSpatial,qd_initial);

// reset pistons
Ravelin::VectorNd q_initial_piston(1),qd_initial_piston(1);
q_initial_piston[0] = 0;
qd_initial_piston[0] = 0.03;
  
piston->get_generalized_coordinates_euler(q_initial_piston);
piston->get_generalized_velocity(Ravelin::DynamicBodyd::eSpatial,qd_initial_piston);

char buffer [50];
static int outfile_number = 0;
outfile_number += 1;
sprintf(buffer,"state-%d.mat",outfile_number);

outfile_name = std::string(buffer);
// Make new output files
output_state(outfile_name,false);
// output initial state
init_time = sim->current_time;
total_trials += 1;
}

// setup simulator callback
void post_step_callback(Simulator* sim)
{
  output_state(outfile_name);
  if(sim->current_time-init_time >= maxtime){
    reset();
  }
}

void post_event_callback(const std::vector<Moby::UnilateralConstraint>& e,
                            boost::shared_ptr<void> empty)
{
  total_events += 1;
  
  output_state(outfile_name);

  reset();
}
/// plugin must be "extern C"
extern "C" {

void init(void* separator, const std::map<std::string, Moby::BasePtr>& read_map, double time)
{
  const unsigned Z = 2;

  // get a reference to the ConstraintSimulator instance
  for (std::map<std::string, Moby::BasePtr>::const_iterator i = read_map.begin();
       i !=read_map.end(); i++)
  {
    // Find the simulator reference
    if (!sim)
      sim = boost::dynamic_pointer_cast<ConstraintSimulator>(i->second);
    if (!grav)
      grav = boost::dynamic_pointer_cast<GravityForce>(i->second);
    if (!piston && i->first == "piston")
      piston = boost::dynamic_pointer_cast<RCArticulatedBody>(i->second);
    if (!pendulum && i->first == "pendulum")
      pendulum = boost::dynamic_pointer_cast<RCArticulatedBody>(i->second);
  }

  init_file_name = std::string("init-states.mat");
  output_final_state(init_file_name,false);
  reset();

  sim->post_step_callback_fn = &post_step_callback;
  sim->constraint_post_callback_fn = &post_event_callback;
}
} // end extern C
