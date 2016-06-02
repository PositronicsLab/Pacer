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

//#define RECORD_EACH_STEP

/////////////////////////////////////
/////// CHOOSE WHICH MODE ////////

#define ORDERED_SEARCH 1
#if ORDERED_SEARCH == 1
  #define SOBOL_SEQUENCE_SEARCH 1
  #define GRID_SEARCH 0
  #define AT_POINT 1
#endif

#define MONTE_CARLO 0

/////////////////////////////////////
/////// SET THESE PARAMETERS ////////

double max_angle = M_PI;
double max_speed = 15.0;
// safety_factor is 1 stddev of search
double safety_factor = 1.0 / 20.0;
double num_stddevs   = 3.0;
const int RESOLUTION = 25;

/////////////////////////////////////
/////////////////////////////////////


#ifdef SOBOL_SEQUENCE_SEARCH
#include "nrutil.h"
#define MAXBIT 30
#define MAXDIM 6
//When n is negative, internally initializes a set of MAXBIT direction numbers for each of MAXDIM
//different Sobol’ sequences. When n is positive (but ≤MAXDIM), returns as the vector x[1..n]
//the next values from n of these sequences. (n must not be changed between initializations.)
void sobseq(int *n, double * x)
{
  int j,k,l;
  unsigned long i,im,ipp;
  static double fac;
  static unsigned long in,ix[MAXDIM+1],*iu[MAXBIT+1];
  static unsigned long mdeg[MAXDIM+1]={0,1,2,3,3,4,4};
  static unsigned long ip[MAXDIM+1]={0,0,1,1,2,1,4};
  static unsigned long iv[MAXDIM*MAXBIT+1]={
    0,1,1,1,1,1,1,3,1,3,3,1,1,5,7,7,3,3,5,15,11,5,15,13,9};
  if (*n < 0) {
    //Initialize, don’t return a vector.
    for (k=1;k<=MAXDIM;k++) ix[k]=0;
    in=0;
    if (iv[1] != 1) return;
    fac=1.0/(1L << MAXBIT);
    for (j=1,k=0;j<=MAXBIT;j++,k+=MAXDIM)
      iu[j] = &iv[k];
    //To allowboth 1D and 2D addressing.
    for (k=1;k<=MAXDIM;k++) {
      for (j=1;j<=mdeg[k];j++)
        iu[j][k] <<= (MAXBIT-j);
      //Stored values only require normalization.
      for (j=mdeg[k]+1;j<=MAXBIT;j++) {
        //Use the recurrence to get other values.
        ipp=ip[k];
        i=iu[j-mdeg[k]][k];
        i ^= (i >> mdeg[k]);
        for (l=mdeg[k]-1;l>=1;l--) {
          if (ipp & 1) i ^= iu[j-l][k];
          ipp >>= 1;
        }
        iu[j][k]=i;
      }
    }
  } else {
    //Calculate the next vector in the sequence.
    im=in++;
    for (j=1;j<=MAXBIT;j++) {
      //Find the rightmost zero bit.
      if (!(im & 1)) break;
      im >>= 1;
    }
    if (j > MAXBIT) throw std::runtime_error("MAXBIT too small in sobseq");
    im=(j-1)*MAXDIM;
    for (k=1;k<=IMIN(*n,MAXDIM);k++) {
      //XOR the appropriate direction number
      //into each component of the
      //vector and convert to a floating
      //number.
      ix[k] ^= iv[im+k];
      x[k]=ix[k]*fac;
    }
  }
}
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

Ravelin::VectorNd q_initial,qd_initial;
Ravelin::VectorNd q_pendulum_start,qd_pendulum_start;

double init_time = 0.0;
double maxtime = 0.2;
int total_events = 0;
int total_trials = 0;
int event_detected = 0;

void output_final_state(std::string& outfile,bool append = true){
  if(!append){
    std::ofstream out(outfile.c_str());
    out.close();
    return;
  }
  int N = q_initial.rows();
  std::ofstream out(outfile.c_str(), std::ostream::app);
  out << event_detected << " ";
  for (int i = 0; i<N; i++) {
     out << q_initial[i] << " ";
  }
  for (int i = 0; i<N; i++) {
    out << qd_initial[i] << " ";
  }
  out << N << std::endl;
#ifndef NDEBUG
  std::cout << event_detected << " , q = [ ";
  for (int i = 0; i<N; i++) {
    std::cout << q_initial[i] << " ";
  }
  std::cout << " ] , qd = [ ";
  for (int i = 0; i<N; i++) {
    std::cout << qd_initial[i] << " ";
  }
  std::cout << " ] , N = " << N << std::endl;
#endif

  out.close();
#if MONTE_CARLO == 1
  if (event_detected == 1) {
    throw std::runtime_error("Experiment complete");
  }
#endif
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
//#ifndef NDEBUG
//  std::cout << sim->current_time-init_time << " " << q_piston[0] << " " <<  qd_piston[0] << " "  << q_pendulum[0] << " " <<  qd_pendulum[0] << std::endl;
//#endif
  out.close();
}

void reset(){
  if(total_trials != 0){
    std::cout << "Trials: " << total_trials << ", Events detected: " << total_events << ", event probability: " << ( (double) total_events / (double) total_trials ) << std::endl;
    // output initial state and if there was an event detected
    output_final_state(init_file_name);
  }
  
  // perturb initial state
  
  static int N = qd_pendulum_start.rows();
  Ravelin::VectorNd q_init(N), qd_init(N);
  
#if ORDERED_SEARCH == 1
  assert(MONTE_CARLO == 0);
  static Ravelin::VectorNd ind_q_qd(N*2);
  if(total_trials == 0){
    std::fill(ind_q_qd.begin(),ind_q_qd.end(),0);
  }
  
#if SOBOL_SEQUENCE_SEARCH == 1
  assert(GRID_SEARCH == 0);
  assert(N*2 <= MAXDIM);
  static int n = -1;
  static double sobol_q_qd[MAXDIM+1];
  if (total_trials == 0) {
    sobseq(&n,sobol_q_qd);
    n = N*2;
  }
  sobseq(&n,sobol_q_qd);

  Ravelin::VectorNd sobol_x_vec(N*2);
  for (int i=0; i<N*2; i++) {
    sobol_x_vec[i] = sobol_q_qd[i+1];
  }

#ifndef NDEBUG
  std::cout << " SOBSEQ = "<< sobol_x_vec << std::endl;
#endif
  (ind_q_qd = sobol_x_vec) *= (double) RESOLUTION;
#endif
  
  // Get grid point
  for (int ii = 0; ii < N*2; ii++) {
    if(ii < N){
      int i = ii;
      q_init[i] = max_angle - (max_angle*2.0 / (double) RESOLUTION) * (double) ind_q_qd[ii];
//      std::cout << " q_init[" << i << "] = " << q_init[i] << std::endl;
    } else {
      int i = ii-N;
      qd_init[i] = max_speed - (2.0 * max_speed / (double) RESOLUTION) * (double) ind_q_qd[ii];
//      std::cout << " qd_init[" << i << "] = " << qd_init[i] << std::endl;
    }
  }
  
#if GRID_SEARCH == 1
  assert(SOBOL_SEQUENCE_SEARCH == 0);
#ifndef NDEBUG
  const long double expected_total = pow(RESOLUTION,N*2);
  // Increment and record sample index
  std::cout << "Indices = [ ";
  for (int ii = 0; ii < N*2; ii++) {
    std::cout << ind_q_qd[ii] << " ";
  }
  std::cout << "] / " << RESOLUTION << ", (total trials) "<< expected_total << std::endl;
#endif
  
  // Increment and record sample index
  ind_q_qd[0] += 1;
  for (int ii = 0; ii < N*2 - 1; ii++) {
    if(ind_q_qd[ii]>=RESOLUTION){
      ind_q_qd[ii] = 0;
      ind_q_qd[ii+1] += 1;
    }
  }

  
  // If this is all the samples (reached all increments)
  if(ind_q_qd[N*2-1]>=RESOLUTION){
    throw std::runtime_error("Experiment complete");
  }
#endif

#if AT_POINT == 1
  // scale to safety_factor*"num_stddevs" standard deviations search
  q_init  *= safety_factor*num_stddevs;
  qd_init *= safety_factor*num_stddevs;
#endif
#endif
  
#if MONTE_CARLO == 1
  assert(ORDERED_SEARCH == 0);
  static Random::Generator q_init_generator, qd_init_generator;
  if(total_trials == 0){
    q_init_generator.set_gaussian(0, max_angle*safety_factor, -max_angle, max_angle);
    qd_init_generator.set_gaussian(0, max_speed*safety_factor, -max_speed, max_speed);
  }
  for (int i = 0; i < N; i++) {
    q_init[i]  = q_init_generator.generate();
    qd_init[i] = qd_init_generator.generate();
  }
#endif
  
  // Assign initial state to pendulum
  q_initial = q_pendulum_start;
  qd_initial = qd_pendulum_start;
#if (AT_POINT == 1) || (MONTE_CARLO == 1)
  q_initial += q_init;
  qd_initial += qd_init;
#else
  q_initial = q_init;
  qd_initial = qd_init;
#endif
//  std::cout << " q_initial = "<< q_initial << std::endl;
//  std::cout << " qd_initial = "<< qd_initial << std::endl;

  pendulum->set_generalized_coordinates_euler(q_initial);
  pendulum->set_generalized_velocity(Ravelin::DynamicBodyd::eSpatial,qd_initial);
  
  // reset pistons
  Ravelin::VectorNd q_initial_piston(1),qd_initial_piston(1);
  q_initial_piston[0] = 0;
  qd_initial_piston[0] = 0.5;
  
  piston->set_generalized_coordinates_euler(q_initial_piston);
  piston->set_generalized_velocity(Ravelin::DynamicBodyd::eSpatial,qd_initial_piston);
  
  char buffer [50];
  static int outfile_number = 0;
  outfile_number += 1;
  sprintf(buffer,"state-%d.mat",outfile_number);
  
  outfile_name = std::string(buffer);
  // Make new output files
#ifdef RECORD_EACH_STEP
  output_state(outfile_name,false);
#endif
  // output initial state
  init_time = sim->current_time;
  total_trials += 1;
}

// setup simulator callback
void post_step_callback(Simulator* sim)
{
#ifdef RECORD_EACH_STEP
  output_state(outfile_name);
#endif
  if(sim->current_time-init_time >= maxtime){
    reset();
  }
}

void pre_event_callback(std::vector<Moby::UnilateralConstraint>& e,
                         boost::shared_ptr<void> empty)
{
  total_events += 1;
  event_detected = 1;
  e.clear();
#ifdef RECORD_EACH_STEP
  output_state(outfile_name);
#endif
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
    
    pendulum->get_generalized_coordinates_euler(q_pendulum_start);
    pendulum->get_generalized_velocity(Ravelin::DynamicBodyd::eSpatial,qd_pendulum_start);
    
    reset();
    
    sim->post_step_callback_fn = &post_step_callback;
    sim->constraint_callback_fn = &pre_event_callback;
  }
} // end extern C
