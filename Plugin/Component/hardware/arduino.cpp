#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include <ServoController/ServoDriver.h>

#include "../plugin.h"

/*  Server Controller test file
 *  Behavior: Servos with ids \in {1..12} will oscilate within 1/10 of their maximum range
 *            3 full periods of sinusoidal oscilation will occur before exit.
 *  usage:
 *     ./ServoController-Test <arduino serial port>
 */
int baud = 115200;
std::vector<int> ids;
std::vector<double> send_pos;
std::map<int,double> TARE;

void loop(){
boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  const int N = ids.size();
  const int Bps = ( baud / 10 );
  const int Bytes = ( N * 3 ) ;
  const double seconds_per_message = 0.005;//( 1.0 / ((double) Bps) ) * ((double)Bytes);
  
  send_pos[0] = sin( t * 16.0) * (M_PI/8.0) + TARE[ids[0]] ;
  send_pos[1] = sin( t * 16.0) * (M_PI/8.0) + TARE[ids[1]] ;
  send_pos[2] = sin( t * 16.0) * (M_PI/8.0) + TARE[ids[2]] ;
  
  //std::cout << t << " : " <<send_pos<< " -- " << ids <<std::endl;
  Arduino::setVal(ids,send_pos);
}

void setup(){
boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  TARE[3] = -1.5709;
  TARE[7] = 0;
  TARE[11] = -1.5709;
  ids.resize(3);
  ids[0] = 3;
  ids[1] = 7;
  ids[2] = 11;
  send_pos.resize(ids.size());
  /*
  for(int i=0;i<12;i++)
    ids.push_back(i+1);
*/

  std::string port;
  ctrl->get_data<std::string>(plugin_namespace+".port",port);
  ctrl->get_data<int>(plugin_namespace+".baud-rate",baud);

  Arduino::init(port.c_str(),baud);
}
