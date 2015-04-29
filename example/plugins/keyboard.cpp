
#include <conio.h>
#include <iostream>
using namespace std;

#define KEY_UP 72
#define KEY_DOWN 80
#define KEY_LEFT 75
#define KEY_RIGHT 77

int main()
{
  
    
  }
  
  return 0;
}

using namespace std;

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
  int X = 0, Y = 1, THETA = 2;
  
    Ravelin::Origin3d command_SE2(0,0,0);

    // Find time since last call
    static double last_time = t;
    double dt = t - last_time;
    last_time = t;
  
  double max_forward_speed = ctrl->get_data<double>(plugin_namespace+"max-forward-speed");
  double max_strafe_speed  = ctrl->get_data<double>(plugin_namespace+"max-strafe-speed");
  double max_turn_speed    = ctrl->get_data<double>(plugin_namespace+"max-turn-speed");

  int c = 0;
  
  switch((c=getch())) {
      cout << endl << c << endl;

    case KEY_UP:
      cout << endl << "Up" << endl;//key up
      command_SE2[X] = max_forward_speed;
      break;
    case KEY_DOWN:
      cout << endl << "Down" << endl;   // key down
      command_SE2[X] = -max_forward_speed;
      break;
    case KEY_LEFT:
      cout << endl << "Right" << endl;  // key right
      command_SE2[THETA] = -max_turn_speed;
      break;
    case KEY_RIGHT:
      cout << endl << "Left" << endl;  // key left
      command_SE2[THETA] = max_turn_speed;
      break;
    default:
      cout << endl << "unknown" << endl;  // key left
      break;
  }

  ctrl->set_data<Ravelin::Origin3d>("SE2_command",command_SE2);
}

/** This is a quick way to register your plugin function of the form:
  * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
  */
#include "register-plugin"
