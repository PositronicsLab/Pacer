// sdl-jstest - Joystick Test Program for SDL
// Copyright (C) 2014 Ingo Ruhnke <grumbel@gmx.de>
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.

//#define USE_CURSES
//#define PS3
#define SABRENT
//#define XBOX

#include <SDL2/SDL.h>
#include <assert.h>
#ifdef USE_CURSES
#include <curses.h>
#endif
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>

/* EOF */
#include <Pacer/controller.h>
#include <Pacer/utilities.h>
std::string plugin_namespace;

#ifdef USE_CURSES
void print_bar(int pos, int len)
{
  addch('[');
  for(int i = 0; i < len; ++i)
  {
    if (i == pos)
      addch('#');
    else
      addch(' ');
  }
  addch(']');
}
#endif

int str2int(const char* str, int* val)
{
  char* endptr;
  errno = 0;    /* To distinguish success/failure after call */

  *val = strtol(str, &endptr, 10);

  /* Check for various possible errors */
  if ((errno == ERANGE && (*val == LONG_MAX || *val == LONG_MIN))
      || (errno != 0 && *val == 0)) {
    return 0;
  }

  if (endptr == str) {
    return 0;
  }

  return 1;
}

class Joystick
{
  public:
  int num_axes    ;
  int num_buttons ;
  int num_hats    ;
  int num_balls   ;

  int * axes    ;
  unsigned short * buttons ;
  unsigned short * hats    ;
  unsigned short * balls;
  SDL_Joystick* joy;

  void update()
  {
  
    SDL_Event event;
    bool something_new = false;
    if(SDL_PollEvent(&event)){
      something_new = true;
      switch(event.type)
      {
        case SDL_JOYAXISMOTION:
          assert(event.jaxis.axis < num_axes);
          axes[event.jaxis.axis] = event.jaxis.value;
          break;

        case SDL_JOYBUTTONDOWN:
        case SDL_JOYBUTTONUP:
          assert(event.jbutton.button < num_buttons);
          buttons[event.jbutton.button] = event.jbutton.state;
          break;

        case SDL_JOYHATMOTION:
          assert(event.jhat.hat < num_hats);
          hats[event.jhat.hat] = event.jhat.value;
          break;

        case SDL_JOYBALLMOTION:
          assert(event.jball.ball < num_balls);
          balls[2*event.jball.ball + 0] = event.jball.xrel;
          balls[2*event.jball.ball + 1] = event.jball.yrel;
          break;

        case SDL_QUIT:
          printf("Recieved interrupt, exiting\n");
          exit(0);
          break;

        default:
          fprintf(stderr, "Error: Unhandled event type: %d\n", event.type);
      }
    }

#ifdef USE_CURSES
    if (something_new)
    {
      //clear();
      move(0,0);

      printw("Joystick Name:   '%s'\n", SDL_JoystickName(joy));
      printw("\n");

      printw("Axes %2d:\n", num_axes);
      for(int i = 0; i < num_axes; ++i)
      {
        int len = COLS - 20;
        printw("  %2d: %6d  ", i, axes[i]);
        print_bar((axes[i] + 32767) * (len-1) / 65534, len);
        addch('\n');
      }
      printw("\n");

      printw("Buttons %2d:\n", num_buttons);
      for(int i = 0; i < num_buttons; ++i)
      {
        printw("  %2d: %d  %s\n", i, buttons[i], buttons[i] ? "[#]":"[ ]");
      }
      printw("\n");

      printw("Hats %2d:\n", num_hats);
      for(int i = 0; i < num_hats; ++i)
      {
        printw("  %2d: value: %d\n", i, hats[i]);
        printw("  +-----+  up:    %c\n"
               "  |%c %c %c|  down:  %c\n"
               "  |%c %c %c|  left:  %c\n"
               "  |%c %c %c|  right: %c\n"
               "  +-----+\n",

               (hats[i] & SDL_HAT_UP)?'1':'0',

               ((hats[i] & SDL_HAT_UP) && (hats[i] & SDL_HAT_LEFT)) ? 'O' : ' ',
               ((hats[i] & SDL_HAT_UP) && !(hats[i] & (SDL_HAT_LEFT | SDL_HAT_RIGHT))) ? 'O' : ' ',
               ((hats[i] & SDL_HAT_UP) && (hats[i] & SDL_HAT_RIGHT)) ? 'O' : ' ',

               (hats[i] & SDL_HAT_DOWN)?'1':'0',

               (!(hats[i] & (SDL_HAT_UP | SDL_HAT_DOWN)) && (hats[i] & SDL_HAT_LEFT)) ? 'O' : ' ',
               (!(hats[i] & (SDL_HAT_UP | SDL_HAT_DOWN)) && !(hats[i] & (SDL_HAT_LEFT | SDL_HAT_RIGHT))) ? 'O' : ' ',
               (!(hats[i] & (SDL_HAT_UP | SDL_HAT_DOWN)) && (hats[i] & SDL_HAT_RIGHT)) ? 'O' : ' ',

               (hats[i] & SDL_HAT_LEFT)?'1':'0',

               ((hats[i] & SDL_HAT_DOWN) && (hats[i] & SDL_HAT_LEFT)) ? 'O' : ' ',
               ((hats[i] & SDL_HAT_DOWN) && !(hats[i] & (SDL_HAT_LEFT | SDL_HAT_RIGHT))) ? 'O' : ' ',
               ((hats[i] & SDL_HAT_DOWN) && (hats[i] & SDL_HAT_RIGHT)) ? 'O' : ' ',

               (hats[i] & SDL_HAT_RIGHT)?'1':'0');
      }
      printw("\n");

      printw("Balls %2d: ", num_balls);
      for(int i = 0; i < num_balls; ++i)
      {
        printw("  %2d: %6d %6d\n", i, balls[2*i+0], balls[2*i+0]);
      }
      printw("\n");
      printw("\n");
      printw("Press Ctrl-c to exit\n");

      refresh();
    }
#endif
  } // while
 
  Joystick(): Joystick(0) {}
  Joystick(int joy_idx){
    SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
 
    // FIXME: We don't need video, but without it SDL will fail to work in SDL_WaitEvent()
    if(SDL_Init(SDL_INIT_TIMER | SDL_INIT_VIDEO | SDL_INIT_JOYSTICK | SDL_INIT_GAMECONTROLLER | SDL_INIT_HAPTIC) < 0)
    {
      throw std::runtime_error("Unable to init SDL");
    }
 
    joy = SDL_JoystickOpen(joy_idx);
    if (!joy)
    {
      throw std::runtime_error("Unable to open joystick");
    }
    else
    {
#ifdef USE_CURSES
      initscr();
      //cbreak();
      //noecho();
      //nonl();
      curs_set(0);
#endif
      num_axes    = SDL_JoystickNumAxes(joy);
      num_buttons = SDL_JoystickNumButtons(joy);
      num_hats    = SDL_JoystickNumHats(joy);
      num_balls   = SDL_JoystickNumBalls(joy);
     
      axes    = (int*) calloc(num_axes,    sizeof(int));
      buttons = (unsigned short*) calloc(num_buttons, sizeof(unsigned short));
      hats    = (unsigned short*) calloc(num_hats,    sizeof(unsigned short));
      balls   = (unsigned short*) calloc(num_balls,   2*sizeof(int));
    }
  }

  ~Joystick(){
    free(balls);
    free(hats);
    free(buttons);
    free(axes);

#ifdef USE_CURSES
    endwin();
#endif
  }
};

#include <boost/assign/list_of.hpp>

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){

  // SDL2 will only report events when the window has focus, so set
  // this hint as we don't have a window
  const int JDEADZONE = 2000;
  std::string GAMEPAD_TYPE = ctrl->get_data<std::string>(plugin_namespace+".type");

  try {
    static Joystick j = Joystick(0);

    j.update();
    
    static bool is_sabrent = (GAMEPAD_TYPE.compare("SABRENT") == 0);
    static bool is_ps = (GAMEPAD_TYPE.compare("PS") == 0);
    static bool is_xbox = (GAMEPAD_TYPE.compare("XBOX") == 0);

//    static double press_time = 0;
//    const double wait_time = 0.01;
  if (is_sabrent){
    static std::vector<double> trot = boost::assign::list_of(0.25)(0.75)(0.75)(0.25);
    static std::vector<double> walk = boost::assign::list_of(0.25)(0.75)(0.0)(0.5);
    static std::vector<double> walk2= boost::assign::list_of(0.25)(0.5)(0.0)(0.75);
    static std::vector<double> pace = boost::assign::list_of(0.25)(0.75)(0.25)(0.75);
    if(j.buttons[5] == 1){
      exit(0);
    } else
    if(j.buttons[1] == 1){
      ctrl->set_data< std::vector<double> >("gait-planner.gait",trot);
    } else
    if(j.buttons[0] == 1){
      ctrl->set_data< std::vector<double> >("gait-planner.gait",walk);
    } else
    if(j.buttons[2] == 1){
      ctrl->set_data< std::vector<double> >("gait-planner.gait",walk2);
    } else
    if(j.buttons[3] == 1){
      ctrl->set_data< std::vector<double> >("gait-planner.gait",pace);
    }
  }
//  } else if (is_ps){
//    const int X_BUTTON = 14;
//    if(j.buttons[X_BUTTON] == 1 && !pressed){ // X button
//      if(t-press_time > wait_time){
//        pressed = true;
//        press_time = t;
//      }
//    } else if(j.buttons[X_BUTTON] == 1 && pressed) {
//      if(t-press_time > wait_time){
//        std::vector<double> target2d(0);
//        ctrl->set_data<std::vector<double> >("waypoints.waypoints",target2d);
//        pressed = false;
//        press_time = t;
//      }
//    }
//  }


    Ravelin::Origin3d command_SE2(0,0,0);

  // Find time since last call
  static double last_time = t;
  double dt = t - last_time;
  last_time = t;
    if (is_sabrent){

      double increment = 0.0002;
      std::vector<double> adjustment = boost::assign::list_of(0)(0);
      for(int i = 0; i < j.num_hats; ++i)
      {
        if((j.hats[i] & SDL_HAT_UP) && (j.hats[i] & SDL_HAT_LEFT)) {
          adjustment[0] += increment;
          adjustment[1] += increment;
        }
        if((j.hats[i] & SDL_HAT_UP) && !(j.hats[i] & (SDL_HAT_LEFT | SDL_HAT_RIGHT))) {
          adjustment[0] += increment;
        }
        if((j.hats[i] & SDL_HAT_UP) && (j.hats[i] & SDL_HAT_RIGHT)) {
          adjustment[0] += increment;
          adjustment[1] -= increment;
        }
        if(!(j.hats[i] & (SDL_HAT_UP | SDL_HAT_DOWN)) && (j.hats[i] & SDL_HAT_LEFT)) {
          adjustment[1] += increment;
        }
        if(!(j.hats[i] & (SDL_HAT_UP | SDL_HAT_DOWN)) && !(j.hats[i] & (SDL_HAT_LEFT | SDL_HAT_RIGHT))) {
          //           Do nothing
        }
        if(!(j.hats[i] & (SDL_HAT_UP | SDL_HAT_DOWN)) && (j.hats[i] & SDL_HAT_RIGHT)) {
          adjustment[1] -= increment;
        }
        if((j.hats[i] & SDL_HAT_DOWN) && (j.hats[i] & SDL_HAT_LEFT)) {
          adjustment[0] -= increment;
          adjustment[1] += increment;
        }
        if((j.hats[i] & SDL_HAT_DOWN) && !(j.hats[i] & (SDL_HAT_LEFT | SDL_HAT_RIGHT))) {
          adjustment[0] -= increment;
        }
        if((j.hats[i] & SDL_HAT_DOWN) && (j.hats[i] & SDL_HAT_RIGHT)) {
          adjustment[0] -= increment;
          adjustment[1] -= increment;
        }
      }
      
      if (j.buttons[7] == 0 && j.buttons[9] == 0 && j.buttons[4] == 0 ) {
        // Pose Adjustments
        static std::vector<double> pose_adjustment = boost::assign::list_of(0)(0)(0)(0)(0)(0);
        static std::vector<double> pose_adjustment_limit = boost::assign::list_of(0.05)(0.05)(0.05)(0.392)(0.2)(0.2);

        if(j.buttons[6] == 0 && j.buttons[8] == 0 ){
          pose_adjustment[0] += adjustment[0];
          pose_adjustment[1] += adjustment[1];
        } else if(j.buttons[6] == 1 && j.buttons[8] == 0 ) {
          pose_adjustment[3] += adjustment[1]*10.0;
          pose_adjustment[4] += adjustment[0]*10.0;
        } else if(j.buttons[6] == 0 && j.buttons[8] == 1 ) {
          pose_adjustment[2] += adjustment[0];
          pose_adjustment[5] += adjustment[1]*10.0;
        } else if(j.buttons[6] == 1 && j.buttons[8] == 1 ) {
          for (int i=0;i<6; i++) {
            pose_adjustment[i] = pose_adjustment[i]*0.95;
          }
        }
        
        static std::vector<double> initial_pose = ctrl->get_data< std::vector<double> >("gait-planner.pose");
        std::vector<double> new_pose(6);
        for (int i=0;i<6; i++) {
          if(pose_adjustment[i] > pose_adjustment_limit[i]){
            pose_adjustment[i] = pose_adjustment_limit[i];
          } else if(pose_adjustment[i] < -pose_adjustment_limit[i]){
            pose_adjustment[i] = -pose_adjustment_limit[i];
          }
          new_pose[i] = initial_pose[i] + pose_adjustment[i];
        }
        ctrl->set_data< std::vector<double> >("gait-planner.pose",new_pose);
        
      } else if (j.buttons[7] == 1 || j.buttons[9] == 1) {
        // Gait Adjustment
        static double initial_width = ctrl->get_data<double>("gait-planner.width");
        static double initial_length = ctrl->get_data<double>("gait-planner.length");
        static double initial_step_height = ctrl->get_data<double>("gait-planner.step-height");
        static double initial_gait_duration = ctrl->get_data<double>("gait-planner.gait-duration");

        static double width = 0;
        static double length = 0;
        static double step_height = 0;
        static double gait_duration = 0;
        
        if(j.buttons[7] == 1 && j.buttons[9] == 0 ){
          width  -= adjustment[1];
          length += adjustment[0];
          ctrl->set_data<double>("gait-planner.length",initial_length+length);
          ctrl->set_data<double>("gait-planner.width",initial_width+width);
        } else if(j.buttons[7] == 0 && j.buttons[9] == 1 ){
          step_height += adjustment[0];
          if(step_height < -initial_step_height)
            step_height = -initial_step_height;

          if(step_height+initial_step_height > 0.06)
            step_height = 0.06-initial_step_height;

          gait_duration -= adjustment[1]*100.0;
          if(gait_duration < (-initial_gait_duration+0.1))
            gait_duration = (-initial_gait_duration+0.1);
          ctrl->set_data<double>("gait-planner.step-height",initial_step_height+step_height);
          ctrl->set_data<double>("gait-planner.gait-duration",initial_gait_duration+gait_duration);
        }  else if(j.buttons[7] == 1 && j.buttons[9] == 1 ){
          width  = 0;
          length = 0;
          step_height = 0;
          gait_duration = 0;
          ctrl->set_data<double>("gait-planner.length",initial_length);
          ctrl->set_data<double>("gait-planner.width",initial_width);
          ctrl->set_data<double>("gait-planner.step-height",initial_step_height);
          ctrl->set_data<double>("gait-planner.gait-duration",initial_gait_duration+gait_duration);
        }
      } else // j.buttons[4] == 1
      {
        static std::vector<double> initial_duty_factor = ctrl->get_data< std::vector<double> >("gait-planner.duty-factor");
        std::vector<double> new_duty_factor = initial_duty_factor;
        static double duty_factor = initial_duty_factor[0];
        duty_factor += adjustment[0]*10.0;
        if(duty_factor < 0.1)
          duty_factor = 0.1;
        if(duty_factor > 0.9)
          duty_factor = 0.9;
        for (int i=0; i<4; i++) {
          new_duty_factor[i] = duty_factor;
        }
        ctrl->set_data< std::vector<double> >("gait-planner.duty-factor",new_duty_factor);

      }


  }
//      else if (is_ps){
//const int
//      BUTTON_UP = 4,
//      BUTTON_LEFT = 7,
//      BUTTON_RIGHT = 5,
//      BUTTON_DOWN = 6;
//      if((j.buttons[BUTTON_UP] == 1) && (j.buttons[BUTTON_LEFT] == 1))
//        movement = Ravelin::Vector3d(1,1,0);
//      else if((j.buttons[BUTTON_UP] == 1) && !(j.buttons[BUTTON_LEFT] == 1 ||  j.buttons[BUTTON_LEFT] == 1))
//        movement = Ravelin::Vector3d(1,0,0);
//      else if((j.buttons[BUTTON_UP] == 1) && (j.buttons[BUTTON_RIGHT] == 1))
//        movement = Ravelin::Vector3d(1,-1,0);
//      
//      else if(!(j.buttons[BUTTON_UP] == 1 ||  j.buttons[BUTTON_DOWN] == 1) && (j.buttons[BUTTON_LEFT] == 1))
//        movement = Ravelin::Vector3d(0,1,0);
//      else if(!(j.buttons[BUTTON_UP] == 1 ||  j.buttons[BUTTON_DOWN] == 1) && !(j.buttons[BUTTON_LEFT] == 1 ||  j.buttons[BUTTON_LEFT] == 1))
//        movement = Ravelin::Vector3d(0,0,0);
//      else if(!(j.buttons[BUTTON_UP] == 1 ||  j.buttons[BUTTON_DOWN] == 1) && (j.buttons[BUTTON_RIGHT] == 1))
//        movement = Ravelin::Vector3d(0,-1,0);
//      
//      else if((j.buttons[BUTTON_DOWN] == 1) && (j.buttons[BUTTON_LEFT] == 1))
//        movement = Ravelin::Vector3d(-1,1,0);
//      else if((j.buttons[BUTTON_DOWN] == 1) && !(j.buttons[BUTTON_LEFT] == 1 ||  j.buttons[BUTTON_LEFT] == 1))
//        movement = Ravelin::Vector3d(-1,0,0);
//      else if((j.buttons[BUTTON_DOWN] == 1) && (j.buttons[BUTTON_RIGHT] == 1))
//        movement = Ravelin::Vector3d(-1,-1,0);
//    }
//      movement*=dt*10;
//      target += movement;
//      Utility::visualize.push_back(Pacer::VisualizablePtr( new Pacer::Point(target,Ravelin::Vector3d(1,0.5,0),1.0)));
//      std::vector<double> target2d(2);
//      target2d[0] = target[0];
//      target2d[1] = target[1];
//      ctrl->set_data<std::vector<double> >("waypoints.waypoints",target2d);
//    } else
    {
      double max_forward_speed = ctrl->get_data<double>(plugin_namespace+".max-forward-speed");
      double max_strafe_speed  = ctrl->get_data<double>(plugin_namespace+".max-strafe-speed");
      double max_turn_speed    = ctrl->get_data<double>(plugin_namespace+".max-turn-speed");
      
      const int MAX_VAL =32767;
      int X = 1,Y = 0,THETA = 2;
      if (is_sabrent || is_ps){
        X = 1;Y = 0;THETA = 2;
      } else if(is_xbox){
        X = 1;Y = 0;THETA = 3;
      }
      double disp = sqrt(j.axes[X]*j.axes[X]+j.axes[Y]*j.axes[Y]);
      if(disp < JDEADZONE){
        command_SE2[0] = 0;
        command_SE2[1] = 0;
      }else{
        command_SE2[0] = max_forward_speed * - (double) j.axes[X] / (double) MAX_VAL;
        command_SE2[1] = max_strafe_speed * - (double) j.axes[Y] / (double) MAX_VAL;
      }
      if(fabs(j.axes[THETA]) < JDEADZONE)
        command_SE2[2] = 0;
      else
        command_SE2[2] = max_turn_speed * - (double) j.axes[THETA] / (double) MAX_VAL;
    }
    ctrl->set_data<Ravelin::Origin3d>("SE2_command",command_SE2);

  //  std::cout << "forward (m/s)     = " << command_SE2[0] << std::endl;
  //  std::cout << "strafe_left (m/s) = " << command_SE2[1] << std::endl;
  //  std::cout << "turn_left (rad/s) = " << command_SE2[2] << std::endl;
  } catch(std::exception& e){
    std::cerr << "Could not connect to Controller: " << GAMEPAD_TYPE << std::endl;
    throw std::runtime_error( "Could not connect to Controller: " + GAMEPAD_TYPE);
  }
}

/** This is a quick way to register your plugin function of the form:
  * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
  */
#include "../register-plugin"
