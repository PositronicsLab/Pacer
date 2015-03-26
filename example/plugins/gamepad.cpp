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

#include <SDL2/SDL.h>
#include <assert.h>
#include <curses.h>
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>

/* EOF */
#include <Pacer/controller.h>
#include <Pacer/utilities.h>
std::string plugin_namespace;

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
          throw std::runtime_error("Exited");
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


void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
  // SDL2 will only report events when the window has focus, so set
  // this hint as we don't have a window
  try {
    static Joystick j = Joystick(0);
 
    j.update();
  
    double max_forward_speed = 0.2;
    double max_strafe_speed = 0.05;
    double max_turn_speed = 1;
    Ravelin::Origin3d command_SE2(0,0,0);
 
    int i; // strafe
    {
      i = 0;
      if(j.axes[i] == 128)
        command_SE2[1] = 0;
      else
        command_SE2[1] = max_strafe_speed * - (double) j.axes[i] / (double) 32767;
    }
    {
      i = 1; // forward
      if(j.axes[i] == 128)
        command_SE2[0] = 0;
      else
        command_SE2[0] = max_forward_speed * - (double) j.axes[i] / (double) 32767;
    }
    {
      i = 2; // turn
      if(j.axes[i] == 128)
        command_SE2[2] = 0;
      else
        command_SE2[2] = max_turn_speed * - (double) j.axes[i] / (double) 32767;
    }
    ctrl->set_data<Ravelin::Origin3d>("SE2_command",command_SE2);
  
  } catch(std::exception& e){
      exit(0);
  }
}

/** This is a quick way to register your plugin function of the form:
  * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
  */
#include "register-plugin"
