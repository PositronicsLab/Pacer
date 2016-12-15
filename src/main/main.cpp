/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <time.h>
namespace Pacer{
extern void init( );
extern double step(double dt = 0);
}

int main(int argc, char* argv[])
{
  Pacer::init();

  double time = 0;
  while (time < 10) {
    time = Pacer::step();
  }

  return 0;
}
