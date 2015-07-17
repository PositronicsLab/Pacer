//
//  Random.h
//
//
//  Created by samzapo on 6/24/15.
//
//

#ifndef _Random_h
#define _Random_h
#include <stdio.h>
#include <iostream>
#include <sys/time.h>

/*
 #include <random>
 
 class Generator {
 std::default_random_engine generator;
 std::normal_distribution<double> distribution;
 double min;
 double max;
 public:
 Generator(double mean, double stddev, double min, double max):
 distribution(mean, stddev), min(min), max(max)
 {}
 
 // Min and max are 3 std above and below the mean
 Generator(double min, double max):
 distribution((min + max) / 2, (max - min) / 6), min(min), max(max)
 {}
 
 double generate() {
 double number;
 do{
 number = this->distribution(generator);
 } while (number < this->min && number > this->max);
 return number;
 }
 };
 */

//*
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
// free the random number generator

/// Gets the current time (as a floating-point number)
static double get_current_time()
{
  //const double MICROSEC = 1.0/1000000;
  timeval t;
  gettimeofday(&t, NULL);
  //double stime = (double) t.tv_sec + (double) t.tv_usec * MICROSEC;
  //unsigned long utime = stime * 1000000;
  return (unsigned long) t.tv_sec * 1000000 + (unsigned long) t.tv_usec;
}


class Generator {
  unsigned long seed;
  gsl_rng * generator;
  double min;
  double max;
  double mu;
  double sigma;
public:
  Generator(double mean, double stddev, double xmin, double xmax):
  min(xmin),max(xmax),mu(mean),sigma(stddev)
  {
    seed = get_current_time();
    generator =  gsl_rng_alloc( gsl_rng_default );
    if( generator == NULL ) std::cout << "failed to initialize rng\n";
    gsl_rng_set( generator, seed );
  }
  
  // Min and max are 3 std above and below the mean
  Generator(double xmin, double xmax):
  min(xmin),max(xmax),mu((xmin + xmax) / 2),sigma((xmax - xmin) / 6)
  {
    seed = get_current_time();
    generator =  gsl_rng_alloc( gsl_rng_default );
    if( generator == NULL ) std::cout << "failed to initialize rng\n";
    gsl_rng_set( generator, seed );
  }
  
  ~Generator(){
    gsl_rng_free( generator );
  }
  
  double generate() {
    double number;
    do{
      number = gsl_ran_gaussian( generator, sigma ) + mu;
    } while (number < this->min && number > this->max);
    return number;
  }
};
//*/
#endif
