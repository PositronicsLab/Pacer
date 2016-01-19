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

#ifdef USE_GSL
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
// free the random number generator

/// Gets the current time (as a floating-point number)
static const double usec_sec = 1000000;
static double get_current_time()
{
  //const double MICROSEC = 1.0/usec_sec;
  timeval t;
  gettimeofday(&t, NULL);
  //double stime = (double) t.tv_sec + (double) t.tv_usec * MICROSEC;
  //unsigned long utime = stime * usec_sec;
  return (unsigned long) t.tv_sec * usec_sec + (unsigned long) t.tv_usec;
}


class Generator {
  unsigned long seed;
  gsl_rng * generator;
  
  double min;
  double max;
  double mu;
  double sigma;
  enum DistributionType{
    none,
    gaussian,
    uniform
  } distribution_type;
  bool first_sample;

public:
  Generator(){
    seed = get_current_time();
    generator =  gsl_rng_alloc( gsl_rng_default );
    if( generator == NULL ) std::cout << "failed to initialize rng\n";
    gsl_rng_set( generator, seed );
    distribution_type = none;
    first_sample = true;
  }
  
  void set_gaussian(double mu, double sigma)
  {
    distribution_type = gaussian;
    this->min = mu - 3.0*sigma;
    this->max = mu + 3.0*sigma;
    this->mu = mu;
    this->sigma = sigma;
  }

  //  void set_gaussian(double mu, double sigma, double min=-std::numeric_limits<float>::max(), double max=std::numeric_limits<float>::max())
  void set_gaussian(double mu, double sigma, double min, double max)
  {
    distribution_type = gaussian;
    this->min = min;
    this->max = max;
    this->mu = mu;
    this->sigma = sigma;
  }
  
  
  void set_gaussian_from_limits(double min, double max, double max_stddev = 3.0)
  {
    distribution_type = gaussian;
    this->min = min;
    this->max = max;
    this->mu = (min + max) / 2.0;
    this->sigma = (max - min) / (max_stddev*2.0);
  }
  
  void set_uniform(double min, double max)
  {
    distribution_type = uniform;
    this->min = min;
    this->max = max;
    this->mu = (min+max) / 2.0;
  }
  
  ~Generator(){
    gsl_rng_free( generator );
  }
  
  double generate() {
    if (first_sample) {
      first_sample = false;
      return mu;
    }
    double number;
    switch(distribution_type) {
      case gaussian:
        do{
          number = gsl_ran_gaussian( generator, sigma ) + mu;
        } while (number < this->min && number > this->max);
        break;
      case uniform:
        number = gsl_rng_uniform(generator) * (max-min) + min;
        break;
      default:
        throw std::runtime_error("'distribution_type' is not set on call to 'generate()'");
    }
    return number;
  }
};
#endif
#endif
