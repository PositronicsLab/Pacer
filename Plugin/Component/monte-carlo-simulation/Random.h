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
#include <Pacer/utilities.h>

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
// free the random number generator

namespace Random{
  
  /// Gets the current time (as a floating-point number)
  static double get_current_time()
  {
    const double usec_sec = 1000000;
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
        OUT_LOG(logDEBUG) << "first_sample, returning average value.";
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
  
  typedef std::pair<std::vector<boost::shared_ptr<Random::Generator> >, std::vector<double> > ParamDefaultPair;
  typedef std::map<std::string, ParamDefaultPair > ParamMap;
  typedef std::map<std::string, std::vector<double> > ParamValueMap;
  
  static void parse_distribution(const std::string& parameters,const boost::shared_ptr<Pacer::Controller> ctrl,std::vector<boost::shared_ptr<Random::Generator> >& generator){
    std::vector<double> min, max, mu, sigma;
    int N = 0;
    bool has_min = false, has_max = false, has_mu = false, has_sigma = false;
    
    std::string distribution_type;
    if(!ctrl->get_data<std::string>(parameters+".distribution", distribution_type))
      throw std::runtime_error("there is no default value OR distribution params for this!: " + parameters);
    
    OUT_LOG(logDEBUG) << parameters << " has a distribution of type: " << distribution_type;
    
    if ((has_min = ctrl->get_data<std::vector<double> >(parameters+".min", min))){
      OUT_LOG(logDEBUG) << "min = " << min;
      N = min.size();
    }
    if ((has_max = ctrl->get_data<std::vector<double> >(parameters+".max", max))){
      OUT_LOG(logDEBUG) << "max = " << max;
      N = max.size();
    }
    if ((has_mu = ctrl->get_data<std::vector<double> >(parameters+".mu", mu))){
      OUT_LOG(logDEBUG) << "mu = " << mu;
      N = mu.size();
    }
    if ((has_sigma = ctrl->get_data<std::vector<double> >(parameters+".sigma", sigma))){
      OUT_LOG(logDEBUG) << "sigma = " << sigma;
      N = sigma.size();
    }
    
    generator.resize(N);
    for (int i=0;i<N;i++) {
      generator[i] = boost::shared_ptr<Random::Generator>(new Random::Generator());
      if(distribution_type.compare("gaussian") == 0){
        if (has_max && has_min && has_mu && has_sigma) {
          generator[i]->set_gaussian(mu[i],sigma[i],min[i],max[i]);
        } else if (has_max && has_min && !has_mu && !has_sigma) {
          generator[i]->set_gaussian_from_limits(min[i],max[i]);
        } else if (has_max && !has_min && !has_mu && !has_sigma) {
          generator[i]->set_gaussian_from_limits(-max[i],max[i]);
        } else if (!has_max && !has_min && has_mu && has_sigma) {
          generator[i]->set_gaussian(mu[i],sigma[i]);
        } else {
          throw std::runtime_error("Not a valid set of params for a GAUSSIAN distribution!");
        }
      } else if(distribution_type.compare("uniform") == 0){
        if (has_max && has_min) {
          generator[i]->set_uniform(min[i],max[i]);
        } else if (has_max && !has_min){
          generator[i]->set_uniform(-max[i],max[i]);
        } else {
          throw std::runtime_error("Not a valid set of params for a UNIFORM distribution!");
        }
      } else {
        throw std::runtime_error("Not a valid distribution!");
      }
    }
  }
  
  // create distribution map
  static void create_distributions(std::string name,const boost::shared_ptr<Pacer::Controller> ctrl,ParamMap& parameter_generator, bool verbose_name = false){
    // Initialize Parameter distributions
    
    // FOR EACH UNCERTAINTY (manufacturing ,state)
    std::vector<std::string> uncertainty_names;
    if(ctrl->get_data<std::vector<std::string> >(name+".id",uncertainty_names)){
      for (int i=0; i<uncertainty_names.size();i++) {
        std::string& uncertainty_name = uncertainty_names[i];
        
        // FOR EACH TYPE (joint, link)
        std::vector<std::string> type_names;
        if(ctrl->get_data<std::vector<std::string> >(name+"."+uncertainty_name+".id",type_names)){
          for (int j=0; j<type_names.size();j++) {
            std::string& type_name = type_names[j];
            
            // FOR EACH OBJECT (joint name, link name)
            std::vector<std::string> object_names;
            if(ctrl->get_data<std::vector<std::string> >(name+"."+uncertainty_name+"."+type_name+".id",object_names)){
              for (int k=0; k<object_names.size();k++) {
                std::string& object_name = object_names[k];
                
                std::vector<std::string> value_names;
                if(ctrl->get_data<std::vector<std::string> >(name+"."+uncertainty_name+"."+type_name+"."+object_name+".id",value_names)){
                  for (int l=0; l<value_names.size();l++) {
                    std::string& value_name = value_names[l];
                    
                    // Create vector of generators and default values
                    std::vector<boost::shared_ptr<Random::Generator> > generator;
                    std::vector<double>                        default_value;
                    // try to use default value
                    if(!ctrl->get_data<std::vector<double> >(name+"."+uncertainty_name+"."+type_name+"."+object_name+"."+value_name,default_value)){
                      // if we're here then there are sub-tags to this parameter (generator params)
                      parse_distribution(name+"."+uncertainty_name+"."+type_name+"."+object_name+"."+value_name,ctrl,generator);
                      
                      OUT_LOG(logDEBUG) << "Created generator for uncertain parameter: "<< object_name << "." << value_name;
                      OUT_LOG(logDEBUG) << "\t FROM: uncertainty."+uncertainty_name+"."+type_name+"."+object_name+"."+value_name;
                    } else {
                      OUT_LOG(logDEBUG) << "Used default for uncertain parameter: "<< object_name << "." << value_name;
                      OUT_LOG(logDEBUG) << "\t FROM: uncertainty."+uncertainty_name+"."+type_name+"."+object_name+"."+value_name;
                      OUT_LOG(logDEBUG) << "\t default: " << default_value;
                    }
                    
                    // error check
                    if( (generator.empty() && default_value.empty()) || (!generator.empty() && !default_value.empty()))
                      throw std::runtime_error("there are default values AND distribution params for this value!");
                    
                    OUT_LOG(logDEBUG) << "parameter: "<< object_name << "." << value_name << " pushed to map.";
                    if(verbose_name){
                      parameter_generator[uncertainty_name+"."+type_name+"."+object_name+"."+value_name] = ParamDefaultPair(generator,default_value);
                    } else {
                      parameter_generator[object_name+"."+value_name] = ParamDefaultPair(generator,default_value);
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }
  
  static void generate_parameters(const ParamMap& parameter_generator,const boost::shared_ptr<Pacer::Controller> ctrl, ParamValueMap& generated_params){
    for(ParamMap::const_iterator it = parameter_generator.begin(); it != parameter_generator.end(); it++){
      if (it->second.first.empty()) {
        for (int i=0;i<it->second.second.size(); i++) {
          double value = it->second.second[i];
          OUT_LOG(logDEBUG) << " " << value;
          generated_params[it->first].push_back(value);
        }
      } else { // Generators created for variable
        for (int i=0;i<it->second.first.size(); i++) {
          double value = it->second.first[i]->generate();
          generated_params[it->first].push_back(value);
        }
      }
    }
  }
}
#endif
