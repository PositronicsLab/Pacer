/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>


#include <Ravelin/Origin3d.h>
#include <Ravelin/VectorNd.h>
#include <Pacer/controller.h>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
using boost::shared_ptr;
using Moby::XMLTree;
using Moby::XMLTreePtr;
using Moby::XMLAttrib;

static bool str2bool(const std::string& str)
{
  if(str.compare(std::string("true")) == 0)
    return 1;
  else if (str.compare(std::string("false")) == 0)
    return 0;
  
  assert(false);
}

static int str2int(const std::string& str)
{
  return std::atoi(str.c_str());
}

std::vector<std::string> split(std::string const &input) {
  std::istringstream buffer(input);
  std::vector<std::string> ret((std::istream_iterator<std::string>(buffer)),
                               std::istream_iterator<std::string>());
  return ret;
}

// Replacing: [](std::string const& val) {return std::stod(val);});
double string_to_double(std::string const& val){
  char * pEnd;
  return std::strtod(val.c_str(),&pEnd);
}

// Replacing: [](std::string const& val) {return std::stod(val);});
bool string_to_bool(std::string const& val){
  return str2bool(val);
}

// Replacing: [](std::string const& val) {return std::stod(val);});
int string_to_int(std::string const& val){
  return str2int(val);
}

void Pacer::Controller::process_tag(std::string tag,shared_ptr<const XMLTree> node){
  // do something with the current node instead of System.out
  //    OUT_LOG(logDEBUG1) << "processing : " << tag ;
  
  std::list<XMLTreePtr> nl = node->children;
  for (std::list<XMLTreePtr>::iterator  i = nl.begin(); i != nl.end(); i++) {
    XMLTreePtr n = *i;
    if(n->children.size() != 0){
      process_tag(tag+n->name+".",n);
    } else {
      XMLAttrib* a = n->get_attrib("type");
      std::string data_type = "no-type";
      if(a){
        data_type = a->get_string_value();
      } else {
        OUT_LOG(logINFO) << tag+n->name << "<" <<  data_type << "> = " <<  n->content << "\n"  << data_type << " is not a valid type!" ;
        throw std::runtime_error("variable doesn't have a 'type': {string, double, bool, vector string, vector double, vector bool, file}");
      }
      
      std::string help = "no tooltip";
      a = n->get_attrib("help");
      if(a)
        help = a->get_string_value();
      
      //          OUT_LOG(logDEBUG1) << tag+n->name << " <-- (" <<  data_type << ") " <<  n->content;
      std::vector<std::string> elements = split(n->content);
      
      //          OUT_LOG(logDEBUG1) << tag+n->name << "<" <<  data_type << "> = " <<  n->content;
      if(elements.size() == 0)
        continue;
      if(data_type.compare("string") == 0){
        if(elements.size()>1){
          set_data<std::vector<std::string> >(tag+n->name,elements);
        }
        else{
          set_data<std::string>(tag+n->name,elements[0]);
        }
      }
      else if(data_type.compare("double") == 0){
        if(elements.size()>1)
        {
          std::vector<double> typed_elements;
          typed_elements.reserve(elements.size());
          std::transform(elements.begin(), elements.end(), std::back_inserter(typed_elements),
                         string_to_double);
          set_data<std::vector<double> >(tag+n->name,typed_elements );
        }
        else{
          char * pEnd;
          set_data<double>(tag+n->name,std::strtod(elements[0].c_str(),&pEnd));
        }
      }
      else if(data_type.compare("bool") == 0){
        if(elements.size()>1)
        {
          std::vector<bool> typed_elements;
          typed_elements.reserve(elements.size());
          std::transform(elements.begin(), elements.end(), std::back_inserter(typed_elements),
                         string_to_bool);
          set_data<std::vector<bool> >(tag+n->name,typed_elements);
        }
        else{
          set_data<bool>(tag+n->name,str2bool(elements[0]));
        }
      }
      else if(data_type.compare("int") == 0){
        if(elements.size()>1)
        {
          std::vector<int> typed_elements;
          typed_elements.reserve(elements.size());
          std::transform(elements.begin(), elements.end(), std::back_inserter(typed_elements),
                         string_to_int);
          set_data<std::vector<int> >(tag+n->name,typed_elements);
        }
        else{
          set_data<int>(tag+n->name,str2int(elements[0]));
        }
      }
      else if(data_type.compare("string vector") == 0){
        set_data<std::vector<std::string> >(tag+n->name,elements);
      }
      else if(data_type.compare("double vector") == 0){
        std::vector<double> typed_elements;
        typed_elements.reserve(elements.size());
        std::transform(elements.begin(), elements.end(), std::back_inserter(typed_elements),
                       string_to_double);
        set_data<std::vector<double> >(tag+n->name,typed_elements );
      }
      else if(data_type.compare("bool vector") == 0){
        std::vector<bool> typed_elements;
        typed_elements.reserve(elements.size());
        std::transform(elements.begin(), elements.end(), std::back_inserter(typed_elements),
                       string_to_bool);
        set_data<std::vector<bool> >(tag+n->name,typed_elements);
      }
      else if(data_type.compare("int vector") == 0){
        std::vector<int> typed_elements;
        typed_elements.reserve(elements.size());
        std::transform(elements.begin(), elements.end(), std::back_inserter(typed_elements),
                       string_to_int);
        set_data<std::vector<int> >(tag+n->name,typed_elements);
      }
      else if(data_type.compare("file") == 0){
        for(int i=0;i<elements.size();i++)
          load_variables(elements[i],tag);
        
      }
      else {
        OUT_LOG(logINFO) << tag+n->name << "<" <<  data_type << "> = " <<  n->content << "\n"  << data_type << " is not a valid type!" ;
        assert(false);
      }
    }
  }
}

void Pacer::Controller::load_variables(std::string fname, std::string root){
  shared_ptr<const XMLTree> root_tree = XMLTree::read_from_xml(fname);
  process_tag(root,root_tree);
}

bool Pacer::Robot::set_data_internal(std::string n, boost::any to_append){
  bool new_var = true;
#ifdef LOG_TO_FILE
  OUT_LOG(logINFO) << "\t" << n << " has type '" << to_append.type().name() << "'";
#endif
  
#ifdef USE_THREADS
  pthread_mutex_lock(&_data_map_mutex);
#endif
  int map_size = _data_map.size();
  _data_map[n] = to_append;
  new_var = (map_size != _data_map.size());
#ifdef USE_THREADS
  pthread_mutex_unlock(&_data_map_mutex);
#endif
  return new_var;
}