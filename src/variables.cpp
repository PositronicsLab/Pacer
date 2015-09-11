/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>


#include <Moby/XMLTree.h>
#include <Moby/XMLReader.h>
#include <Ravelin/Origin3d.h>
#include <Ravelin/VectorNd.h>
#include <Pacer/controller.h>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
using boost::shared_ptr;
using Moby::XMLTree;
using Moby::XMLTreePtr;
using Moby::XMLAttrib;

boost::shared_ptr<Pacer::Robot> robot_ptr;

constexpr unsigned int str2int(const char* str, int h = 0)
{
    return !str[h] ? 5381 : (str2int(str, h+1)*33) ^ str[h];
}

static int str2bool(const std::string& str)
{
  if(str.compare(std::string("true")) == 0)
    return 1;
  else if (str.compare(std::string("false")) == 0)
    return 0;

  assert(false);
}

std::vector<std::string> split(std::string const &input) {
    std::istringstream buffer(input);
    std::vector<std::string> ret((std::istream_iterator<std::string>(buffer)),
                                 std::istream_iterator<std::string>());
    return ret;
}

//template <class X>
//X& var(std::string tag){
//  if(CVarUtils::CVarExists(tag))
//    return Utility::get_variable<X>(tag);
//  assert(CVarUtils::CVarExists(tag));
//  return X();
//}


void process_tag(std::string tag,shared_ptr<const XMLTree> node){
    // do something with the current node instead of System.out
//    OUT_LOG(logDEBUG1) << "processing : " << tag ;

    std::list<XMLTreePtr> nl = node->children;
    for (std::list<XMLTreePtr>::iterator  i = nl.begin(); i != nl.end(); i++) {
        XMLTreePtr n = *i;
        if(n->children.size() == 0){
          XMLAttrib* a = n->get_attrib("type");
          std::string data_type = "no-type";
          if(a)
            data_type = a->get_string_value();

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
                  robot_ptr->set_data<std::vector<std::string> >(tag+n->name,elements);
              }
              else{
                  robot_ptr->set_data<std::string>(tag+n->name,elements[0]);
              }
            }
            else if(data_type.compare("double") == 0){
              if(elements.size()>1)
              {
                std::vector<double> typed_elements;
                typed_elements.reserve(elements.size());
                std::transform(elements.begin(), elements.end(), std::back_inserter(typed_elements),
                        [](std::string const& val) {return std::stod(val);});
                  robot_ptr->set_data<std::vector<double> >(tag+n->name,typed_elements );
              }
              else{
                  robot_ptr->set_data<double>(tag+n->name,std::stod(elements[0]));
              }
            }
            else if(data_type.compare("bool") == 0){
              if(elements.size()>1)
              {
                std::vector<int> typed_elements;
                typed_elements.reserve(elements.size());
                std::transform(elements.begin(), elements.end(), std::back_inserter(typed_elements),
                        [](std::string const& val) {return str2bool(val);});
                  robot_ptr->set_data<std::vector<int> >(tag+n->name,typed_elements);
              }
              else{
                  robot_ptr->set_data<int>(tag+n->name,str2bool(elements[0]));
              }
            }
            else if(data_type.compare("string vector") == 0){
                  robot_ptr->set_data<std::vector<std::string> >(tag+n->name,elements);
            }
            else if(data_type.compare("double vector") == 0){
                std::vector<double> typed_elements;
                typed_elements.reserve(elements.size());
                std::transform(elements.begin(), elements.end(), std::back_inserter(typed_elements),
                        [](std::string const& val) {return std::stod(val);});
                  robot_ptr->set_data<std::vector<double> >(tag+n->name,typed_elements );
            }
            else if(data_type.compare("bool vector") == 0){
                std::vector<int> typed_elements;
                typed_elements.reserve(elements.size());
                std::transform(elements.begin(), elements.end(), std::back_inserter(typed_elements),
                        [](std::string const& val) {return str2bool(val);});
                  robot_ptr->set_data<std::vector<int> >(tag+n->name,typed_elements);
            }
            else {
              OUT_LOG(logINFO) << tag+n->name << "<" <<  data_type << "> = " <<  n->content << "\n"  << data_type << " is not a valid type!" ;
              assert(false);
            }
        } else {
          process_tag(tag+n->name+".",n);
        }
    }
}

void Pacer::Robot::load_variables(std::string fname,boost::shared_ptr<Pacer::Robot> robot_p){
    shared_ptr<const XMLTree> root_tree = XMLTree::read_from_xml(fname);
    robot_ptr = robot_p;
    process_tag("",root_tree);
}
