/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <CVars/CVar.h>

#include <Moby/XMLTree.h>
#include <Moby/XMLReader.h>
#include <Ravelin/Origin3d.h>
#include <Ravelin/VectorNd.h>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
using boost::shared_ptr;
using Moby::XMLTree;
using Moby::XMLTreePtr;
using Moby::XMLAttrib;

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
//    return CVarUtils::GetCVarRef<X>(tag);
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

          std::vector<std::string> elements = split(n->content);

//          OUT_LOG(logDEBUG1) << tag+n->name << "<" <<  data_type << "> = " <<  n->content;
          if(elements.size() == 0)
            continue;
          switch(str2int(data_type.c_str())){
            case (str2int("string")):
              if(elements.size()>1){
                try{
                  CVarUtils::CreateCVar<std::vector<std::string> >(tag+n->name,elements, help );
                } catch (CVarUtils::CVarException& e){
                  OUT_LOG(logDEBUG1) << "\t -- Already set, resetting" ;
                  CVarUtils::SetCVar<std::vector<std::string> >(tag+n->name,elements);
                }
              }
              else{
                try{
                  CVarUtils::CreateCVar<std::string>(tag+n->name,elements[0], help );
                } catch (CVarUtils::CVarException& e){
                  OUT_LOG(logDEBUG1) << "\t -- Already set, resetting" ;
                  CVarUtils::SetCVar<std::string>(tag+n->name,elements[0]);
                }
              }
            break;
            case (str2int("double")):
              if(elements.size()>1)
              {
                std::vector<double> typed_elements;
                typed_elements.reserve(elements.size());
                std::transform(elements.begin(), elements.end(), std::back_inserter(typed_elements),
                        [](std::string const& val) {return std::stod(val);});
                try{
                  CVarUtils::CreateCVar<std::vector<double> >(tag+n->name,typed_elements, help );
                } catch (CVarUtils::CVarException& e){
                  OUT_LOG(logDEBUG1) << "\t -- Already set, resetting" ;
                  CVarUtils::SetCVar<std::vector<double> >(tag+n->name,typed_elements );
                }
              }
              else{
                try{
                  CVarUtils::CreateCVar<double>(tag+n->name,std::stod(elements[0]), help );
                } catch (CVarUtils::CVarException& e){
                  OUT_LOG(logDEBUG1) << "\t -- Already set, resetting" ;
                  CVarUtils::SetCVar<double>(tag+n->name,std::stod(elements[0]));
                }
              }
            break;
            case (str2int("bool")):
              if(elements.size()>1)
              {
                std::vector<int> typed_elements;
                typed_elements.reserve(elements.size());
                std::transform(elements.begin(), elements.end(), std::back_inserter(typed_elements),
                        [](std::string const& val) {return str2bool(val);});
                try{
                  CVarUtils::CreateCVar<std::vector<int> >(tag+n->name,typed_elements, help );
                } catch (CVarUtils::CVarException& e){
                  OUT_LOG(logDEBUG1) << "\t -- Already set, resetting" ;
                  CVarUtils::SetCVar<std::vector<int> >(tag+n->name,typed_elements);
                }
              }
              else{
                try{
                  CVarUtils::CreateCVar<int>(tag+n->name,str2bool(elements[0]), help );
                } catch (CVarUtils::CVarException& e){
                  OUT_LOG(logDEBUG1) << "\t -- Already set, resetting" ;
                  CVarUtils::SetCVar<int>(tag+n->name,str2bool(elements[0]));
                }
              }
            break;
            default:
              OUT_LOG(logINFO) << tag+n->name << "<" <<  data_type << "> = " <<  n->content << "\n"  << data_type << " is not a valid type!" ;
              assert(false);
            break;
          }
        } else {
          process_tag(tag+n->name+".",n);
        }
    }
}

void Utility::load_variables(std::string fname){
    shared_ptr<const XMLTree> root_tree = XMLTree::read_from_xml(fname);
    process_tag("",root_tree);
}

std::vector<double>& Utility::get_variable(const char* tag,std::vector<double>& val){
  return (val = CVarUtils::GetCVar<std::vector<double> >(tag));
}
double& Utility::get_variable(const char* tag,double& val){
  return (val = CVarUtils::GetCVar<double>(tag));
}
std::vector<std::string>& Utility::get_variable(const char* tag,std::vector<std::string>& val){
  return (val = CVarUtils::GetCVar<std::vector<std::string> >(tag));
}
std::string& Utility::get_variable(const char* tag,std::string& val){
  return (val = CVarUtils::GetCVar<std::string>(tag));
}
std::vector<int>& Utility::get_variable(const char* tag,std::vector<int>& val){
  return (val = CVarUtils::GetCVar<std::vector<int> >(tag));
}
int& Utility::get_variable(const char* tag,int& val){
  return (val = CVarUtils::GetCVar<int>(tag));
}

