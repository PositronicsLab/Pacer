#include <quadruped.h>
#include <CVars/CVar.h>

#include <Moby/XMLTree.h>
#include <Moby/XMLReader.h>
#include <Ravelin/Origin3d.h>
#include <Ravelin/VectorNd.h>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
using boost::shared_ptr;
using namespace Moby;

constexpr unsigned int str2int(const char* str, int h = 0)
{
    return !str[h] ? 5381 : (str2int(str, h+1)*33) ^ str[h];
}

static int str2bool(const std::string& str)
{
    return (str.compare(std::string("true")) == 0)? 1 : 0;
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
    std::cout << "processing : " << tag << std::endl;

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

//          std::vector<std::string> elements;
//          boost::algorithm::split(elements, parts[0],  boost::algorithm::is_any_of(" "));
          std::vector<std::string> elements = split(n->content);

          std::cout << tag+n->name << "<" <<  data_type << "> = " <<  n->content << std::endl;
          if(elements.size() == 0)
            continue;
          switch(str2int(data_type.c_str())){
            case (str2int("string")):
              if(elements.size()>1)
                CVarUtils::CreateCVar<std::vector<std::string> >(tag+n->name,elements, help );
              else
                CVarUtils::CreateCVar<std::string>(tag+n->name,elements[0], help );
            break;
            case (str2int("double")):
              if(elements.size()>1)
              {
                std::vector<double> typed_elements;
                typed_elements.reserve(elements.size());
                std::transform(elements.begin(), elements.end(), std::back_inserter(typed_elements),
                        [](std::string const& val) {return std::stod(val);});
                CVarUtils::CreateCVar<std::vector<double> >(tag+n->name,typed_elements, help );
              }
              else
                CVarUtils::CreateCVar<double>(tag+n->name,std::stod(elements[0]), help );
            break;
            case (str2int("bool")):
              if(elements.size()>1)
              {
                std::vector<int> typed_elements;
                typed_elements.reserve(elements.size());
                std::transform(elements.begin(), elements.end(), std::back_inserter(typed_elements),
                        [](std::string const& val) {return str2bool(val);});
                CVarUtils::CreateCVar<std::vector<int> >(tag+n->name,typed_elements, help );
              }
              else
                CVarUtils::CreateCVar<int>(tag+n->name,str2bool(elements[0]), help );
            break;
            default:
              std::cerr << tag+n->name << "<" <<  data_type << "> = " <<  n->content << "\n"  << data_type << " is not a valid type!" << std::endl;
              assert(false);
            break;
          }
        } else {
          process_tag(tag+n->name+".",n);
        }
    }
}

void Quadruped::load_variables(std::string fname){
    shared_ptr<const XMLTree> root_tree = XMLTree::read_from_xml(fname);
    process_tag("",root_tree);
}

std::vector<double>& Quadruped::get_variable(const char* tag,std::vector<double>& val){
  return (val = CVarUtils::GetCVar<std::vector<double>>(tag));
}
double& Quadruped::get_variable(const char* tag,double& val){
  return (val = CVarUtils::GetCVar<double>(tag));
}
std::vector<std::string>& Quadruped::get_variable(const char* tag,std::vector<std::string>& val){
  return (val = CVarUtils::GetCVar<std::vector<std::string>>(tag));
}
std::string& Quadruped::get_variable(const char* tag,std::string& val){
  return (val = CVarUtils::GetCVar<std::string>(tag));
}
std::vector<int>& Quadruped::get_variable(const char* tag,std::vector<int>& val){
  return (val = CVarUtils::GetCVar<std::vector<int>>(tag));
}
int& Quadruped::get_variable(const char* tag,int& val){
  return (val = CVarUtils::GetCVar<int>(tag));
}

