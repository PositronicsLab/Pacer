#include <string>
#include <map>
#include <vector>
#include <boost/shared_ptr.hpp>

#include <GLConsole/GLConsole.h>
//#include <CVars/CVar.h>

void load_variables(std::string fname);
//template <class X>
//X& var(std::string fname);


extern double
       SIM_MU_COULOMB,
       SIM_MU_VISCOSE,
       SIM_PENALTY_KV,
       SIM_PENALTY_KP;
