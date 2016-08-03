#ifndef _MC_COMMON_H_
#define _MC_COMMON_H_

#include <boost/algorithm/string.hpp>
#include <vector>
#include <string>
#include <sstream>
#include <unistd.h>
#include <fcntl.h>

template <typename T>
static std::string SSTR(T x)
{
  std::ostringstream oss;
  oss << std::dec << x;
  return oss.str();
}



//#include <deque>
//static char** param_array_noconst( std::deque< std::string >& params ) {
//  
//  char** pa = (char**)malloc( sizeof(char*) * (params.size() + 1) );
//  int N = params.size();
//  for( unsigned i = 0; i < N; i++ ) {
//    pa[i] = (char*)params.pop_back().c_str();
//  }
//  pa[ N ] = NULL;
//  
//  return (char**) pa;
//}

#endif //_MC_COMMON_H_
