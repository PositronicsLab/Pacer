#ifndef _MC_COMMON_H_
#define _MC_COMMON_H_

#include <boost/algorithm/string.hpp>
#include <vector>
#include <string>
#include <sstream>
#include <unistd.h>
#include <fcntl.h>

static char* const* param_array( std::vector< std::string >& params ) {
  
  const char** pa = (const char**)malloc( sizeof(char*) * (params.size() + 1) );
  for( unsigned i = 0; i < params.size(); i++ ) {
    pa[i] = (const char*)params[i].c_str();
  }
  pa[ params.size() ] = NULL;
  
  return (char* const*) pa;
}

static char** param_array_noconst( std::vector< std::string >& params ) {
  
  char** pa = (char**)malloc( sizeof(char*) * (params.size() + 1) );
  for( unsigned i = 0; i < params.size(); i++ ) {
    pa[i] = (char*)params[i].c_str();
  }
  pa[ params.size() ] = NULL;
  
  return (char**) pa;
}

template <typename T>
static std::string SSTR(T x)
{
  std::ostringstream oss;
  oss << std::dec << x;
  return oss.str();
}


static int MESSAGE_SIZE = 1024;

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
