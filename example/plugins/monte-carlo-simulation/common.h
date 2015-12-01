#ifndef _MC_COMMON_H_
#define _MC_COMMON_H_

#include <vector>
#include <string>

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
#endif //_MC_COMMON_H_
