
#include <stdio.h>
#include <boost/shared_ptr.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>

#ifdef USE_ZMQ
#include <reveal/core/connection.h>
//-----------------------------------------------------------------------------
namespace Reveal {
  //-----------------------------------------------------------------------------
  namespace Core {
    //-----------------------------------------------------------------------------
    typedef boost::shared_ptr<connection_c> connection_ptr;
    //-----------------------------------------------------------------------------
  } // namespace Core
    //-----------------------------------------------------------------------------
} // namespace Reveal
  //-----------------------------------------------------------------------------

namespace Pacer {
namespace Service {
class Connection {
  
  virtual int init(const std::string& port_id) = 0;
  
  
protected:
  Reveal::Core::connection_ptr _connection;
  
  inline int read( std::string& message){
    assert(_connection);
    if( _connection->read( message ) != Reveal::Core::connection_c::ERROR_NONE ) {
      printf( "ERROR: connection failed to read reply\nExiting\n" );
      return 1;
    }
    printf( "received message: %s\n", message.c_str() );
    return 0;
  }
  
  inline int write( const std::string& message){
    assert(_connection);

    printf( "sending message: %s\n", message.c_str() );
    if( _connection->write( message ) != Reveal::Core::connection_c::ERROR_NONE) {
      printf( "ERROR: connection failed to write request\nExiting\n" );
      return 1;
    }
    return 0;
  }
};

class Server : public Connection {
public:
  Server(){  }
  
  Server(const std::string& port_id){
    init(port_id);
  }
  
  Server(const unsigned& port){
    init(port);
  }
  
  inline int serve( std::string& request ) {
    read( request);
    return 0;
  }
  
  inline int respond( const std::string& reply ) {
    write( reply);
    return 0;
  }
  
private:
  int init(const std::string& port_id){
    if(_connection)
      printf( "Connection %s already exists" , port_id.c_str() );
    
    _connection = Reveal::Core::connection_ptr( new Reveal::Core::connection_c(Reveal::Core::connection_c::IPC_SERVER, port_id ) );
    
    if( _connection->open() != Reveal::Core::connection_c::ERROR_NONE ) {
      printf( "planner failed to open server ipc connection\nExiting\n" );
      return 1;
    }
    return 0;
  }
  int init(const unsigned& port){
    if(_connection)
      printf( "Connection %s already exists" , port_id.c_str() );
    
    _connection = Reveal::Core::connection_ptr( new Reveal::Core::connection_c(Reveal::Core::connection_c::TCP_SERVER, port ) );
    
    if( _connection->open() != Reveal::Core::connection_c::ERROR_NONE ) {
      printf( "planner failed to open server ipc connection\nExiting\n" );
      return 1;
    }
    return 0;
  }
  
};

class Client : public Connection {
public:
  Client(){ }
  
  Client(const std::string& port_id){
    init(port_id);
  }
  Client(const std::string& host, const unsigned& port){
    init(host,port);
  }

  inline int request( std::string& request_reply ) {
    write( request_reply);
    read( request_reply);
    return 0;
  }
  
private:
  int init(const std::string& port_id){
    if(_connection)
      printf( "Connection %s already exists" , port_id.c_str() );
    
    _connection = Reveal::Core::connection_ptr( new Reveal::Core::connection_c( Reveal::Core::connection_c::IPC_CLIENT, port_id ));  //< the connection to the planner
    
    if( _connection->open() != Reveal::Core::connection_c::ERROR_NONE ) {
      printf( "simulator failed to open client ipc connection\nExiting\n" );
      return 1;
    }
    return 0;
  }
  int init(const std::string& host, const unsigned& port){
    if(_connection)
      printf( "Connection %s already exists" , port_id.c_str() );
    
    _connection = Reveal::Core::connection_ptr( new Reveal::Core::connection_c( host, port ));  //< the connection to the planner
    
    if( _connection->open() != Reveal::Core::connection_c::ERROR_NONE ) {
      printf( "simulator failed to open client ipc connection\nExiting\n" );
      return 1;
    }
    return 0;
  }
};

}
}
#endif

#include <time.h>
static int sleep_duration(double duration){
  timespec req,rem;
  int seconds = duration;
  req.tv_nsec = (duration - (double)seconds) * 1.0e+9;
  req.tv_sec = seconds;
  nanosleep(&req,&rem);
  return 0;//( ( (double) rem.tv_nsec / 1.0e+9 ) + (double) rem.tv_sec);
}

#include <boost/algorithm/string.hpp>
#include <vector>
#include <string>
#include <sstream>
#include <unistd.h>
#include <fcntl.h>

void tick( int sec = 0, int nsec = 1){
  struct timespec req,rem;
  req.tv_sec = 0;
  req.tv_nsec = 1;
  nanosleep(&req,&rem);
}

#include <sstream>

template <typename T>
static std::string SSTR(T x)
{
  std::ostringstream oss;
  oss << std::dec << x;
  return oss.str();
}

static int MESSAGE_SIZE = 32768;
inline char* const* param_array( std::vector< std::string >& params ) {
  
  const char** pa = (const char**)malloc( sizeof(char*) * (params.size() + 1) );
  for( unsigned i = 0; i < params.size(); i++ ) {
    pa[i] = (const char*)params[i].c_str();
  }
  pa[ params.size() ] = NULL;
  
  return (char* const*) pa;
}

inline char** param_array_noconst( std::vector< std::string >& params ) {
  
  char** pa = (char**)malloc( sizeof(char*) * (params.size() + 1) );
  for( unsigned i = 0; i < params.size(); i++ ) {
    pa[i] = (char*)params[i].c_str();
  }
  pa[ params.size() ] = NULL;
  
  return (char**) pa;
}
