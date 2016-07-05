/*-----------------------------------------------------------------------------
author: James R Taylor (jrt@gwu.edu)

transport_exchange.h defines the transport_exchange_c builder that encapsulates
and manages the exchange of Reveal data across the transport protocol.  The
exchange parses serialized protocols into class instances and builds serialized
protocols from class instances
-----------------------------------------------------------------------------*/

#ifndef _EXCHANGE_H_
#define _EXCHANGE_H_

//----------------------------------------------------------------------------

#include <reveal/core/exchange.h>
#include <reveal/core/pointers.h>

#include <Pacer/messages/data.pb.h>
#include <Pacer/messages/sample.pb.h>

#include <string>
#include <map>
#include <vector>

//----------------------------------------------------------------------------
namespace Pacer {
//----------------------------------------------------------------------------
namespace Messaging {
//----------------------------------------------------------------------------

class exchange_c : public Reveal::Core::exchange_c {
public:

  /// Default constructor
  exchange_c( void );

  /// Destructor
  virtual ~exchange_c( void );

  /// Opens the exchange.  Should be done once by the parent process before any
  /// intercommunication is attempted
  virtual bool open( void );

  /// Closes the exchange.  Should be done once by the parent process when all
  /// other processes have stopped intercommunicating
  virtual void close( void ); 

  /// Clears any information buffered in an instantiated exchange  
  virtual void reset( void );

  virtual std::string type( void ) { return "sample"; }

  virtual bool build( std::string& message );
  virtual bool parse( const std::string& message );

  //----------------------------------------------------------------------------
  //----------------------------------------------------------------------------
  /// The enumerated set of message origins
  enum origin_e {
    ORIGIN_UNDEFINED = 0,  //< an invalid message
    ORIGIN_ADMIN,         
    ORIGIN_WORKER          
  };

  /// The enumerated set of message types
  enum type_e {
    TYPE_UNDEFINED = 0,    //< an invalid message
    TYPE_RESPONSE,    //< an invalid message
    TYPE_REQUEST,            //< the message is an error notification
    TYPE_COMMAND        //< the message is an attempt to handshake
  };
  
  enum request_e {
    REQUEST_DATA = 0,
    REQUEST_STATUS
  };
  
  enum error_e {
    ERROR_NONE = 0,
    ERROR_GENERAL,
  };

  enum command_e {
    COMMAND_START = 0,
    COMMAND_WAIT,
    COMMAND_RESET,
    COMMAND_EXIT
  };


  enum response_e {
    RESPONSE_ISSUED = 0,
    RESPONSE_DENIED
  };
  /// Sets the origin of the message
  /// @param origin the origin of the message
  void set_origin( origin_e origin );
  /// Gets the origin of the message
  /// @return the origin of the message
  origin_e get_origin( void );

  /// Sets the type of the message
  /// @param type the type of the message
  void set_type( type_e type );
  /// Gets the type of the message
  /// @return the type of the message
  type_e get_type( void );
  
  /// Sets the error of the message
  /// @param error the error of the message
  void set_error( error_e error );
  /// Gets the error of the message
  /// @return the error of the message
  error_e get_error( void );
  
  /// Sets the request of the message
  /// @param request the request of the message
  void set_request( request_e request );
  /// Gets the request of the message
  /// @return the request of the message
  request_e get_request( void );
  
  /// Sets the response of the message
  /// @param response the response of the message
  void set_response( response_e response );
  /// Gets the response of the message
  /// @return the response of the message
  response_e get_response( void );
  
  /// Sets the command of the message
  /// @param command the command of the message
  void set_command( command_e command );
  /// Gets the command of the message
  /// @return the command of the message
  command_e get_command( void );

private:
  
  origin_e            _origin;         //< the message's origin
  type_e              _type;           //< the message's type
  error_e             _error;
  request_e           _request;
  response_e          _response;
  command_e           _command;
    
  bool map_origin( Pacer::Messages::Sample::Message* message );
  bool map_type( Pacer::Messages::Sample::Message* message );
  bool map_request( Pacer::Messages::Sample::Message* message );
  bool map_response( Pacer::Messages::Sample::Message* message );
  bool map_command( Pacer::Messages::Sample::Message* message );

  bool map_admin_message( Pacer::Messages::Sample::Message* message );
  bool map_worker_message( Pacer::Messages::Sample::Message* message );

  bool build_admin_message( Pacer::Messages::Sample::Message* message );
  bool build_worker_message( Pacer::Messages::Sample::Message* message );
};

//----------------------------------------------------------------------------
} // namespace Analytics
//----------------------------------------------------------------------------
} // namespace Reveal
//----------------------------------------------------------------------------

#endif //  _EXCHANGE_H_
