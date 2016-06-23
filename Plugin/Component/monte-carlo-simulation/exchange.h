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

//----------------------------------------------------------------------------
namespace Reveal {
//----------------------------------------------------------------------------
namespace Analytics {
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

  virtual std::string type( void ) { return "analytics"; }

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
    TYPE_RESPONSE = 0,    //< an invalid message
    TYPE_REQUEST,            //< the message is an error notification
    TYPE_COMMAND        //< the message is an attempt to handshake
  };
  
  enum request_e {
    REQUEST_DATA = 0,
    REQUEST_PING
  };

  enum command_e {
    RESPONSE_START = 0,
    RESPONSE_WAIT,
    RESPONSE_RESET,
    RESPONSE_EXIT
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

  /// Sets the response of the message
  /// @param response the response of the message
  void set_response( response_e response );
  /// Gets the response of the message
  /// @return the response of the message
  response_e get_response( void );

private:
  origin_e            _origin;         //< the message's origin
  type_e              _type;           //< the message's type
  response_e          _response;

  bool map_origin( Pacer::Messages::Sample::Message* message );
  bool map_type( Pacer::Messages::Sample::Message* message );
  bool map_response( Pacer::Messages::Sample::Message* message );

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
