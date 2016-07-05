#include "exchange.h"

#include "reveal/core/link.h"
#include "reveal/core/joint.h"

//----------------------------------------------------------------------------
namespace Pacer {
//----------------------------------------------------------------------------
namespace Messaging {
//----------------------------------------------------------------------------
exchange_c::exchange_c( void ) {
  reset();
}

//----------------------------------------------------------------------------
exchange_c::~exchange_c( void ) {

}

//----------------------------------------------------------------------------
bool exchange_c::open( void ) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  return true;
}

//----------------------------------------------------------------------------
void exchange_c::close( void ) {
  google::protobuf::ShutdownProtobufLibrary();
}

//----------------------------------------------------------------------------
void exchange_c::reset( void ) {
  _origin = ORIGIN_UNDEFINED;
  _type = TYPE_UNDEFINED;
  _error = ERROR_NONE;
}

//----------------------------------------------------------------------------
bool exchange_c::build( std::string& serialized_message ) {
  bool passed = false;
  Pacer::Messages::Sample::Message msg = Pacer::Messages::Sample::Message();

  // build the authorization component of the message

  // - build the rest of the message -
  switch( _origin ) {
  case ORIGIN_ADMIN:
    passed = build_admin_message( &msg );
    printf( "built message from the admin\n" );
    break;
  case ORIGIN_WORKER:
    passed = build_worker_message( &msg );
    printf( "built message from the worker\n" );
    break;
  default:
    passed = false;
    break;
  }
  // trap any error from building the message
  if( !passed ) return false;

  msg.SerializeToString( &serialized_message );

  return true;
}

//----------------------------------------------------------------------------
bool exchange_c::parse( const std::string& serialized_message ) {
  Pacer::Messages::Sample::Message msg;
  bool passed = false;

  // reset to clear any residual references maintained within this instance
  reset();

  // parse the serialized message into the protocol data structure
  msg.ParseFromString( serialized_message );

  // map the origin component of the message to the class instance
  passed = map_origin( &msg );
  if( !passed ) return false;

  // map the type component of the message to the class instance
  passed = map_type( &msg );
  if( !passed ) return false;

  // map the request component of the message to the class instance
  passed = map_request( &msg );
  if( !passed ) return false;

  // map the response component of the message to the class instance
  passed = map_response( &msg );
  if( !passed ) return false;

  // map the command component of the message to the class instance
  passed = map_command( &msg );
  if( !passed ) return false;

  // map the authorization component of the message to the class instance
//  passed = map_authorization( &msg );
//  if( !passed ) return false;

  // map the rest of the message body
  switch( _origin ) {
  case ORIGIN_ADMIN:
    passed = map_admin_message( &msg );
    break;
  case ORIGIN_WORKER:
    passed = map_worker_message( &msg );
    break;
  default:
    passed = false;
    break;
  }

  return passed;
}

//----------------------------------------------------------------------------
void exchange_c::set_origin( exchange_c::origin_e origin ) {
  _origin = origin;
}

//----------------------------------------------------------------------------
exchange_c::origin_e exchange_c::get_origin( void ) {
  return _origin;
}

//----------------------------------------------------------------------------
void exchange_c::set_type( exchange_c::type_e type ) {
  _type = type;
}

//----------------------------------------------------------------------------
exchange_c::type_e exchange_c::get_type( void ) {
  return _type;
}

//----------------------------------------------------------------------------
void exchange_c::set_response( exchange_c::response_e response ) {
  _response = response;
}

//----------------------------------------------------------------------------
exchange_c::response_e exchange_c::get_response( void ) {
  return _response;
}
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
bool exchange_c::map_origin( Pacer::Messages::Sample::Message* msg ) {
  //transport_exchange_c::error_e error;

  if( msg->header().origin() == Pacer::Messages::Sample::Message::ADMIN ) {
    _origin = ORIGIN_ADMIN;
  } else if( msg->header().origin() == Pacer::Messages::Sample::Message::WORKER ) {
    _origin = ORIGIN_WORKER;
  }
  // Note: pure else case cannot happen

  return true;
}

//----------------------------------------------------------------------------
bool exchange_c::map_type( Pacer::Messages::Sample::Message* msg ) {

  if( msg->header().type() == Pacer::Messages::Sample::Message::RESPONSE ) {
    _type = TYPE_RESPONSE;
  } else if( msg->header().type() == Pacer::Messages::Sample::Message::REQUEST ) {
    _type = TYPE_REQUEST;
  } else if( msg->header().type() == Pacer::Messages::Sample::Message::COMMAND ) {
    _type = TYPE_COMMAND;
  } else {
    // UNDEFINED CASE
    //return false;
  }

  return true;
}

  //----------------------------------------------------------------------------
  bool exchange_c::map_request( Pacer::Messages::Sample::Message* msg ) {
    
    if( msg->header().request() == Pacer::Messages::Sample::Message::DATA ) {
      _request = REQUEST_DATA;
    } else if( msg->header().request() == Pacer::Messages::Sample::Message::PING ) {
        _request = REQUEST_PING;
    } else {
      // UNDEFINED CASE
      //return false;
    }
    
    return true;
  }
  
  
//----------------------------------------------------------------------------
bool exchange_c::map_response( Pacer::Messages::Sample::Message* msg ) {

  if( msg->header().response() == Pacer::Messages::Sample::Message::ISSUED ) {
    _response = RESPONSE_ISSUED;
  } else if( msg->header().response() == Pacer::Messages::Sample::Message::DENIED ) {
    _response = RESPONSE_DENIED;
  } else {
    // UNDEFINED CASE
    //return false;
  }

  return true;
}

  //----------------------------------------------------------------------------
  bool exchange_c::map_command( Pacer::Messages::Sample::Message* msg ) {
    
    if( msg->header().command() == Pacer::Messages::Sample::Message::START ) {
      _command = COMMAND_START;
    } else if( msg->header().command() == Pacer::Messages::Sample::Message::WAIT ) {
      _command = COMMAND_WAIT;
    } else if( msg->header().command() == Pacer::Messages::Sample::Message::RESET ) {
      _command = COMMAND_RESET;
    } else if( msg->header().command() == Pacer::Messages::Sample::Message::EXIT ) {
      _command = COMMAND_EXIT;
    } else {
      // UNDEFINED CASE
      //return false;
    }
    
    return true;
  }
  
  bool exchange_c::map_worker_message( Pacer::Messages::Sample::Message* msg ) {
    return true;
  }

  bool exchange_c::map_admin_message( Pacer::Messages::Sample::Message* msg ) {
    return true;
  }
  
//----------------------------------------------------------------------------
bool exchange_c::build_worker_message( Pacer::Messages::Sample::Message* msg ) {

  //transport_exchange_c::error_e error;
  Pacer::Messages::Sample::Message::Header* header = msg->mutable_header();

  header->set_origin( Pacer::Messages::Sample::Message::WORKER );

//  if( _type == TYPE_REQUEST ) {
//    header->set_type( Pacer::Messages::Sample::Message::REQUEST );
//    header->set_error( Pacer::Messages::Sample::Message::ERROR_NONE );
//    // write_request( msg );
//  } else
    if( _type == TYPE_RESPONSE ) {
    header->set_type( Pacer::Messages::Sample::Message::RESPONSE );
    header->set_error( Pacer::Messages::Sample::Message::ERROR_NONE );
    // write_response( msg );
  }
//  else if( _type == TYPE_COMMAND ) {
//    header->set_type( Pacer::Messages::Sample::Message::COMMAND );
//    header->set_error( Pacer::Messages::Sample::Message::ERROR_NONE );
//    // write_command( msg );
//  }
  else {
    return false;
  }

  return true;
}

//----------------------------------------------------------------------------
bool exchange_c::build_admin_message( Pacer::Messages::Sample::Message* msg ) {

  //transport_exchange_c::error_e error;
  Pacer::Messages::Sample::Message::Header* header = msg->mutable_header();

  header->set_origin( Pacer::Messages::Sample::Message::ADMIN );

  
    if( _type == TYPE_REQUEST ) {
      header->set_type( Pacer::Messages::Sample::Message::REQUEST );
      header->set_error( Pacer::Messages::Sample::Message::ERROR_NONE );
      // write_request( msg );
    }
//  else if( _type == TYPE_RESPONSE ) {
//    header->set_type( Pacer::Messages::Sample::Message::RESPONSE );
//    header->set_error( Pacer::Messages::Sample::Message::ERROR_NONE );
//    // write_response( msg );
//  }
    else if( _type == TYPE_COMMAND ) {
      header->set_type( Pacer::Messages::Sample::Message::COMMAND );
      header->set_error( Pacer::Messages::Sample::Message::ERROR_NONE );
      // write_command( msg );
    }
  else {
    return false;
  }
  //return ERROR_NONE;
  return true;
}
//----------------------------------------------------------------------------
} // namespace Messaging
//----------------------------------------------------------------------------
} // namespace Pacer
//----------------------------------------------------------------------------
