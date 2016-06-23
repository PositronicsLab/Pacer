#include "exchange.h"

#include "reveal/core/link.h"
#include "reveal/core/joint.h"

//----------------------------------------------------------------------------
namespace Reveal {
//----------------------------------------------------------------------------
namespace Analytics {
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

  _authorization = Reveal::Core::authorization_ptr();
}

//----------------------------------------------------------------------------
bool exchange_c::build( std::string& serialized_message ) {
  bool passed = false;
  Reveal::Messages::Analytics::Message msg = Reveal::Messages::Analytics::Message();

  // build the authorization component of the message
  passed = build_authorization( &msg );
  if( !passed ) return false;

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
  Reveal::Messages::Analytics::Message msg;
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
  passed = map_authorization( &msg );
  if( !passed ) return false;

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
bool exchange_c::map_origin( Reveal::Messages::Analytics::Message* msg ) {
  //transport_exchange_c::error_e error;

  if( msg->header().origin() == Reveal::Messages::Analytics::Message::ADMIN ) {
    _origin = ORIGIN_ADMIN;
  } else if( msg->header().origin() == Reveal::Messages::Analytics::Message::WORKER ) {
    _origin = ORIGIN_WORKER;
  }
  // Note: pure else case cannot happen

  return true;
}

//----------------------------------------------------------------------------
bool exchange_c::map_type( Reveal::Messages::Analytics::Message* msg ) {

  if( msg->header().type() == Reveal::Messages::Analytics::Message::POLL ) {
    _type = TYPE_POLL;
  } else if( msg->header().type() == Reveal::Messages::Analytics::Message::START ) {
    _type = TYPE_START;
  } else if( msg->header().type() == Reveal::Messages::Analytics::Message::WAIT ) {
    _type = TYPE_WAIT;
  } else if( msg->header().type() == Reveal::Messages::Analytics::Message::CLEAN ) {
    _type = TYPE_CLEAN;
  } else if( msg->header().type() == Reveal::Messages::Analytics::Message::EXIT ) {
    _type = TYPE_EXIT;
  } else if( msg->header().type() == Reveal::Messages::Analytics::Message::DATA ) {
    _type = TYPE_DATA;
  } else {
    // UNDEFINED CASE
    //return false;
  }

  return true;
}

//----------------------------------------------------------------------------
bool exchange_c::map_response( Reveal::Messages::Analytics::Message* msg ) {

  if( msg->header().response() == Reveal::Messages::Analytics::Message::RESPONSE_JOB_ISSUED ) {
    _response = RESPONSE_JOB_ISSUED;
  } else if( msg->header().response() == Reveal::Messages::Analytics::Message::RESPONSE_JOB_DENIED ) {
    _response = RESPONSE_JOB_DENIED;
  } else {
    // UNDEFINED CASE
    //return false;
  }

  return true;
}

//----------------------------------------------------------------------------
bool exchange_c::build_worker_message( Reveal::Messages::Analytics::Message* msg ) {

  //transport_exchange_c::error_e error;
  Reveal::Messages::Analytics::Message::Header* header = msg->mutable_header();

  header->set_origin( Reveal::Messages::Analytics::Message::WORKER );

  if( _type == TYPE_ERROR ) {
  } else if( _type == TYPE_DATA ) {
    header->set_type( Reveal::Messages::Analytics::Message::DATA );
  } else if( _type == TYPE_REQUEST ) {
    header->set_type( Reveal::Messages::Analytics::Message::REQUEST );
    header->set_error( Reveal::Messages::Analytics::Message::ERROR_NONE );

    // write_request( msg );

  } else if( _type == TYPE_RESPONSE ) {
    header->set_type( Reveal::Messages::Analytics::Message::RESPONSE );
    header->set_error( Reveal::Messages::Analytics::Message::ERROR_NONE );

    // write_response( msg );

  } else if( _type == TYPE_COMMAND ) {
    header->set_type( Reveal::Messages::Analytics::Message::COMMAND );
    header->set_error( Reveal::Messages::Analytics::Message::ERROR_NONE );

    // write_command( msg );

  } else {
    return false;
  }

  return true;
}

//----------------------------------------------------------------------------
bool exchange_c::build_admin_message( Reveal::Messages::Analytics::Message* msg ) {

  //transport_exchange_c::error_e error;
  Reveal::Messages::Analytics::Message::Header* header = msg->mutable_header();

  header->set_origin( Reveal::Messages::Analytics::Message::ADMIN );

  if( _type == TYPE_ERROR ) {
    error_e error = get_error();      // accessed by method to use asserts
  } else if( _type == TYPE_HANDSHAKE ) {
    header->set_type( Reveal::Messages::Analytics::Message::HANDSHAKE );
  } else if( _type == TYPE_REQUEST ) {
    header->set_type( Reveal::Messages::Analytics::Message::REQUEST );
    header->set_error( Reveal::Messages::Analytics::Message::ERROR_NONE );

    // write_request( msg );

  } else if( _type == TYPE_RESPONSE ) {
    header->set_type( Reveal::Messages::Analytics::Message::RESPONSE );
    header->set_error( Reveal::Messages::Analytics::Message::ERROR_NONE );

    // write_response( msg );

  } else if( _type == TYPE_COMMAND ) {
    header->set_type( Reveal::Messages::Analytics::Message::COMMAND );
    header->set_error( Reveal::Messages::Analytics::Message::ERROR_NONE );

    // write_command( msg );

  } else {
    return false;
  }

/*
  if( _type == TYPE_HANDSHAKE ) {
    header->set_type( Reveal::Messages::Analytics::Message::HANDSHAKE );
  } else if( _type == TYPE_ERROR ) {
    header->set_type( Reveal::Messages::Analytics::Message::ERROR );
    // cross reference the correct error
    error_e error = get_error();      // accessed by method to use asserts
    // TODO: map any newly defined errors
    if( error == ERROR_BAD_SCENARIO_REQUEST ) {
      header->set_error( Reveal::Messages::Analytics::Message::ERROR_BAD_SCENARIO );
    } else if( error == ERROR_BAD_TRIAL_REQUEST ) {
      header->set_error( Reveal::Messages::Analytics::Message::ERROR_BAD_TRIAL );
    } else {
      header->set_error( Reveal::Messages::Analytics::Message::ERROR_GENERAL );
    }

  } else if( _type == TYPE_DIGEST ) {
    // digest response
    header->set_type( Reveal::Messages::Analytics::Message::DIGEST );
    header->set_error( Reveal::Messages::Analytics::Message::ERROR_NONE );

    write_digest( msg );

  } else if( _type == TYPE_EXPERIMENT ) {
    // experiment response
    header->set_type( Reveal::Messages::Analytics::Message::EXPERIMENT );
    header->set_error( Reveal::Messages::Analytics::Message::ERROR_NONE );

    write_experiment( msg );
    write_scenario( msg );

  } else if( _type == TYPE_TRIAL ) {
    // trial response
    header->set_type( Reveal::Messages::Analytics::Message::TRIAL );
    header->set_error( Reveal::Messages::Analytics::Message::ERROR_NONE );

    write_experiment( msg );
    write_trial( msg );

  } else if( _type == TYPE_SOLUTION ) {
    // solution receipt
    header->set_type( Reveal::Messages::Analytics::Message::SOLUTION );

    // TODO : More comprehensive receipt including session, experiment id,  and               trial
  } else if( _type == TYPE_STEP ) {
    header->set_type( Reveal::Messages::Analytics::Message::STEP );
  } else if( _type == TYPE_EXIT ) {
    header->set_type( Reveal::Messages::Analytics::Message::EXIT );
  } else {
    _error = ERROR_BUILD;
    //return ERROR_BUILD;
    return false;
  }
*/
  //return ERROR_NONE;
  return true;
}



//----------------------------------------------------------------------------
} // namespace Analytics
//----------------------------------------------------------------------------
} // namespace Reveal
//----------------------------------------------------------------------------
