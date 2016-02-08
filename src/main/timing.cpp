#ifdef USE_REALTIME
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>

//-----------------------------------------------------------------------------
// The signal identifier for the real-time timer
#define RTTIMER_SIGNAL       SIGRTMIN + 4

// Conversion factor for nanoseconds to seconds
#define NSEC_PER_SEC         1E9

//-----------------------------------------------------------------------------
/// Signal handler typedef for ease of use
typedef void ( *timer_sighandler_fn )( int signum, siginfo_t *si, void *data );

//-----------------------------------------------------------------------------
class timer_c {
private:
  sigset_t           _rttimer_mask;
  struct sigaction   _rttimer_sigaction;
  timer_t            _rttimer_id;

public:
  timer_c( void ) { }

  virtual ~timer_c( void ) { }

  /// Error codes for timer control
  enum timer_err_e {
    TIMER_ERROR_NONE = 0,
    TIMER_ERROR_SIGACTION,
    TIMER_ERROR_SIGPROCMASK,
    TIMER_ERROR_CREATE,
    TIMER_ERROR_SETTIME,
    TIMER_ERROR_GETCLOCKID
  };

  /// Creates a high-resolution real-time timer
  timer_err_e create( timer_sighandler_fn sighandler, int signum ) {
    struct sigevent sevt;

    // Establish handler for timer signal
    _rttimer_sigaction.sa_flags = SA_SIGINFO;
    _rttimer_sigaction.sa_sigaction = sighandler;
    sigemptyset( &_rttimer_sigaction.sa_mask );
    if( sigaction( signum, &_rttimer_sigaction, NULL ) == -1) {
        return TIMER_ERROR_SIGACTION;
    }

    // intialize the signal mask
    sigemptyset( &_rttimer_mask );
    sigaddset( &_rttimer_mask, signum );

    sevt.sigev_notify = SIGEV_SIGNAL;
    sevt.sigev_signo = signum;
    sevt.sigev_value.sival_ptr = &_rttimer_id;
    if( timer_create( CLOCK_MONOTONIC, &sevt, &_rttimer_id ) == -1 ) {
       return TIMER_ERROR_CREATE;
    }

    return TIMER_ERROR_NONE;
  }

  /// Arms the timer for a period of nanoseconds less than 1e9 ns (1 s)
  timer_err_e arm( const unsigned long long& period_nsec ) {
    struct itimerspec its;

    unsigned long long period = period_nsec;

    its.it_interval.tv_sec = 0;
    its.it_interval.tv_nsec = 0;
    its.it_value.tv_sec = 0;

    its.it_value.tv_nsec = (unsigned long long)(period);

    if( timer_settime( _rttimer_id, 0, &its, NULL ) == -1 ) {
        return TIMER_ERROR_SETTIME;
    }

    return TIMER_ERROR_NONE;
  }
};

//-----------------------------------------------------------------------------
/// Utility function to compute the difference in time between two timespecs
// Is not robust for large time intervals, but works for anything under 1 sec.
unsigned long long diff(struct timespec ts0, struct timespec ts1) {
  unsigned long long result = 0;

  struct timespec delta;

  delta.tv_sec = ts1.tv_sec - ts0.tv_sec;
  delta.tv_nsec = ts1.tv_nsec - ts0.tv_nsec;

  result = delta.tv_sec * NSEC_PER_SEC + delta.tv_nsec;

  return result;
}

//-----------------------------------------------------------------------------
void timer_sighandler( int signum, siginfo_t *si, void *data ) {
  //printf( "timer event\n" );

  // If more accurate timing is necessary, add IPC here.
  // Warning: Do not do more than trigger IPC in a signal handler.  Logic needs
  // to occur in main thread
}

//-----------------------------------------------------------------------------
int main( void ) {
  timer_c timer;

  if( timer.create( timer_sighandler, RTTIMER_SIGNAL ) != timer_c::TIMER_ERROR_NONE ) {
    printf( "Failed to create timer\nExiting\n" );
    exit( 1 );
  }

  for( unsigned i = 0; i < 10; i++ ) {
    //  - validation part 1 -
    struct timespec ts0, ts1;
    clock_gettime( CLOCK_MONOTONIC, &ts0 );

    // - the guts -
    // set period to however long the timer should wait to fire
    unsigned long long period = 2E7;

    timer.arm( period );

    // sleep only blocks this process until the timer fires.  The timer will not
    // fire until AT LEAST period has passed so adjust period accordingly to get
    // a more accurate timing event.
    // Note: this approach offers no real guarantees in terms of priority and 
    // block-waiting time.  It is a very simple way to try to ensure time 
    // slicing.
    sleep( 1 );
    // - end guts -

    // - validation part 2 -
    clock_gettime( CLOCK_MONOTONIC, &ts1 );
    unsigned long long delta = diff( ts0, ts1 );
    printf( "delta: %llu\n", delta );
  }

  return 0;
}
#endif //USE_REALTIME
