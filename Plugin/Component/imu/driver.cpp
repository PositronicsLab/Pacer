#include "driver.h"
#include <algorithm>
#include <iostream>
#include <boost/bind.hpp>

//using boost::asio;
//using boost::posix_time;
using namespace boost;
using namespace microstrain_3dm_gx3_35;

// setup static variables
unsigned char IMU::sync1 = 0x75;
unsigned char IMU::sync2 = 0x65;

/// Converts an unsigned value to two characters
static void to_two(unsigned value, unsigned char& c1, unsigned char& c2)
{
  c1 = value / 256;
  c2 = value %  256;
}

/// Gets the current time (as a floating-point number)
static unsigned long get_current_time()
{
  timeval t;
  gettimeofday(&t, NULL);
  return (unsigned long) t.tv_sec * 1000000 + (unsigned long) t.tv_usec;
}

IMU::IMU(int rate, models model) : 
  io(), serial(io), timer(io), timeout(posix_time::seconds(0)) ,
  _model(model)
{
  // check whether the value is reasonable
  _rate = rate; 
  assert(_rate > 0 && _rate <= 1000);

  // clear position and velocity
  _x.set_zero();
  _xd.set_zero();

  // initialize the mutex
  pthread_mutex_init(&_mutex, NULL);
}

IMU::~IMU() {
}

/// Opens the IMU as a serial port for asynchronous communication
bool IMU::openPort(const std::string& port, unsigned int baud_rate, boost::asio::serial_port_base::parity opt_parity,
    boost::asio::serial_port_base::character_size opt_csize,
    boost::asio::serial_port_base::flow_control opt_flow,
    boost::asio::serial_port_base::stop_bits opt_stop) {

  // if the port is already open, close it
  if(isOpen()) closePort();

  try {

    // open port and set desired parameters
    serial.open(port);
    serial.set_option(asio::serial_port_base::baud_rate(baud_rate));
    serial.set_option(opt_parity);
    serial.set_option(opt_csize);
    serial.set_option(opt_flow);
    serial.set_option(opt_stop);

  } catch(boost::system::system_error& e) {

      std::cerr<<"Error: " << e.what() <<std::endl;
      return false;
  }


  return true;

}

/// Opens the serial port
bool IMU::isOpen() const {

  return serial.is_open();

}

/// Closes the serial port
void IMU::closePort() {

  if(isOpen()==false) return;
  serial.close();

}

/// Sets the serial port timeout
void IMU::setTimeout(const posix_time::time_duration& t)
{
    timeout=t;
}

/// Writes data to the serial port
void IMU::write(const std::vector<unsigned char>& data)
{
    asio::write(serial,asio::buffer(&data[0],data.size()));
}

/// Callback function for I/O communication
void IMU::timeoutExpired(const boost::system::error_code& error)
{
     if(!error && result==resultInProgress) result=resultTimeoutExpired;
}

/// Callback function for I/O communication
void IMU::readCompleted(const boost::system::error_code& error,
        const size_t bytesTransferred)
{
    if(!error)
    {
        result=resultSuccess;
        this->bytesTransferred=bytesTransferred;
        return;
    }

    if(error.value()==125) return; //Linux outputs error 125

    result=resultError;
}

/// Reads the specified number of bytes from the serial device
void IMU::read(size_t size, std::vector<unsigned char>& output)
{
  output.resize(0);
  output.resize(size,'\0');//Allocate a vector with the desired size
  read((char*) &output[0],size);//Fill it with values
}

/// Keeps polling the IMU until a specified message is read 
void IMU::waitForMsg() {

  static std::vector<unsigned char> data, recv;

  // clear recv
  recv.clear();

  unsigned char prev = ' ';

  do {

    if (recv.size() > 0) 
      prev = recv[0];
    else 
      prev = ' ';

    recv.clear();
    read(1, recv);

  } while (!(prev==sync1 && recv[0]==sync2));
}

/// Reads the specified amount of data
void IMU::read(char *data, size_t size)
{
    // if there is some data in the buffer from a previous read, read it
    if (readData.size() > 0)
    {
      std::basic_istream<char> is(&readData);
      size_t toRead=std::min(readData.size(),size);//How many bytes to read?
      is.read(data,toRead);
      data+=toRead;
      size-=toRead;
      if(size==0) return;//If read data was enough, just return
    }

    setupParameters=ParameterReader((unsigned char*) data,size);
    performReadSetup(setupParameters);

    //For this code to work, there should always be a timeout, so the
    //request for no timeout is translated into a very long timeout
    if(timeout!=posix_time::seconds(0)) timer.expires_from_now(timeout);
    else timer.expires_from_now(posix_time::hours(100000));

    // setup the callback function for timer expiration
    timer.async_wait(boost::bind(&IMU::timeoutExpired,this,
                asio::placeholders::error));

    // read until all bytes are transferred
    result=resultInProgress;
    bytesTransferred=0;
    for(;;)
    {
        io.run_one();
        switch(result)
        {
          case resultInProgress:

            continue;

            case resultSuccess:
                timer.cancel();
                return;
            case resultTimeoutExpired:
                serial.cancel();
                throw(timeout_exception("Timeout expired"));
            case resultError:
                timer.cancel();
                serial.cancel();
                throw(boost::system::system_error(boost::system::error_code(),
                        "Error while reading"));
            //if resultInProgress remain in the loop
        }
    }
}

/// Sets up the parameters for reading from the serial port
void IMU::performReadSetup(const ParameterReader& param)
{
    if(param.fixedSize)
    {
        asio::async_read(serial,asio::buffer(param.data,param.size),boost::bind(
                &IMU::readCompleted,this,asio::placeholders::error,
                asio::placeholders::bytes_transferred));
    } else {
        asio::async_read_until(serial,readData,param.delim,boost::bind(
                &IMU::readCompleted,this,asio::placeholders::error,
                asio::placeholders::bytes_transferred));
    }
}

/// Verifies the IMU's existence
bool IMU::ping() {

  return sendNoDataCmd(CMD_SET_BASIC, CMD_BASIC_PING);

}

/// Polls the GPS
bool IMU::pollGPS() {

  static std::vector<unsigned char> data, recv;

  // clear vectors
  data.clear();
  recv.clear();

  // get ready to write the data
  data.push_back(sync1);
  data.push_back(sync2);
  data.push_back(CMD_SET_3DM); // desc set
  data.push_back(0x04); // length
  data.push_back(0x04);
  data.push_back(CMD_3DM_POLL_GPS);
  data.push_back(0x1); // suppress ACK
  data.push_back(0x0);

  calcFletcher(data);
  write(data);
  waitForMsg();

  size_t PACKET_SZ = 48; // 44+6, TODO this is really stupid... there must be some parsing etc....

  unsigned long current_time = get_current_time();
  read(PACKET_SZ, recv);

  if (!checkFletcher(recv)) return false;

  if (recv[2] != 0x2C || recv[3] != 0x03) {
    throw std::runtime_error("GPS: Wrong msg format");
  }

//  _gps_data.time =  (uint64_t)(curtime.tv_sec) * 1000000000 + (uint64_t)(curtime.tv_nsec);

  _gps_data.time = current_time;
  _gps_data.latitude = extractDouble(&recv[4]);
  _gps_data.longtitude = extractDouble(&recv[4+8]);
  _gps_data.horizontal_accuracy = extractFloat(&recv[4+32]);

  uint16_t flags = /*((uint16_t)recv[6+40])<<2 | */(uint16_t)recv[4+41];

  _gps_data.lat_lon_valid = (flags & 0x1);
  _gps_data.hor_acc_valid = (flags & (0x1<<5));

  return true;
}

/// Get acceleration data, Euler angles, etc.
bool IMU::pollAHRS() {

  const float G = 9.80655;
  static std::vector<unsigned char> data, recv;

  // clear vectors
  data.clear();
  recv.clear();

  data.push_back(sync1);
  data.push_back(sync2);
  data.push_back(CMD_SET_3DM); // desc set
  data.push_back(0x04); // length
  data.push_back(0x04);
  data.push_back(CMD_3DM_POLL_AHRS);
  data.push_back(0x1); // suppress ACK
  data.push_back(0x0);

  // calculate Fletcher values
  calcFletcher(data);

  // write the data
  write(data);

  // wait for ACK
  waitForMsg();

  // (last packet data + 2 byte checksum) 
  size_t PACKET_SZ = 78; 

  unsigned long current_time = get_current_time();

  // read the data
  read(PACKET_SZ, recv);

  // check Fletcher
  if (!checkFletcher(recv)) return false;
//  if (!checkACK(recv,CMD_SET_3DM,CMD_3DM_POLL_AHRS)) return false;

  // quaternion 0x0A, field length 18, MSB first

  //quat.time = posix_time::microsec_clock::local_time();

  if (recv[2] != 0x0E || recv[3] != 0x04) {

    errMsg("AHRS: Wrong msg format (0x04).");
    return false;

  }

  // get acceleration vector
  unsigned ST = 4;
  _ahrs_data.ax = extractFloat(&recv[ST])*G;  ST+= 4; // 0x04
  _ahrs_data.ay = extractFloat(&recv[ST])*G;  ST+= 4;
  _ahrs_data.az = extractFloat(&recv[ST])*G;  ST+= 4;

  if (recv[ST] != 0x0E || recv[ST+1] != 0x05) {

    errMsg("AHRS: Wrong msg format (0x05).");
    return false;

  }

  // get gravity vector
  ST += 2;
  _ahrs_data.gx = extractFloat(&recv[ST]);   ST+= 4; // 0x05
  _ahrs_data.gy = extractFloat(&recv[ST]);   ST+= 4;
  _ahrs_data.gz = extractFloat(&recv[ST]);   ST+= 4;

  // get delta velocity vector
  if (recv[ST] != 0x0E || recv[ST+1] != 0x08) {

    errMsg("AHRS: Wrong msg format (0x08).");
    return false;
  }
  ST += 2;
  _ahrs_data.vx = extractFloat(&recv[ST])*G;   ST+= 4; // 0x08
  _ahrs_data.vy = extractFloat(&recv[ST])*G;   ST+= 4;
  _ahrs_data.vz = extractFloat(&recv[ST])*G;   ST+= 4;

  // get quaternion orientation
  if (recv[ST] != 0x12 || recv[ST+1] != 0x0A) {

    errMsg("AHRS: Wrong msg format (0x0A).");
    return false;

  }

  ST += 2;
  _ahrs_data.q0 = extractFloat(&recv[ST]);  ST += 4;
  _ahrs_data.q1 = extractFloat(&recv[ST]);  ST += 4;
  _ahrs_data.q2 = extractFloat(&recv[ST]);  ST += 4;
  _ahrs_data.q3 = extractFloat(&recv[ST]);  ST += 4;

  // get roll-pitch-yaw
  if (recv[ST] != 0x0E || recv[ST+1] != 0x0C) {
    errMsg("AHRS: Wrong msg format (0x0C).");
    return false;
  }

  ST += 2;
  _ahrs_data.r = extractFloat(&recv[ST]); ST += 4; // 0x0C
  _ahrs_data.p = extractFloat(&recv[ST]); ST += 4;
  _ahrs_data.y = extractFloat(&recv[ST]); ST += 4;

  // get time 
  if (recv[ST] != 0x06 || recv[ST+1] != 0x0E) {
    errMsg("AHRS: Wrong msg format (0x0E).");
    return false;
  }

  ST += 2;
  _ahrs_data.time = (uint64_t) (extractUInt(&recv[ST]) / 62.5);

  return true;
}

/// Gets AHRS data
tahrs& IMU::getAHRS() {
  return _ahrs_data;
}

/// Gets GPS data
tgps& IMU::getGPS() {
  return _gps_data;
}

/// Extracts an unsigned int from four bytes
unsigned IMU::extractUInt(unsigned char* addr) {

  assert(sizeof(unsigned) == 4);
  unsigned tmp;

  *((unsigned char*)(&tmp) + 3) = *(addr);
  *((unsigned char*)(&tmp) + 2) = *(addr+1);
  *((unsigned char*)(&tmp) + 1) = *(addr+2);
  *((unsigned char*)(&tmp)) = *(addr+3);

  return tmp;
}

/// Extracts a single precision floating point value from four bytes
float IMU::extractFloat(unsigned char* addr) {

  float tmp;

  *((unsigned char*)(&tmp) + 3) = *(addr);
  *((unsigned char*)(&tmp) + 2) = *(addr+1);
  *((unsigned char*)(&tmp) + 1) = *(addr+2);
  *((unsigned char*)(&tmp)) = *(addr+3);

  return tmp;
}

/// Encodes a single precision floating point value as four bytes
void IMU::encodeFloat(float in, std::vector<unsigned char>& arr) {

  arr.push_back(*((unsigned char*)(&in) + 3));
  arr.push_back(*((unsigned char*)(&in) + 2));
  arr.push_back(*((unsigned char*)(&in) + 1));
  arr.push_back(*((unsigned char*)(&in)));

}

double IMU::extractDouble(unsigned char* addr) {

  double tmp;

  *((unsigned char*)(&tmp) + 7) = *(addr);
  *((unsigned char*)(&tmp) + 6) = *(addr+1);
  *((unsigned char*)(&tmp) + 5) = *(addr+2);
  *((unsigned char*)(&tmp) + 4) = *(addr+3);

  *((unsigned char*)(&tmp) + 3) = *(addr+4);
  *((unsigned char*)(&tmp) + 2) = *(addr+5);
  *((unsigned char*)(&tmp) + 1) = *(addr+6);
  *((unsigned char*)(&tmp)) = *(addr+7);

  return tmp;
}


bool IMU::setGPSMsgFormat() {

  static std::vector<unsigned char> data, recv;

  // clear data and recv vector 
  data.clear();
  recv.clear();

  // prepare message
  data.push_back(sync1);
  data.push_back(sync2);
  data.push_back(CMD_SET_3DM); // desc set
  data.push_back(0x07); // length
  data.push_back(0x07);
  data.push_back(CMD_3DM_GPS_MSG_FORMAT);

  data.push_back(FUN_USE_NEW);
  data.push_back(0x01); // desc count

  data.push_back(0x03); // LLH
  data.push_back(0x0);
  data.push_back(0x1); // 4 Hz / TODO check this!!!

  // compute the Fletcher value and write the packet 
  calcFletcher(data);
  write(data);
  waitForMsg();

  // read packet
  size_t PACKET_SZ = 8;
  read(PACKET_SZ, recv);

  // verify Fletcher and acknowledgement
  if (!checkFletcher(recv)) return false;
  if (!checkACK(recv,CMD_SET_3DM,CMD_3DM_GPS_MSG_FORMAT)) return false;

  return true;
}

/// Sets the dynamics mode for the IMU
bool IMU::setDynamicsMode()
{
  static std::vector<unsigned char> data, recv;

  // clear data and recv vectors
  data.clear();
  recv.clear();

  // setup packet
  data.push_back(sync1);
  data.push_back(sync2);
  data.push_back(CMD_SET_3DM); // desc set (0x0C)
  data.push_back(0x04);        // payload length is fixed 
  data.push_back(0x04);        // payload length is fixed 
  data.push_back(0x34);        // set dynamics 

  // data packet begins here
  data.push_back(0x01);             // apply new settings 

  // dynamics mode- airborne 
  data.push_back(0x07);  

  // compute Fletcher and write the packet
  calcFletcher(data);
  write(data);
  waitForMsg();

  // read the packet
  size_t PACKET_SZ = 8;
  read(PACKET_SZ, recv);

  // verify Fletcher and acknowledgement
  if (!checkFletcher(recv)) return false;
  if (!checkACK(recv,CMD_SET_3DM,0x34)) return false;

  return true;
}

/// Sets signal conditioning for the IMU
bool IMU::setAHRSSignalCond() {

  static std::vector<unsigned char> data, recv;

  // clear data and recv vectors
  data.clear();
  recv.clear();

  // setup packet
  data.push_back(sync1);
  data.push_back(sync2);
  data.push_back(CMD_SET_3DM); // desc set (0x0C)
  data.push_back(0x10);        // payload length is fixed 
  data.push_back(0x10);        // payload length is fixed 
  data.push_back(0x35); // signal conditioning 

  // data packet begins here
  data.push_back(0x01);             // apply new settings 

  // decimation for orientation
  data.push_back(0x00);  // first, set placeholders           
  data.push_back(0x00);             
  to_two(0x01, data[data.size()-2], data[data.size()-1]);

  // data conditioning flags
  data.push_back(0x0);  // first, set placeholders
  data.push_back(0x0);
  to_two(0x1000 + 0x0001 + 0x0002, data[data.size()-2], data[data.size()-1]);

  // accel/gyro adjustible filter bandwidth (1000 Hz)
  data.push_back(0x01);

  // mag adjustible filter bandwidth (1000 Hz)
  data.push_back(0x01);

  // up compensation values 
  data.push_back(0x0);  // first, set placeholders
  data.push_back(0x0);
  to_two(0x01, data[data.size()-2], data[data.size()-1]);

  // North compensation values 
  data.push_back(0x0);  // first, set placeholders
  data.push_back(0x0);
  to_two(0x01, data[data.size()-2], data[data.size()-1]);

  // mag power/bandwidth
  data.push_back(0x00);

  // setup reserved data
  data.push_back(0x00);
  data.push_back(0x00);

  // compute Fletcher and write the packet
  calcFletcher(data);
  write(data);
  waitForMsg();

  // read the packet
  size_t PACKET_SZ = 8;
  read(PACKET_SZ, recv);

  // verify Fletcher and acknowledgement
  if (!checkFletcher(recv)) return false;
  if (!checkACK(recv,CMD_SET_3DM,0x35)) return false;

  return true;
}

bool IMU::setAHRSMsgFormat() {

  static std::vector<unsigned char> data, recv;

  // clear data and recv vectors
  data.clear();
  recv.clear();

  // setup packet
  data.push_back(sync1);
  data.push_back(sync2);
  data.push_back(CMD_SET_3DM); // desc set (0x0C)
  data.push_back(22);  // total command length = 6*3 + 4 
  data.push_back(22); 
  data.push_back(CMD_3DM_AHRS_MSG_FORMAT); // (0x08)

  // data packet begins here
  data.push_back(FUN_USE_NEW);             // (0x01)
  data.push_back(0x06); // desc count

  data.push_back(0x04); // accelerometer vector
  data.push_back(0x0);
  data.push_back(1000/_rate); 

  data.push_back(0x05); // gyro vector
  data.push_back(0x0);
  data.push_back(1000/_rate); 

  data.push_back(0x08); // velocity vector 
  data.push_back(0x0);
  data.push_back(1000/_rate); 

  data.push_back(0x0A); // quaternion vector 
  data.push_back(0x0);
  data.push_back(1000/_rate); 

  data.push_back(0x0C); // euler angles
  data.push_back(0x0);
  data.push_back(1000/_rate); 

  data.push_back(0x0E); // timestamp
  data.push_back(0x0);
  data.push_back(1000/_rate);

  // compute Fletcher and write the packet
  calcFletcher(data);
  write(data);
  waitForMsg();

  // read the packet
  size_t PACKET_SZ = 8;
  read(PACKET_SZ, recv);

  // verify Fletcher and acknowledgement
  if (!checkFletcher(recv)) return false;
  if (!checkACK(recv,CMD_SET_3DM,CMD_3DM_AHRS_MSG_FORMAT)) return false;

  return true;
}

/// EMD: unsure what this does
bool IMU::resume() {
  return sendNoDataCmd(CMD_SET_BASIC, CMD_BASIC_RESUME);
}


/// EMD: unsure what this does
bool IMU::setToIdle() {
  return sendNoDataCmd(CMD_SET_BASIC, CMD_BASIC_SET_TO_IDLE);

}

bool IMU::sendNoDataCmd(uint8_t cmd_set, uint8_t cmd) {

  static std::vector<unsigned char> data, recv;

  // clear the vectors
  data.clear();
  recv.clear();

  // set the data
  data.push_back(sync1);
  data.push_back(sync2);
  data.push_back(cmd_set); // desc set
  data.push_back(0x02); // length
  data.push_back(0x02);
  data.push_back(cmd);

  // compute the Fletcher value
  calcFletcher(data);
  write(data);

  // read the packet
  size_t PACKET_SZ = 8;
  waitForMsg();
  read(PACKET_SZ, recv);

  // check the packet
  if (!checkFletcher(recv)) return false;

  // verify acknowledgement
  if (!checkACK(recv,cmd_set,cmd)) return false;

  return true;
}

bool IMU::selfTest() {

  posix_time::time_duration timeout_orig = timeout;

  setTimeout(posix_time::seconds(6));

  static std::vector<unsigned char> data, recv;

  // clear data and recv vectors
  data.clear();
  recv.clear();

  // prepare the packet
  data.push_back(sync1);
  data.push_back(sync2);
  data.push_back(CMD_SET_BASIC);
  data.push_back(0x02);
  data.push_back(0x02);
  data.push_back(CMD_BASIC_DEV_BUILTIN_TEST);

  // compute the Fletcher and write the packet
  calcFletcher(data);
  write(data);
  waitForMsg();

  // read the packet
  size_t PACKET_SZ = 14;
  read(PACKET_SZ, recv);

  // compute a Fletcher check
  if (!checkFletcher(recv)) {
    setTimeout(timeout_orig);
    return false;
  }

  // set the timeout value
  setTimeout(timeout_orig);

  // verify acknowledgement
  if (!checkACK(recv,CMD_SET_BASIC, CMD_BASIC_DEV_BUILTIN_TEST)) return false;

  if (recv[8]==0 && recv[9]==0 && recv[10]==0 && recv[11]==0) return true;
  else {

    if (recv[8] & 0x1) errMsg("AP-1: I2C Hardware Error.");
    if (recv[8] & 0x2) errMsg("AP-1: I2C EEPROM Error.");

    if (recv[9] & 0x1) errMsg("AHRS: Communication Error.");

    if (recv[10] & 0x1) errMsg("GPS: Communication Error.");
    if (recv[10] & 0x2) errMsg("GPS: 1PPS Signal Error.");
    if (recv[10] & 0x4) errMsg("GPS: 1PPS Inhibit Error.");
    if (recv[10] & 0x8) errMsg("GPS: Power Control Error.");

    return false;
  }
}


bool IMU::devStatus() {

  static std::vector<unsigned char> data, recv;

  // clear the data and recv vectors
  data.clear();
  recv.clear();

  // setup the message
  data.push_back(sync1);
  data.push_back(sync2);
  data.push_back(CMD_SET_3DM);
  data.push_back(0x05);
  data.push_back(0x05);
  data.push_back(CMD_3DM_DEV_STATUS);
  //data.push_back(((uint16_t)MODEL_ID>>2) & 0xff);
  //data.push_back((uint16_t)MODEL_ID & 0xff);
  if (_model == GX3_45){
    data.push_back(0x18);
    data.push_back(0x54);
  }else if( _model == GX3_35){
    data.push_back(0x18);
    data.push_back(0x51);
  }
  data.push_back(0x01); // basic status

  // calculate Fletcher value and write the packet
  calcFletcher(data);
  write(data);
  waitForMsg();

  // read the packet
  size_t PACKET_SZ = 25;
  read(PACKET_SZ, recv);

  // verify Fletcher value
  if (!checkFletcher(recv)) {
    return false;
  }

  // verify acknowledgement
  if (!checkACK(recv,CMD_SET_3DM, CMD_3DM_DEV_STATUS)) return false;

  //if (((uint8_t)recv[10] != ((MODEL_ID>>2)&0xff)) || ((uint8_t)recv[11] != (MODEL_ID & 0xff))) {
  if ((_model == GX3_45 && (((uint8_t)recv[8] != 0x18) || ((uint8_t)recv[9] != 0x54))) 
      ||(_model == GX3_35 && (((uint8_t)recv[8] != 0x18) || ((uint8_t)recv[9] != 0x51))) ){

    errMsg("Wrong model number.");
    return false;
  }

  if (recv[11] != COMM_MODE_MIP) {
    errMsg("Not in MIP mode.");
    return false;
  }

  return true;
}

bool IMU::setStream(uint8_t stream, bool state) {

  static std::vector<unsigned char> data, recv;

  // clear data and recv vectors
  data.clear();
  recv.clear();

  // prepare message
  data.push_back(sync1);
  data.push_back(sync2);
  data.push_back(CMD_SET_3DM);
  data.push_back(0x05);
  data.push_back(0x05);
  data.push_back(CMD_3DM_STREAM_STATE);
  data.push_back(0x1);
  data.push_back(stream);
  if (state) data.push_back(0x01);
  else data.push_back(0x0);

  // prepare message
  calcFletcher(data);
  write(data);
  waitForMsg();

  // read the packet
  size_t PACKET_SZ = 8;
  read(PACKET_SZ, recv);

  // verify Fletcher value
  if (!checkFletcher(recv)) 
    return false;

  // verify acknowledgement
  if (!checkACK(recv,CMD_SET_3DM, CMD_3DM_STREAM_STATE)) return false;

  return true;
}

/// Disables all streams for the IMU
bool IMU::disAllStreams() {

  bool ret = true;

  if (!setStream(0x01,false)) ret = false; // AHRS
  if (!setStream(0x02,false)) ret = false; // GPS

  // destroy running thread, if necessary
  if (_thread_running)
    _thread_running = false; 

  return ret;
}

/// Reads from the AHRS stream
void IMU::readFromAHRSStream()
{
  const float G = 9.80655;
  static uint64_t last_time = std::numeric_limits<uint64_t>::max();
  static std::vector<unsigned char> recv;
  unsigned MAX_BUF_SIZE = 1000;

  // read two bytes (header)
  recv.clear();
  read(2, recv);

  if (recv[0] != 'u' || recv[1] != 'e')
    return;

  // (last packet data [76] + 2 byte checksum) 
  size_t PACKET_SZ = 84; 

  unsigned long current_time = get_current_time();

  // read the data
  recv.clear();
  read(PACKET_SZ, recv);

  // check Fletcher
  if (!checkFletcher(recv)) return;
//  if (!checkACK(recv,CMD_SET_3DM,CMD_3DM_POLL_AHRS)) return false;

  if (recv[2] != 0x0E || recv[3] != 0x04) {

    errMsg("AHRS: Wrong msg format (0x04).");
    return;

  }

  // create some ahrs data
  tahrs data;
  data.time = current_time;

  // get acceleration vector
  unsigned ST = 4;
  data.ax = extractFloat(&recv[ST])*G;  ST+= 4; // 0x04
  data.ay = extractFloat(&recv[ST])*G;  ST+= 4;
  data.az = extractFloat(&recv[ST])*G;  ST+= 4;

  if (recv[ST] != 0x0E || recv[ST+1] != 0x05) {

    errMsg("AHRS: Wrong msg format (0x05).");
    return;

  }

  // get gravity vector
  ST += 2;
  data.gx = extractFloat(&recv[ST]);   ST+= 4; // 0x05
  data.gy = extractFloat(&recv[ST]);   ST+= 4;
  data.gz = extractFloat(&recv[ST]);   ST+= 4;

  // get delta velocity vector
  if (recv[ST] != 0x0E || recv[ST+1] != 0x08) {

    errMsg("AHRS: Wrong msg format (0x08).");
    return;
  }
  ST += 2;
  data.vx = extractFloat(&recv[ST])*G;   ST+= 4; // 0x08
  data.vy = extractFloat(&recv[ST])*G;   ST+= 4;
  data.vz = extractFloat(&recv[ST])*G;   ST+= 4;

  // get quaternion orientation
  if (recv[ST] != 0x12 || recv[ST+1] != 0x0A) {

    errMsg("AHRS: Wrong msg format (0x0A).");
    return;

  }

  ST += 2;
  data.q0 = extractFloat(&recv[ST]);  ST += 4;
  data.q1 = extractFloat(&recv[ST]);  ST += 4;
  data.q2 = extractFloat(&recv[ST]);  ST += 4;
  data.q3 = extractFloat(&recv[ST]);  ST += 4;

  // get roll-pitch-yaw
  if (recv[ST] != 0x0E || recv[ST+1] != 0x0C) {

    errMsg("AHRS: Wrong msg format (0x0C).");
    return;

  }

  ST += 2;
  data.r = extractFloat(&recv[ST]); ST += 4; // 0x0C
  data.p = extractFloat(&recv[ST]); ST += 4;
  data.y = extractFloat(&recv[ST]); ST += 4;

  // get time 
  if (recv[ST] != 0x06 || recv[ST+1] != 0x0E) {
    errMsg("AHRS: Wrong msg format (0x0E).");
    return;
  }

  ST += 2;
  data.time = (uint64_t) (extractUInt(&recv[ST]) / 62.5);

  // add the data to the buffer
  pthread_mutex_lock(&_mutex);
  if (_ahrs_buffer.size() > MAX_BUF_SIZE)
    _ahrs_buffer.pop();
  _ahrs_buffer.push(data);
  pthread_mutex_unlock(&_mutex);

  // update IMU state data
  uint64_t dtint = (last_time == std::numeric_limits<uint64_t>::max()) ? 0 : data.time - last_time;
  float dt = dtint/1000.0;
  last_time = data.time;

  // update linear acceleration and angular velocity
  _xdd[0] = data.ax;
  _xdd[1] = data.ay;
  _xdd[2] = data.az;

  // update angular velocity
  _omega[0] = -data.gx;
  _omega[1] = data.gy;
  _omega[2] = -data.gz;

  // update the orientation
  _quat = Ravelin::Quatd::rpy(-data.r, data.p, -data.y);

  // update the acceleration to the global frame
  _xdd = _quat * _xdd;

  // unbias linear acceleration
  _xdd[2] += G;

  // determine velocity
  _xd += _xdd * dt;

  // determine position
  _x += _xd * dt;
}

/// Gets a AHRS data item off of the buffer
tahrs IMU::getAHRSItem()
{
  pthread_mutex_lock(&_mutex);
  tahrs data = _ahrs_buffer.front();
  _ahrs_buffer.pop();
  pthread_mutex_unlock(&_mutex);
  return data;
}

/// Enables AHRS streaming 
bool IMU::enableAHRSStream() {

  bool value = setStream(0x01,true); // AHRS

  // create a new thread to continually read from the stream
  if (value)
  {
    // indicate that the thread is running
    _thread_running = true;

    // start the thread
    pthread_t thread;
    pthread_create(&thread, NULL, &ahrs_thread, this);
  }

  return value;
}

/// Creates the thread
void* IMU::ahrs_thread(void* data)
{
  const unsigned SLEEP_US = 100;

  // get the IMU pointer
  IMU* imu = (IMU*) data;

  // continually read from the IMU
  while (true)
  {
    // see whether the thread is still running
    if (!imu->_thread_running)
      return NULL;

    // read from the AHRS stream
    imu->readFromAHRSStream();

    // sleep a little
    usleep(SLEEP_US);
  }

  return NULL;
}

/// Adds an error message to the vector of error messages
void IMU::errMsg(const std::string& msg) {
  error_desc.push_back(msg);
}

/// Checks acknowledgement
bool IMU::checkACK(const std::vector<unsigned char>& arr, uint8_t cmd_set, uint8_t cmd) {

  if (arr.size() < 6) {
    errMsg("IMU::checkACK() - too short of a reply.");
    return false;
  }

  /*if (arr[0] != sync1 || arr[1] != sync2) {

    errMsg("Strange synchronization bytes.");
    return false;

  }*/

  if (arr[0] != cmd_set) {
    errMsg("IMU::checkACK() - wrong description set in reply");
    return false;
  }

  if (arr[4] != cmd) {
    errMsg("IMU::checkACK() - wrong command echo");
    return false;
  }

  if (arr[5] != 0x0) {
    errMsg("IMU::checkACK() - NACK.");
    return false;
  }

  return true;
}

/// Gets the last error message - error messages are progressively added to a vector
std::string IMU::getLastError() {

  if (error_desc.size() > 0) {
    std::string tmp = error_desc.back();
    error_desc.pop_back();
    return tmp;
  } else return "";
}

/// Performs a Fletcher check for a string of unsigned characters
bool IMU::checkFletcher(const std::vector<unsigned char>& arr) {

  unsigned char b1=0;
  unsigned char b2=0;

  if ( ((uint8_t)arr[1]+4) != (uint8_t)arr.size() ) {
    std::cerr << "Sizes mismatch." << std::endl;
    return false;
  }

  uint8_t end;

  if ( ((uint8_t)arr[1]+2) <= (uint8_t)arr.size()) end = (uint8_t)arr[1]+2;
  else end = (uint8_t)arr.size();

  b1 += sync1;
  b2 += b1;
  b1 += sync2;
  b2 += b1;

  for(unsigned int i=0; i<end; i++)
  {
   b1 += arr[i];
   b2 += b1;
  }

  if (b1==(unsigned char)arr[arr.size()-2] && b2==(unsigned char) arr.back()) 
    return true;
  else {
    std::cerr << "Bad Fletcher value detected";
    return false;
  }
}

/// Computes a Fletcher value for a string of unsigned characters
void IMU::calcFletcher(std::vector<unsigned char>& arr) {

  unsigned char b1=0;
  unsigned char b2=0;

  for(unsigned int i=0; i<arr.size(); i++)
  {
   b1 += arr[i];
   b2 += b1;
  }

  arr.push_back(b1);
  arr.push_back(b2);
}

