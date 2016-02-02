#ifndef MS_3DMGX3_35_HH
#define MS_3DMGX3_35_HH

#include <queue>
#include <iostream>
#include <boost/asio/serial_port.hpp> 
#include <boost/asio.hpp>
#include <boost/utility.hpp>
#include <boost/lexical_cast.hpp>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <Ravelin/Origin3d.h>
#include <Ravelin/Quatd.h>

/*
 *
 * Serial communication based on following tutorial: http://www.webalice.it/fede.tft/serial_port/serial_port.html
 *
 * TODOs
 *  fix timeouts
 *	antenna offset (0x0D, 0x13)
 *
 *
 */

namespace microstrain_3dm_gx3_35 {

	struct tahrs {

                // linear acceleration vector
		float ax;
		float ay;
		float az;

                // gravity vector
		float gx;
		float gy;
		float gz;

                // delta velocity vector
                float vx, vy, vz;

                // roll, pitch, and yaw
		float r;
		float p;
		float y;

                // four quaternion values
                float q0, q1, q2, q3;

                // time at which the sample was taken
		uint64_t time;

	};

	struct tgps {

		double latitude;
		double longtitude;

		//double height;

		float horizontal_accuracy;
		//float vertical_accuracy;

		bool lat_lon_valid;
		bool hor_acc_valid;
		//bool height_valid;

		uint64_t time;

	};

	class timeout_exception: public std::runtime_error
	{
	public:
		timeout_exception(const std::string& arg): runtime_error(arg) {}
	};


  class IMU : private boost::noncopyable
    {
    
    public:

	  enum cmd_set {

		  CMD_SET_BASIC = 0x01,
		  CMD_SET_3DM = 0x0C,
		  CMD_SET_SYSTEM = 0x7F
	  };

	  enum cmd_set_basic {

		  CMD_BASIC_PING = 0x01,
		  CMD_BASIC_SET_TO_IDLE = 0x02,
		  CMD_BASIC_GET_DEV_INFO = 0x03,
		  CMD_BASIC_GET_DEV_DESC_SETS = 0x04,
		  CMD_BASIC_DEV_BUILTIN_TEST = 0x05,
		  CMD_BASIC_RESUME = 0x06,
		  CMD_BASIC_RESET = 0x7E
	  };

	  enum cmd_set_3dm {

		  CMD_3DM_POLL_AHRS = 0x01,
		  CMD_3DM_POLL_GPS = 0x02,
		  CMD_3DM_POLL_NAV = 0x03,
		  CMD_3DM_DEV_STATUS = 0x64,
		  CMD_3DM_STREAM_STATE = 0x11,
		  CMD_3DM_AHRS_MSG_FORMAT = 0x08,
		  CMD_3DM_GPS_MSG_FORMAT = 0x09,
		  CMD_3DM_NAV_MSG_FORMAT = 0x0A
	  };

	  enum glob_descs {

		  DESC_ACK = 0xF1
	  };

	  enum comm_modes {

		  COMM_MODE_MIP = 0x01,
		  COMM_MODE_AHRS = 0x02,
		  COMM_MODE_GPS = 0x03

	  };

	  enum others {

		  MODEL_ID = 0x1854,
		  DATA_AHRS = 0x80,
		  DATA_GPS = 0x81

	  };

    enum models {
      GX3_45 = 0x1854,
      GX3_35 = 0x1851
    };

	  enum functions {

		  FUN_USE_NEW = 0x01,
		  FUN_READ_CURRENT = 0x02,
		  FUN_SAVE_CURR_AS_STARTUP = 0x03,
		  FUN_LOAD_SAVE_STARTUP = 0x04,
		  FUN_RESET_TO_FACTORY_DEF = 0x05
	  };

    models _model;
    
      IMU(int rate, models model=microstrain_3dm_gx3_35::IMU::GX3_35);

      const Ravelin::Origin3d& get_position() const { return _x; }
      const Ravelin::Quatd& get_orientation() const { return _quat; }

      bool openPort(const std::string& port, unsigned int baud_rate, boost::asio::serial_port_base::parity opt_parity=
              boost::asio::serial_port_base::parity(
                  boost::asio::serial_port_base::parity::none),
          boost::asio::serial_port_base::character_size opt_csize=
              boost::asio::serial_port_base::character_size(8),
          boost::asio::serial_port_base::flow_control opt_flow=
              boost::asio::serial_port_base::flow_control(
                  boost::asio::serial_port_base::flow_control::none),
          boost::asio::serial_port_base::stop_bits opt_stop=
              boost::asio::serial_port_base::stop_bits(
                  boost::asio::serial_port_base::stop_bits::one));

      ~IMU();


      bool isOpen() const;

      void closePort();

      void setTimeout(const boost::posix_time::time_duration& t);

      bool ping();

      bool selfTest();

      bool devStatus();

      bool disAllStreams();
      bool enableAHRSStream();
      bool isAHRSBufferEmpty() { return _ahrs_buffer.empty(); }
      tahrs getAHRSItem();

      std::string getLastError();

      bool setAHRSMsgFormat();
      bool setAHRSSignalCond();
      bool setDynamicsMode();

      bool setGPSMsgFormat();

      bool setToIdle();

      bool resume();

      bool pollAHRS();

      bool pollGPS();

      bool setStream(uint8_t stream, bool state);

      tahrs& getAHRS();
      tgps& getGPS();
      static void calcFletcher(std::vector<unsigned char>& arr);
      static bool checkFletcher(const std::vector<unsigned char>& arr);
 
    protected:
      static void* ahrs_thread(void* data); 
      void readFromAHRSStream();
      int _rate;
      bool _thread_running;
      pthread_mutex_t _mutex;
      tahrs _ahrs_data;
      tgps _gps_data;

      std::queue<tahrs> _ahrs_buffer;
      static unsigned char sync1;
      static unsigned char sync2;

      bool checkACK(const std::vector<unsigned char>& arr, uint8_t cmd_set, uint8_t cmd);

      bool sendNoDataCmd(uint8_t cmd_set, uint8_t cmd);

      std::vector<std::string> error_desc;

      void errMsg(const std::string& msg);

      class ParameterReader
      {
        public:
          ParameterReader(): fixedSize(false), delim(""), data(0), size(0) {}

          explicit ParameterReader(const std::string& delim):
                  fixedSize(false), delim(delim), data(0), size(0) { }

          ParameterReader(unsigned char *data, size_t size): fixedSize(true),
                  delim(""), data(data), size(size) { }

          //Using default copy constructor, operator=

          bool fixedSize; ///< True if need to read a fixed number of parameters
          std::string delim; ///< String end delimiter (valid if fixedSize=false)
          unsigned char *data; ///< Pointer to data array (valid if fixedSize=true)
          size_t size; ///< Array size (valid if fixedSize=true)
      };

      void performReadSetup(const ParameterReader& param);

      boost::asio::io_service io;
      boost::asio::serial_port serial;
      boost::asio::deadline_timer timer; // timer for timeout
      boost::posix_time::time_duration timeout;
      boost::asio::streambuf readData;

      size_t bytesTransferred;
      ParameterReader setupParameters;

      void read(char *data, size_t size);
      std::string readStringUntil(const std::string& delim="ue");

      void waitForMsg();

      void write(const std::vector<unsigned char>& data);

      void read(size_t size, std::vector<unsigned char>& output);

      void timeoutExpired(const boost::system::error_code& error);

      void readCompleted(const boost::system::error_code& error, const size_t bytesTransferred);

      enum ReadResult { resultInProgress, resultSuccess, resultError, resultTimeoutExpired};

      enum ReadResult result;

      float extractFloat(unsigned char* addr);
      unsigned extractUInt(unsigned char* addr);
      double extractDouble(unsigned char* addr);
      void encodeFloat(float in, std::vector<unsigned char>& arr);

      Ravelin::Origin3d _x, _xd, _xdd, _omega;
      Ravelin::Quatd _quat;
  };

} // namespace

#endif
