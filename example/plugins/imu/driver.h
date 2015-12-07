#ifndef MS_3DMGX3_35_HH
#define MS_3DMGX3_35_HH

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

	typedef std::vector<char> tbyte_array;

	typedef struct {

		float ax;
		float ay;
		float az;

		float gx;
		float gy;
		float gz;

		float r;
		float p;
		float y;

		uint64_t time;

	} tahrs;

	typedef struct {

		double latitude;
		double longtitude;

		//double height;

		float horizontal_accuracy;
		//float vertical_accuracy;

		bool lat_lon_valid;
		bool hor_acc_valid;
		//bool height_valid;


		uint64_t time;

	} tgps;

	typedef struct {

		enum filter_states {

			FILTER_STARTUP = 0x00,
			FILTER_INITIALIZATION = 0x01,
			FILTER_RUNNING_VALID = 0x02,
			FILTER_RUNNING_ERROR = 0x03

		};

		enum filter_flags {

			FILTER_ATT_NOT_INITIALIZED = 0x1000,
			FILTER_POS_VEL_NOT_INITIALIZED = 0x2000,
			FILTER_IMU_UNAVAILABLE = 0x0001,
			FILTER_GPS_UNAVAILABLE = 0x0002,
			FILTER_MATRIX_SINGULARITY = 0x0008,
			FILTER_POS_COV_HI = 0x0010,
			FILTER_VEL_COV_HI = 0x0020,
			FILTER_ATT_COV_HI = 0x0040,
			FILTER_NAN_IN_SOLUTION = 0x0080

		};

		uint16_t filter_state; // Filter Status (0x82, 0x10) -> 8 B
		uint16_t filter_status_flags;

		double est_latitude; // Estimated LLH Position (0x82, 0x01) -> 28 B
		double est_longtitude;
		double est_height;
		bool est_llh_valid;

		float est_vel_north; // Estimated NED Velocity (0x82, 0x02) -> 16 B
		float est_vel_east;
		float est_vel_down;
		bool est_ned_valid;

		float est_r; // Estimated Orientation, Euler Angles (0x82, 0x05) -> 16 B
		float est_p;
		float est_y;
		bool est_rpy_valid;

		float est_north_pos_unc; // Estimated LLH Position Uncertainty (0x82, 0x08) -> 16 B
		float est_east_pos_unc;
		float est_down_pos_unc;
		bool est_pos_unc_valid;

		float est_north_vel_unc; // Estimated NED Velocity Uncertainty (0x82, 0x09) -> 16 B
		float est_east_vel_unc;
		float est_down_vel_unc;
		bool est_vel_unc_valid;

		float est_r_unc; // Estimated Attitude Uncertainty, Euler Angles (0x82, 0x0A) -> 16 B
		float est_p_unc;
		float est_y_unc;
		bool est_rpy_unc_valid;

		float est_acc_lin_x; // Estimated Linear Acceleration (0x82, 0x0D) -> 16 B
		float est_acc_lin_y;
		float est_acc_lin_z;
		bool est_acc_lin_valid;

		float est_acc_rot_x; // Estimated Angular Rate (0x82, 0x0E) -> 16 B
		float est_acc_rot_y;
		float est_acc_rot_z;
		bool est_acc_rot_valid;

		uint64_t time;

	} tnav;


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
		  CMD_SET_NAVFILTER = 0x0D,
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

	  enum cmd_set_nav {

		  CMD_NAV_SET_INIT_FROM_AHRS = 0x04

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

    models model_;
    
      IMU(int rate, models model=microstrain_3dm_gx3_35::IMU::GX3_35);

      bool openPort(std::string port, unsigned int baud_rate, boost::asio::serial_port_base::parity opt_parity=
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

      std::string getLastError();

      bool setAHRSMsgFormat();

      bool setGPSMsgFormat();

      bool setNAVMsgFormat();

      bool setToIdle();

      bool resume();

      bool initKalmanFilter(float decl);

      bool pollAHRS();

      bool pollGPS();

      bool pollNAV();

      bool setStream(uint8_t stream, bool state);

      tahrs getAHRS();
      tgps getGPS();
      tnav getNAV();
    
    private:

    protected:
    
      int rate_;

      tahrs ahrs_data_;
      tgps gps_data_;
      tnav nav_data_;

      void crc(tbyte_array& arr);
      bool crcCheck(tbyte_array& arr);

      int comm_mode;

      char sync1;
      char sync2;

      bool checkACK(tbyte_array& arr, uint8_t cmd_set, uint8_t cmd);

      bool sendNoDataCmd(uint8_t cmd_set, uint8_t cmd);

      std::vector<std::string> error_desc;

      void errMsg(std::string msg);

      class ReadSetupParameters
          {
          public:
              ReadSetupParameters(): fixedSize(false), delim(""), data(0), size(0) {}

              explicit ReadSetupParameters(const std::string& delim):
                      fixedSize(false), delim(delim), data(0), size(0) { }

              ReadSetupParameters(char *data, size_t size): fixedSize(true),
                      delim(""), data(data), size(size) { }

              //Using default copy constructor, operator=

              bool fixedSize; ///< True if need to read a fixed number of parameters
              std::string delim; ///< String end delimiter (valid if fixedSize=false)
              char *data; ///< Pointer to data array (valid if fixedSize=true)
              size_t size; ///< Array size (valid if fixedSize=true)
          };

      void performReadSetup(const ReadSetupParameters& param);

      boost::asio::io_service io;
      boost::asio::serial_port serial;
      boost::asio::deadline_timer timer; // timer for timeout
      boost::posix_time::time_duration timeout;
      boost::asio::streambuf readData;

      size_t bytesTransferred;
      ReadSetupParameters setupParameters;

      void read(char *data, size_t size);
      std::string readStringUntil(const std::string& delim="ue");

      void waitForMsg();

      void write(const tbyte_array& data);

      tbyte_array read(size_t size);

      void timeoutExpired(const boost::system::error_code& error);

      void readCompleted(const boost::system::error_code& error, const size_t bytesTransferred);

      enum ReadResult { resultInProgress, resultSuccess, resultError, resultTimeoutExpired};

      enum ReadResult result;

      float extractFloat(char* addr);
      double extractDouble(char* addr);
      void encodeFloat(tbyte_array& arr, float in);

  };

} // namespace

#endif
