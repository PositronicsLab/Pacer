#include "driver.h"

#include <algorithm>
#include <iostream>
#include <boost/bind.hpp>

using namespace microstrain_3dm_gx3_35;
using namespace std;
using namespace boost;

/// Gets the current time (as a floating-point number)
static double get_current_time()
{
  timeval t;
  gettimeofday(&t, NULL);
  return (unsigned long) t.tv_sec * 1000000 + (unsigned long) t.tv_usec;
}

IMU::IMU(int rate, models model) : 
  io(), serial(io), timer(io), timeout(posix_time::seconds(0)) ,
  model_(model)
{

  sync1 = 0x75;
  sync2 = 0x65;
  
  rate_ = rate; // TODO do some check if the value is reasonable

  comm_mode = -1;

  //cout<<"class created"<<endl;
  //model_ = microstrain_3dm_gx3_45::IMU::GX3_35;
}

IMU::~IMU() {

  //cout<<"class destroyed"<<endl;

}

bool IMU::openPort(string port, unsigned int baud_rate, boost::asio::serial_port_base::parity opt_parity,
    boost::asio::serial_port_base::character_size opt_csize,
    boost::asio::serial_port_base::flow_control opt_flow,
    boost::asio::serial_port_base::stop_bits opt_stop) {

	if(isOpen()) closePort();

	try {

		serial.open(port);
		serial.set_option(asio::serial_port_base::baud_rate(baud_rate));
		serial.set_option(opt_parity);
		serial.set_option(opt_csize);
		serial.set_option(opt_flow);
		serial.set_option(opt_stop);

	} catch(boost::system::system_error& e) {

		  cout<<"Error: "<<e.what()<<endl;
		  return false;
	}


	return true;

}

bool IMU::isOpen() const {

	return serial.is_open();

}

void IMU::closePort() {

	if(isOpen()==false) return;
	serial.close();

}

void IMU::setTimeout(const posix_time::time_duration& t)
{
    timeout=t;
}

void IMU::write(const tbyte_array& data)
{
    asio::write(serial,asio::buffer(&data[0],data.size()));
}

void IMU::timeoutExpired(const boost::system::error_code& error)
{
     if(!error && result==resultInProgress) result=resultTimeoutExpired;
}

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

tbyte_array IMU::read(size_t size)
{
    tbyte_array result(size,'\0');//Allocate a vector with the desired size
    read(&result[0],size);//Fill it with values
    return result;
}

void IMU::waitForMsg() {

/* try {

		readStringUntil();

	} catch(boost::system::system_error& e) {

		return;

	    }*/

	tbyte_array recv; // TODO just for testing!!!!! rewrite it

	char prev = ' ';

	do {

		if (recv.size() > 0) prev = recv[0];
		else prev = ' ';

		recv.clear();
		recv = read(1);

	} while (!(prev=='u' && recv[0]=='e'));


}

void IMU::read(char *data, size_t size)
{
    if(readData.size()>0)//If there is some data from a previous read
    {
    	basic_istream<char> is(&readData);
        size_t toRead=min(readData.size(),size);//How many bytes to read?
        is.read(data,toRead);
        data+=toRead;
        size-=toRead;
        if(size==0) return;//If read data was enough, just return
    }

    setupParameters=ReadSetupParameters(data,size);
    performReadSetup(setupParameters);

    //For this code to work, there should always be a timeout, so the
    //request for no timeout is translated into a very long timeout
    if(timeout!=posix_time::seconds(0)) timer.expires_from_now(timeout);
    else timer.expires_from_now(posix_time::hours(100000));

    timer.async_wait(boost::bind(&IMU::timeoutExpired,this,
                asio::placeholders::error));

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

void IMU::performReadSetup(const ReadSetupParameters& param)
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

/*string IMU::readStringUntil(const string& delim)
{
    // Note: if readData contains some previously read data, the call to
    // async_read_until (which is done in performReadSetup) correctly handles
    // it. If the data is enough it will also immediately call readCompleted()
    setupParameters=ReadSetupParameters(delim);
    performReadSetup(setupParameters);

    //For this code to work, there should always be a timeout, so the
    //request for no timeout is translated into a very long timeout
    if(timeout!=posix_time::seconds(0)) timer.expires_from_now(timeout);
    else timer.expires_from_now(posix_time::hours(100000));

    timer.async_wait(boost::bind(&IMU::timeoutExpired,this,
                asio::placeholders::error));

    result=resultInProgress;
    bytesTransferred=0;
    for(;;)
    {
        io.run_one();
        switch(result)
        {
            case resultSuccess:
                {
                    timer.cancel();
                    bytesTransferred-=delim.size();//Don't count delim
                    istream is(&readData);
                    string result(bytesTransferred,'\0');//Alloc string
                    is.read(&result[0],bytesTransferred);//Fill values
                    is.ignore(delim.size());//Remove delimiter from stream
                    return result;
                }
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
}*/

bool IMU::ping() {

	return sendNoDataCmd(CMD_SET_BASIC, CMD_BASIC_PING);

}

bool IMU::pollGPS() {

	tbyte_array data;

	data.push_back(sync1);
	data.push_back(sync2);
	data.push_back(CMD_SET_3DM); // desc set
	data.push_back(0x04); // length
	data.push_back(0x04);
	data.push_back(CMD_3DM_POLL_GPS);
	data.push_back(0x1); // suppress ACK
	data.push_back(0x0);

	crc(data);
	write(data);
	waitForMsg();

	tbyte_array recv;

	size_t n = 48; // 44+6, TODO this is really stupid... there must be some parsing etc....

//	struct timespec curtime;
//	clock_gettime(CLOCK_REALTIME, &curtime);
  double current_time = get_current_time();
	recv = read(n);

	if (!crcCheck(recv)) return false;

	if (recv[2] != 0x2C || recv[3] != 0x03) {

		errMsg("GPS: Wrong msg format.");
		return false;

	}

//	gps_data_.time =  (uint64_t)(curtime.tv_sec) * 1000000000 + (uint64_t)(curtime.tv_nsec);

  gps_data_.time = current_time;
	gps_data_.latitude = extractDouble(&recv[4]);
	gps_data_.longtitude = extractDouble(&recv[4+8]);
	gps_data_.horizontal_accuracy = extractFloat(&recv[4+32]);

	uint16_t flags = /*((uint16_t)recv[6+40])<<2 | */(uint16_t)recv[4+41];

	gps_data_.lat_lon_valid = (flags & 0x1);
	gps_data_.hor_acc_valid = (flags & (0x1<<5));

	return true;
}

bool IMU::pollNAV() {

	tbyte_array data;

	data.push_back(sync1);
	data.push_back(sync2);
	data.push_back(CMD_SET_3DM); // desc set
	data.push_back(0x04); // length
	data.push_back(0x04);
	data.push_back(CMD_3DM_POLL_NAV);
	data.push_back(0x1); // suppress ACK
	data.push_back(0x0);

	crc(data);
	write(data);
	waitForMsg();

	tbyte_array recv;

	size_t n = 148+4; // TODO this is really (?) stupid... there must be some parsing etc....

  //	struct timespec curtime;
  //	clock_gettime(CLOCK_REALTIME, &curtime);
  double current_time = get_current_time();

	recv = read(n);

	//cout << (0xff & unsigned(recv[1])) << endl;

	if (!crcCheck(recv)) return false;

//	nav_data_.time =  (uint64_t)(curtime.tv_sec) * 1000000000 + (uint64_t)(curtime.tv_nsec);
  nav_data_.time = current_time;

	if (recv[2] != 0x08 || recv[3] != 0x10) { // check length of each field and its descriptor

		errMsg("Wrong msg format (0x10).");
		return false;

	}

	// filter state
	nav_data_.filter_state = ((uint16_t)recv[4]<<8) | (uint16_t)recv[5];
	nav_data_.filter_status_flags = ((uint16_t)recv[8]<<8) | (uint16_t)recv[9];


	if (recv[10] != 28 || recv[11] != 0x01) {

			errMsg("Wrong msg format (0x01).");
			return false;

		}

	nav_data_.est_latitude = extractDouble(&recv[12]);
	nav_data_.est_longtitude = extractDouble(&recv[12+8]);
	nav_data_.est_height = extractDouble(&recv[12+16]);

	if (recv[12+24+1]) nav_data_.est_llh_valid = true;
	else nav_data_.est_llh_valid = false;


	if (recv[38] != 16 || recv[39] != 0x02) {

		errMsg("Wrong msg format (0x02).");
		return false;

	}

	nav_data_.est_vel_north = extractFloat(&recv[40]);
	nav_data_.est_vel_east = extractFloat(&recv[40+4]);
	nav_data_.est_vel_down = extractFloat(&recv[40+8]);

	if (recv[40+12+1]) nav_data_.est_ned_valid = true;
	else nav_data_.est_ned_valid = false;

	if (recv[54] != 16 || recv[55] != 0x05) {

		errMsg("Wrong msg format (0x05).");
		return false;

	}

	nav_data_.est_r = extractFloat(&recv[56]);
	nav_data_.est_p = extractFloat(&recv[56+4]);
	nav_data_.est_y = extractFloat(&recv[56+8]);

	if (recv[56+12+1]) nav_data_.est_rpy_valid = true;
	else nav_data_.est_rpy_valid = false;


	if (recv[70] != 16 || recv[71] != 0x08) {

		errMsg("Wrong msg format (0x08).");
		return false;

	}

	nav_data_.est_north_pos_unc = extractFloat(&recv[72]);
	nav_data_.est_east_pos_unc = extractFloat(&recv[72+4]);
	nav_data_.est_down_pos_unc = extractFloat(&recv[72+8]);

	if (recv[72+12+1]) nav_data_.est_pos_unc_valid = true;
	else nav_data_. est_pos_unc_valid = false;

	if (recv[86] != 16 || recv[87] != 0x09) {

		errMsg("Wrong msg format (0x09).");
		return false;

	}

	nav_data_.est_north_vel_unc = extractFloat(&recv[88]);
	nav_data_.est_east_vel_unc = extractFloat(&recv[88+4]);
	nav_data_.est_down_vel_unc = extractFloat(&recv[88+8]);

	if (recv[88+12+1]) nav_data_.est_vel_unc_valid = true;
	else nav_data_. est_vel_unc_valid = false;

	if (recv[102] != 16 || recv[103] != 0x0A) {

		errMsg("Wrong msg format (0x0A).");
		return false;

	}

	nav_data_.est_r_unc = extractFloat(&recv[104]);
	nav_data_.est_p_unc = extractFloat(&recv[104+4]);
	nav_data_.est_y_unc = extractFloat(&recv[104+8]);

	if (recv[104+12+1]) nav_data_.est_rpy_unc_valid = true;
	else nav_data_. est_rpy_unc_valid = false;

	if (recv[118] != 16 || recv[119] != 0x0D) {

		errMsg("Wrong msg format (0x0D).");
		return false;

	}

	nav_data_.est_acc_lin_x = extractFloat(&recv[120]);
	nav_data_.est_acc_lin_y = extractFloat(&recv[120+4]);
	nav_data_.est_acc_lin_z = extractFloat(&recv[120+8]);

	if (recv[120+12+1]) nav_data_.est_acc_lin_valid = true;
	else nav_data_.est_acc_lin_valid = false;

	if (recv[134] != 16 || recv[135] != 0x0E) {

		errMsg("Wrong msg format (0x0E).");
		return false;

	}

	nav_data_.est_acc_rot_x = extractFloat(&recv[136]);
	nav_data_.est_acc_rot_y = extractFloat(&recv[136+4]);
	nav_data_.est_acc_rot_z = extractFloat(&recv[136+8]);

	if (recv[136+12+1]) nav_data_.est_acc_rot_valid = true;
	else nav_data_.est_acc_rot_valid = false;

	return true;

}

tnav IMU::getNAV() {

	return nav_data_;

}

bool IMU::pollAHRS() {

	tbyte_array data;

	data.push_back(sync1);
	data.push_back(sync2);
	data.push_back(CMD_SET_3DM); // desc set
	data.push_back(0x04); // length
	data.push_back(0x04);
	data.push_back(CMD_3DM_POLL_AHRS);
	data.push_back(0x1); // suppress ACK
	data.push_back(0x0);

	crc(data);
	write(data);
	waitForMsg();

	tbyte_array recv;

	size_t n = 46; // TODO this is really stupid... there must be some parsing etc....

  //	struct timespec curtime;
  //	clock_gettime(CLOCK_REALTIME, &curtime);
  double current_time = get_current_time();

	recv = read(n);

	if (!crcCheck(recv)) return false;

	//if (!checkACK(recv,CMD_SET_3DM,CMD_3DM_POLL_AHRS)) return false;

	// quaternion 0x0A, field length 18, MSB first

	//quat.time = posix_time::microsec_clock::local_time();

	if (recv[2] != 0x0E || recv[3] != 0x04) {

		errMsg("AHRS: Wrong msg format (0x04).");
		return false;

	}

//	ahrs_data_.time =  (uint64_t)(curtime.tv_sec) * 1000000000 + (uint64_t)(curtime.tv_nsec);
  ahrs_data_.time = current_time;

	ahrs_data_.ax = extractFloat(&recv[4]); // 0x04
	ahrs_data_.ay = extractFloat(&recv[4+4]);
	ahrs_data_.az = extractFloat(&recv[4+8]);

	if (recv[16] != 0x0E || recv[17] != 0x05) {

		errMsg("AHRS: Wrong msg format (0x05).");
		return false;

	}

	ahrs_data_.gx = extractFloat(&recv[18]); // 0x05
	ahrs_data_.gy = extractFloat(&recv[18+4]);
	ahrs_data_.gz = extractFloat(&recv[18+8]);

	if (recv[30] != 0x0E || recv[31] != 0x0C) {

		errMsg("AHRS: Wrong msg format (0x0C).");
		return false;

	}

	ahrs_data_.r = extractFloat(&recv[32]); // 0x0C
	ahrs_data_.p = extractFloat(&recv[32+4]);
	ahrs_data_.y = extractFloat(&recv[32+8]);

	/*quat.q0 = extractFloat(&recv[6]);
	quat.q1 = extractFloat(&recv[6+4]);
	quat.q2 = extractFloat(&recv[6+8]);
	quat.q3 = extractFloat(&recv[6+12]);*/

	//cout << quat.q0 << " " << quat.q1 << " " << quat.q2 << " " << quat.q3 << endl;

	return true;

}

tahrs IMU::getAHRS() {

	return ahrs_data_;

}

tgps IMU::getGPS() {

	return gps_data_;

}

float IMU::extractFloat(char* addr) {

  float tmp;

  *((unsigned char*)(&tmp) + 3) = *(addr);
  *((unsigned char*)(&tmp) + 2) = *(addr+1);
  *((unsigned char*)(&tmp) + 1) = *(addr+2);
  *((unsigned char*)(&tmp)) = *(addr+3);

  return tmp;
}

void IMU::encodeFloat(tbyte_array& arr, float in) {

  arr.push_back(*((unsigned char*)(&in) + 3));
  arr.push_back(*((unsigned char*)(&in) + 2));
  arr.push_back(*((unsigned char*)(&in) + 1));
  arr.push_back(*((unsigned char*)(&in)));

}

double IMU::extractDouble(char* addr) {

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


bool IMU::setNAVMsgFormat() {

	tbyte_array data;

	data.push_back(sync1);
	data.push_back(sync2);
	data.push_back(CMD_SET_3DM); // desc set
	data.push_back(0x1F); // length
	data.push_back(0x1F);
	data.push_back(CMD_3DM_NAV_MSG_FORMAT);

	data.push_back(FUN_USE_NEW);
	data.push_back(0x09); // desc count

	data.push_back(0x10); // Filter Status (0x82, 0x10) -> 8 B
	data.push_back(0x0);
	data.push_back(100/rate_);

	data.push_back(0x01); // Estimated LLH Position (0x82, 0x01) -> 28 B
	data.push_back(0x0);
	data.push_back(100/rate_);

	data.push_back(0x02); // Estimated NED Velocity (0x82, 0x02) -> 16 B
	data.push_back(0x0);
	data.push_back(100/rate_);

	data.push_back(0x05); // Estimated Orientation, Euler Angles (0x82, 0x05) -> 16 B
	data.push_back(0x0);
	data.push_back(100/rate_);

	data.push_back(0x08);  // Estimated LLH Position Uncertainty (0x82, 0x08) -> 16 B
	data.push_back(0x0);
	data.push_back(100/rate_);

	data.push_back(0x09); // Estimated NED Velocity Uncertainty (0x82, 0x09) -> 16 B
	data.push_back(0x0);
	data.push_back(100/rate_);

	data.push_back(0x0A); // Estimated Attitude Uncertainty, Euler Angles (0x82, 0x0A) -> 16 B
	data.push_back(0x0);
	data.push_back(100/rate_);

	data.push_back(0x0D); // Estimated Linear Acceleration (0x82, 0x0D) -> 16 B
	data.push_back(0x0);
	data.push_back(100/rate_);

	data.push_back(0x0E); // Estimated Angular Rate (0x82, 0x0E) -> 16 B
	data.push_back(0x0);
	data.push_back(100/rate_);

	crc(data);
	write(data);
	waitForMsg();

	tbyte_array recv;
	size_t n = 8;

	recv = read(n);

	if (!crcCheck(recv)) return false;
	if (!checkACK(recv,CMD_SET_3DM,CMD_3DM_NAV_MSG_FORMAT)) return false;

	return true;

}

bool IMU::setGPSMsgFormat() {

	tbyte_array data;

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

	crc(data);
	write(data);
	waitForMsg();

	tbyte_array recv;

	size_t n = 8;
	recv = read(n);

	if (!crcCheck(recv)) return false;
	if (!checkACK(recv,CMD_SET_3DM,CMD_3DM_GPS_MSG_FORMAT)) return false;

	return true;
}

bool IMU::setAHRSMsgFormat() {

	tbyte_array data;

	data.push_back(sync1);
	data.push_back(sync2);
	data.push_back(CMD_SET_3DM); // desc set
	data.push_back(0x0D); // length
	data.push_back(0x0D);
	data.push_back(CMD_3DM_AHRS_MSG_FORMAT);

	data.push_back(FUN_USE_NEW);
	data.push_back(0x03); // desc count

	data.push_back(0x04); // accelerometer vector
	data.push_back(0x0);
	data.push_back(100/rate_); // 20 Hz

	data.push_back(0x05); // gyro vector
	data.push_back(0x0);
	data.push_back(100/rate_); // 20 Hz

	data.push_back(0x0C); // euler angles
	data.push_back(0x0);
	data.push_back(100/rate_); // rate decimation -> 20 Hz

	crc(data);
	write(data);
	waitForMsg();

	tbyte_array recv;

	size_t n = 8;

	recv = read(n);

	if (!crcCheck(recv)) return false;

	if (!checkACK(recv,CMD_SET_3DM,CMD_3DM_AHRS_MSG_FORMAT)) return false;

	return true;

}

bool IMU::initKalmanFilter(float decl) {

	tbyte_array data;

	data.push_back(sync1);
	data.push_back(sync2);
	data.push_back(CMD_SET_NAVFILTER); // desc set
	data.push_back(0x06); // length
	data.push_back(0x06);
	data.push_back(CMD_NAV_SET_INIT_FROM_AHRS);

	/*data.push_back((decl>>16)&0xff); // MSB
	data.push_back((decl>>8)&0xff);
	data.push_back((decl>>4)&0xff);
	data.push_back((decl)&0xff); // LSB*/

	encodeFloat(data,decl);

	crc(data);
	write(data);
	waitForMsg();

	tbyte_array recv;

	size_t n = 8;

	recv = read(n);

	if (!crcCheck(recv)) return false;

	if (!checkACK(recv,CMD_SET_NAVFILTER,CMD_NAV_SET_INIT_FROM_AHRS)) return false;

	return true;

}

bool IMU::resume() {

	return sendNoDataCmd(CMD_SET_BASIC, CMD_BASIC_RESUME);

}


bool IMU::setToIdle() {

	return sendNoDataCmd(CMD_SET_BASIC, CMD_BASIC_SET_TO_IDLE);

}

bool IMU::sendNoDataCmd(uint8_t cmd_set, uint8_t cmd) {

	tbyte_array data;

	data.push_back(sync1);
	data.push_back(sync2);
	data.push_back(cmd_set); // desc set
	data.push_back(0x02); // length
	data.push_back(0x02);
	data.push_back(cmd);

	crc(data);
	write(data);

	tbyte_array recv;

	size_t n = 8;

	waitForMsg();
	cout << "do some reading..." << endl;
	recv = read(n);
	cout << "we have some data..." << endl;

	if (!crcCheck(recv)) return false;

	if (!checkACK(recv,cmd_set,cmd)) return false;

	return true;

}

bool IMU::selfTest() {

	posix_time::time_duration timeout_orig = timeout;

	setTimeout(posix_time::seconds(6));

	tbyte_array data;

	data.push_back(sync1);
	data.push_back(sync2);
	data.push_back(CMD_SET_BASIC);
	data.push_back(0x02);
	data.push_back(0x02);
	data.push_back(CMD_BASIC_DEV_BUILTIN_TEST);

	crc(data);
	write(data);
	waitForMsg();

	tbyte_array recv;
	size_t n = 14;

	recv = read(n);

	if (!crcCheck(recv)) {

		setTimeout(timeout_orig);
		return false;

	}

	setTimeout(timeout_orig);

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

	tbyte_array data;

	data.push_back(sync1);
	data.push_back(sync2);
	data.push_back(CMD_SET_3DM);
	data.push_back(0x05);
	data.push_back(0x05);
	data.push_back(CMD_3DM_DEV_STATUS);
	//data.push_back(((uint16_t)MODEL_ID>>2) & 0xff);
	//data.push_back((uint16_t)MODEL_ID & 0xff);
  if (model_ == GX3_45){
    data.push_back(0x18);
    data.push_back(0x54);
  }else if( model_ == GX3_35){
    data.push_back(0x18);
    data.push_back(0x51);
  }
	data.push_back(0x01); // basic status


	crc(data);
	write(data);
  cout<<"devStatus waiting"<<endl;
	waitForMsg();

	tbyte_array recv;
	size_t n = 25;

	recv = read(n);

	if (!crcCheck(recv)) {

		return false;

	}

	if (!checkACK(recv,CMD_SET_3DM, CMD_3DM_DEV_STATUS)) return false;

	//if (((uint8_t)recv[10] != ((MODEL_ID>>2)&0xff)) || ((uint8_t)recv[11] != (MODEL_ID & 0xff))) {
	if ((model_ == GX3_45 && (((uint8_t)recv[8] != 0x18) || ((uint8_t)recv[9] != 0x54))) 
      ||(model_ == GX3_35 && (((uint8_t)recv[8] != 0x18) || ((uint8_t)recv[9] != 0x51))) ){

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

	tbyte_array data;

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


	crc(data);
	write(data);
	waitForMsg();

	tbyte_array recv;
	size_t n = 8;

	recv = read(n);

	if (!crcCheck(recv)) {

		return false;

	}

	if (!checkACK(recv,CMD_SET_3DM, CMD_3DM_STREAM_STATE)) return false;

	return true;

}

bool IMU::disAllStreams() {

	bool ret = true;

	if (!setStream(0x01,false)) ret = false; // AHRS
	if (!setStream(0x02,false)) ret = false; // GPS
  if (model_ == GX3_45)
  {
	  if (!setStream(0x03,false)) ret = false; // NAV
  }
	return ret;

}

void IMU::errMsg(std::string msg) {

	error_desc.push_back(msg);

}

bool IMU::checkACK(tbyte_array& arr, uint8_t cmd_set, uint8_t cmd) {

	if (arr.size() < 6) {

		errMsg("Too short reply.");
		return false;

	}

	/*if (arr[0] != sync1 || arr[1] != sync2) {

		errMsg("Strange synchronization bytes.");
		return false;

	}*/

	if (arr[0] != cmd_set) {

		errMsg("Wrong desc set in reply.");
		return false;

	}

	if (arr[4] != cmd) {

		errMsg("Wrong command echo.");
		return false;

	}

	if (arr[5] != 0x0) {

		errMsg("NACK.");
		return false;

	}

	return true;

}

string IMU::getLastError() {

	if (error_desc.size() > 0) {

		string tmp;

		tmp = error_desc.back();
		error_desc.pop_back();
		return tmp;

	} else return "";

}

bool IMU::crcCheck(tbyte_array& arr) {

	unsigned char b1=0;
	unsigned char b2=0;

	//cout << arr.size() << endl;

	if ( ((uint8_t)arr[1]+4) != (uint8_t)arr.size() ) {

		cout << "Sizes mismatch." << endl;

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

	/*for(unsigned int i=0; i<(arr.size()); i++)
	cout << static_cast<int>(arr[i]) << " ";
	cout << endl;*/


	if (b1==(unsigned char)arr[arr.size()-2] && b2==(unsigned char)arr[arr.size()-1]) return true;
	else {

		errMsg("Bad CRC.");
		return false;

	}

}

void IMU::crc(tbyte_array& arr) {

	char b1=0;
	char b2=0;

	for(unsigned int i=0; i<arr.size(); i++)
	{
	 b1 += arr[i];
	 b2 += b1;
	}

	arr.push_back(b1);
	arr.push_back(b2);

}
