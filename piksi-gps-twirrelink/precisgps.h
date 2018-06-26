/*
 * Twirre: architecture for autonomous UAVs using interchangeable commodity components
 *
 * Copyright© 2017 Centre of expertise in Computer Vision & Data Science, NHL Stenden University of applied sciences
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef PRECISGPS_H_
#define PRECISGPS_H_

#include <string>
#include <functional>
#include <thread>
#include <Serial/SerialRW.h>

namespace precisgps
{
	enum GPSQuality
	{
		UNKNOWN = 0,
		SPP = 1,
		DGPS = 2,
		PPS_INVALID = 3,
		RTK_FIXED = 4,
		RTK_FLOAT = 5,
		ESTIMATE = 6,
		USER_CONSTRAINED = 7,
		SIMULATION = 8,
		WAAS = 9
	};

	struct utctime
	{
		uint8_t hours;
		uint8_t minutes;
		uint8_t seconds;
		uint32_t microseconds;
	};

	struct nmea_fixinformation
	{
		utctime time;				//time of fix data in UTC time
		double latitude;			//format: degrees north of equator
		double longitude;			//format: degrees west of prime meridian
		GPSQuality gpsMode;			//current GPS operating mode
		uint16_t satellitesInView;	//number of satellites currently in view
		float HDOPS;				//horizontal dilution of precision
		float altitude_amsl; 		//true altitude of antenna above MSL
		float geoidalSeparation; 	//difference between WGS-84 ellipsoid and MSL in meters
		float differential_age;		//time since last differential data update
		uint16_t differential_id;	//ID of differential reference station
	};

	enum satellite_constellation
	{
		gps = 0,
		glonass = 1,
		beidou = 2,
		galileo = 3
	};

	struct nmea_satelliteinformation
	{
		satellite_constellation constellation;	//GNSS constellation this satellite belongs to
		int number;								//nmea-id of satellite (PRN - 87)
		int elevation;							//degrees
		int azimuth;							//degrees to true
		int SNR;								//reception quality in dB
	};

	class precisgps
	{
	public:
		precisgps(const std::string & devicePath);
		virtual ~precisgps();

		nmea_fixinformation getFixInformation();
		std::vector<nmea_satelliteinformation> getGPSSatInfo();
		std::vector<nmea_satelliteinformation> getGlonassSatInfo();
		std::vector<nmea_satelliteinformation> getBeidouSatInfo();
		std::vector<nmea_satelliteinformation> getCombinedSatInfo();
		void setDebugOutput(bool enable);

	private:

		bool _dbgOut = false;

		bool _rxThreadRun = false;
		std::thread* _rxThread = nullptr;	//this thread will handle data receiving
		void rxThreadMain();

		void handleNMEA(const std::string & talker, const std::string & type, const std::vector<std::string> & parts );
		void handleGGA(const std::string & talker, const std::vector<std::string> & parts);
		void handleGSV(const std::string & talker, const std::vector<std::string> & parts);
		void handleGSV_constellation(const std::vector<std::string> & parts, satellite_constellation constellation, int & msgcounter, std::vector<nmea_satelliteinformation> & satInfosBuild, std::vector<nmea_satelliteinformation> & satInfosFinal);

		/* gps data */
		std::mutex _gpsDataMutex;
		nmea_fixinformation _fixInfo;
		std::vector<nmea_satelliteinformation> _GPSSatInfo;
		std::vector<nmea_satelliteinformation> _GlonassSatInfo;
		std::vector<nmea_satelliteinformation> _BeidouSatInfo;

		/* thread local vars */
		SerialRW* _rxSerial = nullptr;
		int _gpsInfoCount = -1;
		int _glonassInfoCount = -1;
		int _beidouInfoCount = -1;
		std::vector<nmea_satelliteinformation> _buildGPSSatInfo;
		std::vector<nmea_satelliteinformation> _buildGlonassSatInfo;
		std::vector<nmea_satelliteinformation> _buildBeidouSatInfo;

	};

}

#endif /* PRECISGPS_H_ */
