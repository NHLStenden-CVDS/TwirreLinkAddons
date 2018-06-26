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

#ifndef GPS_SENSOR_H_
#define GPS_SENSOR_H_

#include <thread>

#include <Core/Sensor.h>
#include <Core/Value.h>

namespace piksigps
{
	class PiksiManager;
}

namespace twirre
{
	class PiksiGPS: public Sensor
	{
	public:
		PiksiGPS(std::string name, const char * path);
		virtual ~PiksiGPS();

	protected:
		piksigps::PiksiManager * _gps;

		bool _stopThread = true;
		std::thread _updateThread;
		void thread_main();

		ValueImpl<uint8_t> _HB_systemError { "hb_systemerror", 2 };
		ValueImpl<uint8_t> _HB_ioError { "hb_ioerror", 2 };
		ValueImpl<uint8_t> _HB_napError { "hb_naperror", 2 };
		ValueImpl<uint8_t> _HB_extAntenna { "hb_extantenna", 2 };

		ValueImpl<uint16_t> _GPSTIME_weeknumber { "gpstime_weekno", 0 };
		ValueImpl<uint32_t> _GPSTIME_tow_ms { "gpstime_tow_ms", 0 };
		ValueImpl<int32_t> _GPSTIME_tow_ns { "gpstime_tow_ns", -1 };

		ValueImpl<uint32_t> _DOPS_tow_ms { "dops_tow_ms", 0 };
		ValueImpl<uint16_t> _DOPS_gdop { "dops_gdop", 0 };
		ValueImpl<uint16_t> _DOPS_pdop { "dops_pdop", 0 };
		ValueImpl<uint16_t> _DOPS_tdop { "dops_tdop", 0 };
		ValueImpl<uint16_t> _DOPS_hdop { "dops_hdop", 0 };
		ValueImpl<uint16_t> _DOPS_vdop { "dops_vdop", 0 };

		ValueImpl<uint32_t> _POS_ECEF_tow_ms { "pos_ecef_tow_ms", 0 };
		ValueImpl<double> _POS_ECEF_x { "pos_ecef_x", 0 };
		ValueImpl<double> _POS_ECEF_y { "pos_ecef_y", 0 };
		ValueImpl<double> _POS_ECEF_z { "pos_ecef_z", 0 };
		ValueImpl<uint16_t> _POS_ECEF_accuracy { "pos_ecef_accuracy", 0 };
		ValueImpl<uint8_t> _POS_ECEF_numsats { "pos_ecef_numsats", 0 };
		ValueImpl<uint8_t> _POS_ECEF_fixmode { "pos_ecef_fixmode", 0 };

		ValueImpl<uint32_t> _POS_LLH_tow_ms { "pos_llh_tow_ms", 0 };
		ValueImpl<double> _POS_LLH_lat { "pos_llh_lat", 0 };
		ValueImpl<double> _POS_LLH_lon { "pos_llh_lon", 0 };
		ValueImpl<double> _POS_LLH_height { "pos_llh_height", 0 };
		ValueImpl<uint16_t> _POS_LLH_h_accuracy { "pos_llh_h_accuracy", 0 };
		ValueImpl<uint16_t> _POS_LLH_v_accuracy { "pos_llh_v_accuracy", 0 };
		ValueImpl<uint8_t> _POS_LLH_numsats { "pos_llh_numsats", 0 };
		ValueImpl<uint8_t> _POS_LLH_fixmode { "pos_llh_fixmode", 0 };
		ValueImpl<uint8_t> _POS_LLH_heightmode { "pos_llh_heightmode", 0 };

		ValueImpl<uint32_t> _BASE_ECEF_tow_ms { "base_ecef_tow_ms", 0 };
		ValueImpl<int32_t> _BASE_ECEF_x { "base_ecef_x", 0 };
		ValueImpl<int32_t> _BASE_ECEF_y { "base_ecef_y", 0 };
		ValueImpl<int32_t> _BASE_ECEF_z { "base_ecef_z", 0 };
		ValueImpl<uint16_t> _BASE_ECEF_accuracy { "base_ecef_accuracy", 0 };
		ValueImpl<uint8_t> _BASE_ECEF_numsats { "base_ecef_numsats", 0 };
		ValueImpl<uint8_t> _BASE_ECEF_fixmode { "base_ecef_fixmode", 0 };

		ValueImpl<uint32_t> _BASE_NED_tow_ms { "base_ned_tow_ms", 0 };
		ValueImpl<int32_t> _BASE_NED_n { "base_ned_n", 0 };
		ValueImpl<int32_t> _BASE_NED_e { "base_ned_e", 0 };
		ValueImpl<int32_t> _BASE_NED_d { "base_ned_d", 0 };
		ValueImpl<uint16_t> _BASE_NED_h_accuracy { "base_ned_h_accuracy", 0 };
		ValueImpl<uint16_t> _BASE_NED_v_accuracy { "base_ned_v_accuracy", 0 };
		ValueImpl<uint8_t> _BASE_NED_numsats { "base_ned_numsats", 0 };
		ValueImpl<uint8_t> _BASE_NED_fixmode { "base_ned_fixmode", 0 };

		ValueImpl<uint32_t> _VEL_ECEF_tow_ms { "vel_ecef_tow_ms", 0 };
		ValueImpl<int32_t> _VEL_ECEF_x { "vel_ecef_x", 0 };
		ValueImpl<int32_t> _VEL_ECEF_y { "vel_ecef_y", 0 };
		ValueImpl<int32_t> _VEL_ECEF_z { "vel_ecef_z", 0 };
		ValueImpl<uint16_t> _VEL_ECEF_accuracy { "vel_ecef_accuracy", 0 };
		ValueImpl<uint8_t> _VEL_ECEF_numsats { "vel_ecef_numsats", 0 };

		ValueImpl<uint32_t> _VEL_NED_tow_ms { "vel_ned_tow_ms", 0 };
		ValueImpl<int32_t> _VEL_NED_n { "vel_ned_n", 0 };
		ValueImpl<int32_t> _VEL_NED_e { "vel_ned_e", 0 };
		ValueImpl<int32_t> _VEL_NED_d { "vel_ned_d", 0 };
		ValueImpl<uint16_t> _VEL_NED_h_accuracy { "vel_ned_h_accuracy", 0 };
		ValueImpl<uint16_t> _VEL_NED_v_accuracy { "vel_ned_v_accuracy", 0 };
		ValueImpl<uint8_t> _VEL_NED_numsats { "vel_ned_numsats", 0 };
	};

} /* namespace twirre */

#endif /* GPS_SENSOR_H_ */
