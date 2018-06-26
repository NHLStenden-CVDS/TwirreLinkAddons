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

#include "gps_sensor.h"
#include "piksigps.h"

using namespace piksigps;

namespace twirre
{
	PiksiGPS::PiksiGPS(std::string name, const char * path) :
			Sensor(name, "Piksi RTK GPS")
	{
		registerValues( { &_HB_systemError, &_HB_ioError, &_HB_napError, &_HB_extAntenna });
		registerValues( { &_GPSTIME_weeknumber, &_GPSTIME_tow_ms, &_GPSTIME_tow_ns });
		registerValues( { &_DOPS_tow_ms, &_DOPS_gdop, &_DOPS_pdop, &_DOPS_tdop, &_DOPS_hdop, &_DOPS_vdop });
		registerValues( { &_POS_ECEF_tow_ms, &_POS_ECEF_x, &_POS_ECEF_y, &_POS_ECEF_z, &_POS_ECEF_accuracy, &_POS_ECEF_numsats, &_POS_ECEF_fixmode });
		registerValues( { &_POS_LLH_tow_ms, &_POS_LLH_lat, &_POS_LLH_lon, &_POS_LLH_height, &_POS_LLH_h_accuracy, &_POS_LLH_v_accuracy, &_POS_LLH_numsats, &_POS_LLH_fixmode, &_POS_LLH_heightmode });
		registerValues( { &_BASE_ECEF_tow_ms, &_BASE_ECEF_x, &_BASE_ECEF_y, &_BASE_ECEF_z, &_BASE_ECEF_accuracy, &_BASE_ECEF_numsats, &_BASE_ECEF_fixmode });
		registerValues( { &_BASE_NED_tow_ms, &_BASE_NED_n, &_BASE_NED_e, &_BASE_NED_d, &_BASE_NED_h_accuracy, &_BASE_NED_v_accuracy, &_BASE_NED_numsats, &_BASE_NED_fixmode });
		registerValues( { &_VEL_ECEF_tow_ms, &_VEL_ECEF_x, &_VEL_ECEF_y, &_VEL_ECEF_z, &_VEL_ECEF_accuracy, &_VEL_ECEF_numsats });
		registerValues( { &_VEL_NED_tow_ms, &_VEL_NED_n, &_VEL_NED_e, &_VEL_NED_d, &_VEL_NED_h_accuracy, &_VEL_NED_v_accuracy, &_VEL_NED_numsats });

		_gps = new piksigps::PiksiManager(path);

		_stopThread = false;
		_updateThread = std::thread(&PiksiGPS::thread_main, this);
	}

	PiksiGPS::~PiksiGPS()
	{
		_stopThread = true;
		_updateThread.join();
		delete _gps;
	}

	void PiksiGPS::thread_main()
	{
		_gps->setMessageCallback([this](SBP_heartbeat_message msg)
		{
			_HB_systemError.setNativeValue(msg.statusflags.SystemError);
			_HB_ioError.setNativeValue(msg.statusflags.IOError);
			_HB_napError.setNativeValue(msg.statusflags.SwiftNAPError);
			_HB_extAntenna.setNativeValue(msg.statusflags.externalAntennaPresent);
		});

		_gps->setMessageCallback([this](SBP_GPSTime_message msg)
		{
			_GPSTIME_weeknumber.setNativeValue(msg.weekNumber);
			_GPSTIME_tow_ms.setNativeValue(msg.tow_ms);
			_GPSTIME_tow_ns.setNativeValue(msg.tow_ns);
		});

		_gps->setMessageCallback([this](SBP_DOPS_message msg)
		{
			_DOPS_tow_ms.setNativeValue(msg.tow_ms);
			_DOPS_gdop.setNativeValue(msg.gdop);
			_DOPS_pdop.setNativeValue(msg.pdop);
			_DOPS_tdop.setNativeValue(msg.tdop);
			_DOPS_hdop.setNativeValue(msg.hdop);
			_DOPS_vdop.setNativeValue(msg.vdop);
		});

		_gps->setMessageCallback([this](SBP_position_ECEF_message msg)
		{
			_POS_ECEF_tow_ms.setNativeValue(msg.tow_ms);
			_POS_ECEF_x.setNativeValue(msg.x);
			_POS_ECEF_y.setNativeValue(msg.y);
			_POS_ECEF_z.setNativeValue(msg.z);
			_POS_ECEF_accuracy.setNativeValue(msg.accuracy);
			_POS_ECEF_numsats.setNativeValue(msg.numSats);
			_POS_ECEF_fixmode.setNativeValue(msg.flags.fixMode);
		});

		_gps->setMessageCallback([this](SBP_position_LLH_message msg)
		{
			_POS_LLH_tow_ms.setNativeValue(msg.tow_ms);
			_POS_LLH_lat.setNativeValue(msg.lat);
			_POS_LLH_lon.setNativeValue(msg.lon);
			_POS_LLH_height.setNativeValue(msg.height);
			_POS_LLH_h_accuracy.setNativeValue(msg.h_accuracy);
			_POS_LLH_v_accuracy.setNativeValue(msg.v_accuracy);
			_POS_LLH_numsats.setNativeValue(msg.numSats);
			_POS_LLH_fixmode.setNativeValue(msg.flags.fixMode);
		});

		_gps->setMessageCallback([this](SBP_baseline_ECEF_message msg)
		{
			_BASE_ECEF_tow_ms.setNativeValue(msg.tow_ms);
			_BASE_ECEF_x.setNativeValue(msg.x);
			_BASE_ECEF_y.setNativeValue(msg.y);
			_BASE_ECEF_z.setNativeValue(msg.z);
			_BASE_ECEF_accuracy.setNativeValue(msg.accuracy);
			_BASE_ECEF_numsats.setNativeValue(msg.numSats);
			_BASE_ECEF_fixmode.setNativeValue(msg.flags.fixMode);
		});

		_gps->setMessageCallback([this](SBP_baseline_NED_message msg)
		{
			_BASE_NED_tow_ms.setNativeValue(msg.tow_ms);
			_BASE_NED_n.setNativeValue(msg.n);
			_BASE_NED_e.setNativeValue(msg.e);
			_BASE_NED_d.setNativeValue(msg.d);
			_BASE_NED_h_accuracy.setNativeValue(msg.h_accuracy);
			_BASE_NED_v_accuracy.setNativeValue(msg.v_accuracy);
			_BASE_NED_numsats.setNativeValue(msg.numSats);
			_BASE_NED_fixmode.setNativeValue(msg.flags.fixMode);
		});

		_gps->setMessageCallback([this](SBP_velocity_ECEF_message msg)
		{
			_VEL_ECEF_tow_ms.setNativeValue(msg.tow_ms);
			_VEL_ECEF_x.setNativeValue(msg.x);
			_VEL_ECEF_y.setNativeValue(msg.y);
			_VEL_ECEF_z.setNativeValue(msg.z);
			_VEL_ECEF_accuracy.setNativeValue(msg.accuracy);
			_VEL_ECEF_numsats.setNativeValue(msg.numSats);
		});

		_gps->setMessageCallback([this](SBP_velocity_NED_message msg)
		{
			_VEL_NED_tow_ms.setNativeValue(msg.tow_ms);
			_VEL_NED_n.setNativeValue(msg.n);
			_VEL_NED_e.setNativeValue(msg.e);
			_VEL_NED_d.setNativeValue(msg.d);
			_VEL_NED_h_accuracy.setNativeValue(msg.h_accuracy);
			_VEL_NED_v_accuracy.setNativeValue(msg.v_accuracy);
			_VEL_NED_numsats.setNativeValue(msg.numSats);
		});

		while (!_stopThread)
		{
			_gps->update();
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}

} /* namespace twirre */
