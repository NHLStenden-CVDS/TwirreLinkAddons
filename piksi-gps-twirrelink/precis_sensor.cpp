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

#include "precis_sensor.h"
#include "precisgps.h"

namespace twirre
{
	PrecisSensor::PrecisSensor(std::string name, const std::string & path) :
			Sensor(name, "Tersus Precis RTK GPS"),
			Actuator(name, "Tersus Precis RTK GPS")
	{
		registerValues({&_fixtime_h, &_fixtime_m, &_fixtime_s, &_fixtime_us, &_latitude, &_longitude, &_gpsmode, &_numsats, &_hdops, &_altitude, &_geoidalSeparation, &_differentialAge, &_differentialId});
		registerValues({&_satNumbers, &_satConstellations, &_satAzimuths, &_satElevations, &_satSNRs});

		registerParameter(&_debugOutput);

		_gps = new precisgps::precisgps(path);
	}

	PrecisSensor::~PrecisSensor()
	{
		delete _gps;
	}

	void PrecisSensor::onSense(const std::vector<std::string> &names)
	{
		auto fixinfo = _gps->getFixInformation();
		_fixtime_h.setNativeValue(fixinfo.time.hours);
		_fixtime_m.setNativeValue(fixinfo.time.minutes);
		_fixtime_s.setNativeValue(fixinfo.time.seconds);
		_fixtime_us.setNativeValue(fixinfo.time.microseconds);
		_latitude.setNativeValue(fixinfo.latitude);
		_longitude.setNativeValue(fixinfo.longitude);
		_gpsmode.setNativeValue(fixinfo.gpsMode);
		_numsats.setNativeValue(fixinfo.satellitesInView);
		_hdops.setNativeValue(fixinfo.HDOPS);
		_altitude.setNativeValue(fixinfo.altitude_amsl);
		_geoidalSeparation.setNativeValue(fixinfo.geoidalSeparation);
		_differentialAge.setNativeValue(fixinfo.differential_age);
		_differentialId.setNativeValue(fixinfo.differential_id);

		auto satinfo = _gps->getCombinedSatInfo();
		int numsatinfos = satinfo.size();
		_satNumbers.setSize(numsatinfos);
		_satConstellations.setSize(numsatinfos);
		_satElevations.setSize(numsatinfos);
		_satAzimuths.setSize(numsatinfos);
		_satSNRs.setSize(numsatinfos);
		auto * satNumBuf = _satNumbers.getNativeBuffer();
		auto * satConstBuf = _satConstellations.getNativeBuffer();
		auto * satElevBuf = _satElevations.getNativeBuffer();
		auto * satAziBuf = _satAzimuths.getNativeBuffer();
		auto * satSNRBuf = _satSNRs.getNativeBuffer();

		for(int i = 0; i < numsatinfos; i++)
		{
			precisgps::nmea_satelliteinformation currentSat = satinfo[i];
			satNumBuf[i] = currentSat.number;
			satConstBuf[i] = currentSat.constellation;
			satElevBuf[i] = currentSat.elevation;
			satAziBuf[i] = currentSat.azimuth;
			satSNRBuf[i] = currentSat.SNR;
		}
	}

	void PrecisSensor::ActuateImpl()
	{
		_gps->setDebugOutput(_debugOutput.getNativeValue() == 1);
	}

} /* namespace twirre */
