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

#ifndef PRECIS_SENSOR_H_
#define PRECIS_SENSOR_H_

#include <thread>

#include <Core/Sensor.h>
#include <Core/Actuator.h>
#include <Core/Value.h>

namespace precisgps
{
	class precisgps;
}

namespace twirre
{
	class PrecisSensor: public Sensor, public Actuator
	{
	public:
		PrecisSensor(std::string name, const std::string & path);
		virtual ~PrecisSensor();

		virtual void onSense(const std::vector<std::string> &names) override;
		virtual void ActuateImpl() override;

	protected:
		precisgps::precisgps * _gps;
		ValueImpl<uint8_t> _fixtime_h { "gpstime_h", 0 };
		ValueImpl<uint8_t> _fixtime_m { "gpstime_m", 0 };
		ValueImpl<uint8_t> _fixtime_s { "gpstime_s", 0 };
		ValueImpl<uint32_t> _fixtime_us { "gpstime_us", 0 };
		ValueImpl<double> _latitude { "latitude", 0 };
		ValueImpl<double> _longitude { "longitude", 0 };
		ValueImpl<uint8_t> _gpsmode { "gpsmode", 0 };
		ValueImpl<uint16_t> _numsats { "satellitesInView", 0 };
		ValueImpl<float> _hdops { "HDOPS", 999 };
		ValueImpl<double> _altitude { "altitude", 0 };
		ValueImpl<double> _geoidalSeparation { "geoidalSeparation", 0 };
		ValueImpl<double> _differentialAge { "differentialAge", 0};
		ValueImpl<uint16_t> _differentialId { "differentialID", 0};

		ArrayValue<uint8_t> _satConstellations { "satConstellations" };
		ArrayValue<int16_t> _satNumbers { "satNumbers" };
		ArrayValue<int16_t> _satElevations { "satElevations" };
		ArrayValue<int16_t> _satAzimuths { "satAzimuths" };
		ArrayValue<int16_t> _satSNRs { "satSNRs" };

		//params
		ValueImpl<uint8_t> _debugOutput { "debugoutput", 0 };
	};

} /* namespace twirre */

#endif /* GPS_SENSOR_H_ */
