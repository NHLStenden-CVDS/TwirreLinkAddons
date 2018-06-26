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

#ifndef MYAHRS_H_
#define MYAHRS_H_

#include <thread>

#include <Core/Sensor.h>
#include <Core/Value.h>

#include "myAHRS_SDK/myahrs_plus.hpp"


namespace twirre
{
	static const int MYAHRS_BAUDRATE = 115200;


	class MyAHRS: public Sensor
	{
	public:
		MyAHRS(std::string name, const char * path);
		virtual ~MyAHRS();

	protected:
		WithRobot::MyAhrsPlus _sensor;

		bool _stopThread = true;
		std::thread _updateThread;
		void thread_main();


		ValueImpl<uint8_t> _whoami {"whoami", 0};
		ValueImpl<uint8_t> _revMajor {"rev_major", 0};
		ValueImpl<uint8_t> _revMinor {"rev_minor", 0};
		ValueImpl<uint8_t> _status {"status", 0};

		ValueImpl<float> _accX {"accX", 0};
		ValueImpl<float> _accY {"accY", 0};
		ValueImpl<float> _accZ {"accZ", 0};

		ValueImpl<float> _gyroX {"gyroX", 0};
		ValueImpl<float> _gyroY {"gyroY", 0};
		ValueImpl<float> _gyroZ {"gyroZ", 0};

		ValueImpl<float> _magX {"magX", 0};
		ValueImpl<float> _magY {"magY", 0};
		ValueImpl<float> _magZ {"magZ", 0};

		ValueImpl<float> _temp {"temp", 0};

		ValueImpl<float> _roll {"roll", 0};
		ValueImpl<float> _pitch {"pitch", 0};
		ValueImpl<float> _yaw {"yaw", 0};

		ValueImpl<double> _quaternionX {"quaternionX", 0};
		ValueImpl<double> _quaternionY {"quaternionY", 0};
		ValueImpl<double> _quaternionZ {"quaternionZ", 0};
		ValueImpl<double> _quaternionW {"quaternionW", 0};
	};

} /* namespace twirre */

#endif /* MYAHRS_H_ */
