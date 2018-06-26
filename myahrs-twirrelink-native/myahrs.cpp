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

#include "myahrs.h"

namespace twirre
{
	MyAHRS::MyAHRS(std::string name, const char * path) : Sensor(name, "MyAHRS+ IMU")
	{
		registerValues( { &_whoami, &_revMajor, &_revMinor, &_status });

		registerValues( { &_accX, &_accY, &_accZ });
		registerValues( { &_gyroX, &_gyroY, &_gyroZ });
		registerValues( { &_magX, &_magY, &_magZ });
		registerValue(&_temp);
		registerValues( { &_pitch, &_yaw, &_roll });
		registerValues( { &_quaternionX, &_quaternionY, &_quaternionZ, &_quaternionW });

		if(! _sensor.start(path, MYAHRS_BAUDRATE))
		{
			throw std::runtime_error("unable to open sensor");
		}

		/*
		 *  set binary output format
		 *   - select Quaternion and IMU data
		 */
		if (_sensor.cmd_binary_data_format("QUATERNION, IMU") == false)
		{
			throw std::runtime_error("myAHRS_init: cmd_binary_data_format() returns false");
		}

		/*
		 *  set divider
		 *   - output rate(Hz) = max_rate/divider
		 */
		if (_sensor.cmd_divider("1") == false)
		{
			throw std::runtime_error("myAHRS_init: cmd_divider() returns false");
		}

		/*
		 *  set transfer mode
		 *   - BC : Binary Message & Continuous mode
		 */
		if (_sensor.cmd_mode("BC") == false)
		{
			throw std::runtime_error("myAHRS_init: cmd_mode() returns false");
		}

		_stopThread = false;
		_updateThread = std::thread(&MyAHRS::thread_main, this);
	}

	MyAHRS::~MyAHRS()
	{
		_stopThread = true;
		_updateThread.join();
	}

	void MyAHRS::thread_main()
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		WithRobot::SensorData sensor_data;
		while (!_stopThread)
		{
			if (_sensor.wait_data() == true)
			{ // waiting for new data
				/*
				 * copy sensor data
				 */
				_sensor.get_data(sensor_data);

				/*
				 * print quaternion & imu data
				 */
				WithRobot::Quaternion& q = sensor_data.quaternion;
				WithRobot::ImuData<float>& imu = sensor_data.imu;
				WithRobot::EulerAngle e = q.to_euler_angle();

				//set IMU data
				_accX.setNativeValue(imu.ax);
				_accY.setNativeValue(imu.ay);
				_accZ.setNativeValue(imu.az);

				_gyroX.setNativeValue(imu.gx);
				_gyroY.setNativeValue(imu.gy);
				_gyroZ.setNativeValue(imu.gz);

				_magX.setNativeValue(imu.mx);
				_magY.setNativeValue(imu.my);
				_magZ.setNativeValue(imu.mz);

				_temp.setNativeValue(imu.temperature);

				//set quaternion data
				_quaternionX.setNativeValue(q.x);
				_quaternionY.setNativeValue(q.y);
				_quaternionZ.setNativeValue(q.z);
				_quaternionW.setNativeValue(q.w);

				//set RPY data
				_pitch.setNativeValue(e.pitch);
				_yaw.setNativeValue(e.yaw);
				_roll.setNativeValue(e.roll);
			}
		}
	}

} /* namespace twirre */
