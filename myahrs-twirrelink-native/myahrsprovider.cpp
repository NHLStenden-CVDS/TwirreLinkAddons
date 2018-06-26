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

#include "myahrsprovider.h"
#include "myahrs.h"

#include "FakeAHRS.h"

namespace twirre
{

	MyAHRSProvider::MyAHRSProvider()
	{
		// TODO Auto-generated constructor stub

	}

	MyAHRSProvider::~MyAHRSProvider()
	{
		// TODO Auto-generated destructor stub
	}

	void MyAHRSProvider::addAHRS(std::string name, const char * path)
	{
		_sensors[name] = new MyAHRS(name, path);

		doNotifyChange();
	}

	void MyAHRSProvider::addFakeAHRS(std::string name)
	{
		_sensors[name] = new FakeAHRS(name);

		doNotifyChange();
	}

	const std::map<std::string, Actuator*> & MyAHRSProvider::getActuators()
	{
		return _actuators;
	}
	const std::map<std::string, Sensor*> & MyAHRSProvider::getSensors()
	{
		return _sensors;
	}

}
/* namespace twirrelink */
