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

#include <string>
#include <iostream>
#include <vector>
#include <cmath>
#include <mutex>

#include "precisgps.h"

using namespace std;

namespace precisgps
{

	precisgps::precisgps(const std::string & devicePath)
	{
		_rxSerial = new SerialRW();
		_rxSerial->Initialize(devicePath.c_str(), 115200);

		_rxThreadRun = true;
		_rxThread = new std::thread(&precisgps::rxThreadMain, this);
	}

	precisgps::~precisgps()
	{
		_rxThreadRun = false;
		_rxThread->join();

	}

	nmea_fixinformation precisgps::getFixInformation()
	{
		lock_guard<mutex> gpsDataLock(_gpsDataMutex);
		nmea_fixinformation result = _fixInfo;
		return result;
	}

	std::vector<nmea_satelliteinformation> precisgps::getGPSSatInfo()
	{
		lock_guard<mutex> gpsDataLock(_gpsDataMutex);
		return _GPSSatInfo;
	}
	std::vector<nmea_satelliteinformation> precisgps::getGlonassSatInfo()
	{
		lock_guard<mutex> gpsDataLock(_gpsDataMutex);
		return _GlonassSatInfo;
	}
	std::vector<nmea_satelliteinformation> precisgps::getBeidouSatInfo()
	{
		lock_guard<mutex> gpsDataLock(_gpsDataMutex);
		return _BeidouSatInfo;
	}
	std::vector<nmea_satelliteinformation> precisgps::getCombinedSatInfo()
	{
		std::vector<nmea_satelliteinformation> returnVec;

		lock_guard<mutex> gpsDataLock(_gpsDataMutex);

		for(const auto & sat : _GPSSatInfo)
		{
			returnVec.push_back(sat);
		}
		for(const auto & sat : _GlonassSatInfo)
		{
			returnVec.push_back(sat);
		}
		for(const auto & sat : _BeidouSatInfo)
		{
			returnVec.push_back(sat);
		}

		return returnVec;
	}
	void precisgps::setDebugOutput(bool enable)
	{
		_dbgOut = enable;
	}

	vector<string> splitString(const string & str, const char delim)
	{
		vector<string> result;
		size_t loc, pos = 0; //loc: location of delimiter, pos: position of current string element
		while((loc = str.find(delim,pos)) != string::npos)
		{
			result.push_back(str.substr(pos, loc - pos)); //store current element
			pos = loc + 1;
		}
		if(pos < str.size())
		{
			result.push_back(str.substr(pos, str.size())); //store last string element
		}
		else
		{
			result.push_back(""); //last char in string was delimiter, so push an additional empty element
		}
		return result;
	}

	void precisgps::rxThreadMain()
	{
		//todo: debug
		std::cout << "rx thread start" << std::endl;

		while(_rxThreadRun)
		{
			try
			{
				/* sync NMEA start */
				char rxChar = _rxSerial->Read<char>();
				while(rxChar != '$')
				{
					rxChar = _rxSerial->Read<char>();
				}

				/* read rest of NMEA string */
				std::string nmea;
				_rxSerial->readString(nmea, StringTerminator::LF);

				//strip carriage return
				nmea = nmea.substr(0, nmea.size() - 1);

				//todo:debug
				if(_dbgOut)
				{
					std::cout << "raw nmea recv: " << nmea << std::endl;
				}

				//scan for checksum
				auto checksumPos = nmea.find('*');
				if(checksumPos != std::string::npos)
				{
					//check if two characters remain after checksum indicator
					if(checksumPos != nmea.size() - 3)
					{
						std::cerr << "nmea string has incorrect checksum specification (should end with '*xx', but '*' was found in wrong position): " << nmea << std::endl;
						continue;
					}

					//read checksum
					int msgChecksum = std::stoi(nmea.substr(checksumPos + 1, 2), 0, 16); //parses last two characters of nmea string as hex

					//strip checksum part from string
					nmea = nmea.substr(0, nmea.size() - 3);

					//validate checksum
					int rxChecksum = 0;
					for(char c : nmea)
					{
						rxChecksum ^= c;
					}
					if(msgChecksum != rxChecksum)
					{
						std::cerr << "nmea checksum failed, should be " << msgChecksum << " but is " << rxChecksum << " for message " << nmea << std::endl;
					}
				}

				//split NMEA string in subparts
				auto stringParts = splitString(nmea,',');

				if(stringParts.size() < 1)
				{
					std::cerr << "empty nmea string" << nmea << std::endl;
				}

				//get talker ID and message ID
				string msgID = stringParts[0];
				if(msgID.size() != 5)
				{
					std::cerr << "invalid message ID: " << msgID << " in " << nmea << std::endl;
				}

				string talker = stringParts[0].substr(0,2);
				string type = stringParts[0].substr(2,3);

				//process NMEA data
				handleNMEA(talker, type, stringParts);
			}
			catch(exception &e)
			{
				std::cerr << "caught exception in NMEA read loop: " << e.what() << std::endl;
			}
			catch(...)
			{
				std::cerr << "caught exception in NMEA read loop with unknown type" << std::endl;
			}
		}
	}

	void precisgps::handleNMEA(const std::string & talker, const std::string & type, const std::vector<std::string> & parts )
	{
		if(type.compare("GGA") == 0)
		{
			handleGGA(talker, parts);
			return;
		}
		if(type.compare("GSV") == 0)
		{
			handleGSV(talker, parts);
			return;
		}

		//no match
		std::cerr << "unrecognized nmea message type: " << type << std::endl;
	}

	inline int trystoi(const string & str, int def = 0)
	{
		try
		{
			return stoi(str);
		}
		catch(...)
		{
			//ignore
		}
		return def;
	}

	inline double trystod(const string & str, double def = 0)
	{
		try
		{
			return stod(str);
		}
		catch(...)
		{
			//ignore
		}
		return def;
	}

	static utctime decodeTimeString(const string & str)
	{
		if(str.size() < 6)
		{
			return {0,0,0,0};
		}

		uint8_t hours = trystoi(str.substr(0,2));
		uint8_t minutes = trystoi(str.substr(2,2));
		uint8_t seconds = trystoi(str.substr(4,2));
		uint32_t micros;

		if(str.size() > 7)
		{
			micros = trystoi(str.substr(7,str.length())) * pow(10,13 - str.size());
		}
		else
		{
			micros = 0;
		}

		return {hours, minutes, seconds, micros};
	}

	static double decodeDegrees(const string & degStr, int minuteStart, const string & hemi)
	{
		if(degStr.size() <= minuteStart) return 0;

		int deg_int = trystoi(degStr.substr(0, minuteStart));
		double mins = trystod(degStr.substr(minuteStart, string::npos));
		double degs = ((double) deg_int) + (mins / 60.0);
		if((hemi.compare("S") == 0) || (hemi.compare("W") == 0))
		{
			degs = -degs;
		}
		return degs;
	}



	void precisgps::handleGGA(const std::string & talker, const std::vector<std::string> & parts)
	{
		nmea_fixinformation newInfo;

		if(parts.size() < 15)
		{
			return;
		}

		newInfo.time = decodeTimeString(parts[1]);
		newInfo.latitude = decodeDegrees(parts[2], 2, parts[3]);
		newInfo.longitude = decodeDegrees(parts[4], 3, parts[5]);
		newInfo.gpsMode = (GPSQuality)trystoi(parts[6],-1);
		newInfo.satellitesInView = trystoi(parts[7]);
		newInfo.HDOPS = trystod(parts[8],999);
		newInfo.altitude_amsl = trystod(parts[9]);
		newInfo.geoidalSeparation = trystod(parts[11]);	//note: element 10 skipped because it indicates unit of altitude, which is always meters
		newInfo.differential_age = trystod(parts[13]);		//note: element 12 skipped because it indicates unit of geoidal separation, which is always meters
		newInfo.differential_id = trystoi(parts[14]);

		//update info
		{
			lock_guard<mutex> gpsDataLock(_gpsDataMutex);
			_fixInfo = newInfo;
		}

		//debug output
		if(_dbgOut)
		{
			std::cout << "updated fix info: " << endl;
			cout << "\ttime: " << (int)newInfo.time.hours << "h" << (int)newInfo.time.minutes << "m" << (int)newInfo.time.seconds << "s" << newInfo.time.microseconds << "us" << endl;
			cout << "\tlat: " << newInfo.latitude << endl;
			cout << "\tlon: " << newInfo.longitude << endl;
			cout << "\tgpsmode: " << newInfo.gpsMode << endl;
			cout << "\tsats: " << newInfo.satellitesInView << endl;
			cout << "\tHDOPS: " << newInfo.HDOPS << endl;
			cout << "\taltitude: " << newInfo.altitude_amsl << endl;
			cout << "\tgeoidalSeparation: " << newInfo.geoidalSeparation << endl;
			cout << "\tdifferential_age: " << newInfo.differential_age << endl;
			cout << "\tdifferential_id: " << newInfo.differential_id << endl;
		}
	}

	void precisgps::handleGSV(const std::string & talker, const std::vector<std::string> & parts)
	{
		//check constellation
		if(talker.compare("GP") == 0)
		{
			handleGSV_constellation(parts, satellite_constellation::gps, _gpsInfoCount, _buildGPSSatInfo, _GPSSatInfo);
		}
		else if(talker.compare("GL") == 0)
		{
			handleGSV_constellation(parts, satellite_constellation::glonass, _glonassInfoCount, _buildGlonassSatInfo, _GlonassSatInfo);
		}
		else if((talker.compare("GB") == 0) || (talker.compare("BD") == 0))
		{
			handleGSV_constellation(parts, satellite_constellation::beidou, _beidouInfoCount, _buildBeidouSatInfo, _BeidouSatInfo);
		}
		else if(talker.compare("GA") == 0)
		{
			return; //not supported yet
			//currentConstellation = satellite_constellation::galileo;
		}
		else
		{
			return; //unsupported talker
		}
	}

	void precisgps::handleGSV_constellation(const std::vector<std::string> & parts, satellite_constellation constellation, int & msgcounter, std::vector<nmea_satelliteinformation> & satInfosBuild, std::vector<nmea_satelliteinformation> & satInfosFinal)
	{
		int totalMessages = trystoi(parts[1]);
		int currentMessageNum = trystoi(parts[2]);

		//check for start of new sequence
		if(currentMessageNum == 1)
		{
			//reset msg counter
			msgcounter = 1;

			//clear the build list
			satInfosBuild.clear();
		}

		//check if the current message number is expected
		if(currentMessageNum == msgcounter)
		{
			msgcounter++;
		}
		else
		{
			//missed a message, abort
			return;
		}

		//decode satellites
		int numSats = (parts.size() - 3) / 4;
		for(int i = 0; i < numSats; i++)
		{
			nmea_satelliteinformation currentInfo;
			currentInfo.constellation = constellation;
			currentInfo.number = trystoi(parts[4 + i * 4]);
			currentInfo.elevation = trystoi(parts[5 + i * 4], -1);
			currentInfo.azimuth = trystoi(parts[6 + i * 4], -1);
			currentInfo.SNR = trystoi(parts[7 + i * 4]);

			satInfosBuild.push_back(currentInfo);
		}

		//if this was the last message, swap info buffers
		if(currentMessageNum == totalMessages)
		{
			{
				std::lock_guard<std::mutex> dataLock(_gpsDataMutex);

				std::swap(satInfosBuild, satInfosFinal);
			}

			if(_dbgOut)
			{
				cout << "new satellite info for constellation " << static_cast<int>(constellation) << endl;
				for(const auto & sat : satInfosFinal)
				{
					cout << "\t" << "satellite " << sat.number << ":" << endl;
					cout << "\t\t" << "elevation: " << sat.elevation << endl;
					cout << "\t\t" << "azimuth  : " << sat.azimuth << endl;
					cout << "\t\t" << "SNR      : " << sat.SNR << endl;
				}
			}
		}
	}


}
