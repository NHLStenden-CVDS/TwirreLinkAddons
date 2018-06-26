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
 
#ifndef PIKSIGPS_H_
#define PIKSIGPS_H_

#include <queue>
#include <functional>
#include <Serial/SerialRW.h>

/* start of SBP message frame */
#define SBP_PREAMBLE 0x55

namespace piksigps
{
	/*
	 * Known Swiftnav Binary Protocol message types
	 */
	enum SBPmsgType
		: uint16_t
		{
			SBP_MSG_STARTUP = 0xFF00, SBP_MSG_HARTBEAT = 0xFFFF, SBP_MSG_GPSTIME = 0x0100,		// GPS TIME 11 GPS Time
		SBP_MSG_DOP = 0x0206,			// DOPS 14 Dilution of Precision
		SBP_MSG_POS_ECEF = 0x0200,		// POS ECEF 32 Position in ECEF
		SBP_MSG_POS_LLH = 0x0201,		// POS LLH 34 Geodetic Position
		SBP_MSG_BASELINE_ECEF = 0x0202,	// BASELINE ECEF 20 Baseline in ECEF
		SBP_MSG_BASELINE_NED = 0x0203,	// BASELINE NED 22 Baseline in NED
		SBP_MSG_VEL_ECEF = 0x0204,		// VEL ECEF 20 Velocity in ECEF
		SBP_MSG_VEL_NED = 0x0205		// VEL NED 22 Velocity in NED
	};

	/*
	 * Swiftnav Binary Protocol messages
	 */
#pragma pack(push,1)

	/*
	 * Raw Swiftnav Binary Protocol message (header + payload + crc)
	 */
	struct SBP_rawMessage
	{
		SBPmsgType msgType;
		uint16_t senderID;
		uint8_t payloadSize;
		uint8_t* payload = nullptr;
		uint16_t CRC;

		~SBP_rawMessage();
	};

//The system start-up message is sent once on system start-up. It is intended to be used to notify the host or other attached
//devices that the system has started and is now ready to respond to commands or configuration requests.
	struct SBP_startup_message
	{
		uint32_t __reserved;
	};

//The heartbeat message is sent periodically to inform the host or other attached devices that the system is running. It is
//intended to be used to monitor for system malfunctions and also contains status flags that indicate to the host the status of
//the system and if it is operating correctly.
	struct SBP_heartbeat_message
	{
		struct flags_t
		{
			uint32_t SystemError :1,			//0: system healthy			1: an error has occurred
					IOError :1, SwiftNAPError :1, __reserved :28, externalAntennaPresent :1;	//0: no external antenna	1: external antenna present
		} statusflags;	//Status flags
	};

//GPS Time.
	struct SBP_GPSTime_message
	{
		uint16_t weekNumber;	//GPS week number
		uint32_t tow_ms;		//GPS Time of Week rounded to the nearest ms
		int32_t tow_ns; 		//Nanosecond remainder of rounded tow
		uint8_t flags;			//Status flags (reserved)
	};

//Dilution of Precision.
	struct SBP_DOPS_message
	{
		uint32_t tow_ms;		//GPS Time of Week (ms)
		uint16_t gdop;			//Geometric Dilution of Precision (0.01)
		uint16_t pdop;			//Position Dilution of Precision (0.01)
		uint16_t tdop;			//Time Dilution of Precision (0.01)
		uint16_t hdop;			//Horizontal Dilution of Precision (0.01)
		uint16_t vdop;			//Vertical Dilution of Precision (0.01)
	};

//Position solution in absolute Earth Centered Earth Fixed (ECEF) coordinates.
	struct SBP_position_ECEF_message
	{
		uint32_t tow_ms;		//GPS Time of Week (ms)
		double x;				//ECEF X coordinate (m)
		double y;				//ECEF Y coordinate (m)
		double z;				//ECEF Z coordinate (m)
		uint16_t accuracy;		//Position accuracy estimate (mm)
		uint8_t numSats;		//number of satellites used in solution
		struct flags_t
		{
			uint8_t fixMode :3,	//0: SPP	1: RTK
					__reserved :5;
		} flags;
	};

//Geodetic position solution.
	struct SBP_position_LLH_message
	{
		uint32_t tow_ms;		//GPS Time of Week (ms)
		double lat;				//Latitude (deg)
		double lon;				//Longitude (deg)
		double height;			//Altitude (m)
		uint16_t h_accuracy;	//Horizontal accuracy estimate (mm)
		uint16_t v_accuracy;	//Vertical accuracy estimate (mm)
		uint8_t numSats;		//number of satellites used in solution
		struct flags_t
		{
			uint8_t fixMode :3,	//0: SPP	1: RTK
					heightMode :1,	//0: Height above ellipsoid		1: Height above average sea level
					__reserved :4;
		} flags;
	};

//Baseline in Earth Centered Earth Fixed (ECEF) coordinates.
	struct SBP_baseline_ECEF_message
	{
		uint32_t tow_ms;		//GPS Time of Week (ms)
		int32_t x;				//Baseline ECEF X coordinate (mm)
		int32_t y;				//Baseline ECEF Y coordinate (mm)
		int32_t z;				//Baseline ECEF Z coordinate (mm)
		uint16_t accuracy;		//Position accuracy estimate (mm)
		uint8_t numSats;		//number of satellites used in solution
		struct flags_t
		{
			uint8_t fixMode :3,	//0: Float	1: Fixed RTK
					__reserved :5;
		} flags;
	};

//Baseline in local North East Down (NED) coordinates.
	struct SBP_baseline_NED_message
	{
		uint32_t tow_ms;		//GPS Time of Week (ms)
		int32_t n;				//Baseline North coordinate (mm)
		int32_t e;				//Baseline East coordinate (mm)
		int32_t d;				//Baseline Down coordinate (mm)
		uint16_t h_accuracy;	//Horizontal accuracy estimate (mm)
		uint16_t v_accuracy;	//Vertical accuracy estimate (mm)
		uint8_t numSats;		//number of satellites used in solution
		struct flags_t
		{
			uint8_t fixMode :3,	//0: Float	1: Fixed RTK
					__reserved :5;
		} flags;
	};

//Velocity in Earth Centered Earth Fixed (ECEF) coordinates.
	struct SBP_velocity_ECEF_message
	{
		uint32_t tow_ms;		//GPS Time of Week (ms)
		int32_t x;				//Velocity ECEF X coordinate
		int32_t y;				//Velocity ECEF Y coordinate
		int32_t z;				//Velocity ECEF Z coordinate
		uint16_t accuracy;		//Velocity accuracy estimate
		uint8_t numSats;		//number of satellites used in solution
		uint8_t flags;			//Status flags (reserved)
	};

//Velocity in local North East Down (NED) coordinates.
	struct SBP_velocity_NED_message
	{
		uint32_t tow_ms;		//GPS Time of Week (ms)
		int32_t n;				//Velocity North coordinate
		int32_t e;				//Velocity East coordinate
		int32_t d;				//Velocity Down coordinate
		uint16_t h_accuracy;	//Horizontal accuracy estimate
		uint16_t v_accuracy;	//Vertical accuracy estimate
		uint8_t numSats;		//number of satellites used in solution
		uint8_t flags;			//Status flags (reserved)
	};

	/* message processing callbacks */
	typedef std::function<void(SBP_startup_message)> SBP_StartupCallback;
	typedef std::function<void(SBP_heartbeat_message)> SBP_HeartbeatCallback;
	typedef std::function<void(SBP_GPSTime_message)> SBP_GPSTimeCallback;
	typedef std::function<void(SBP_DOPS_message)> SBP_DOPSCallback;
	typedef std::function<void(SBP_position_ECEF_message)> SBP_positionECEFCallback;
	typedef std::function<void(SBP_position_LLH_message)> SBP_positionLLHCallback;
	typedef std::function<void(SBP_baseline_ECEF_message)> SBP_baselineECEFCallback;
	typedef std::function<void(SBP_baseline_NED_message)> SBP_baselineNEDCallback;
	typedef std::function<void(SBP_velocity_ECEF_message)> SBP_velocityECEFCallback;
	typedef std::function<void(SBP_velocity_NED_message)> SBP_velocityNEDCallback;

#pragma pack(pop)

	class PiksiManager
	{
	protected:

		/* shared vars */
		std::queue<SBP_rawMessage*> _msgBuf;
		std::mutex _msgBufMutex;
		bool _rx_thread_run;
		uint64_t _messages_RX_ok;
		uint64_t _messages_RX_CRCfail;

		/* host local vars */
		std::thread* _rxThread = nullptr;
		SBP_StartupCallback _startupCB = nullptr;
		SBP_HeartbeatCallback _heartbeatCB = nullptr;
		SBP_GPSTimeCallback _gpsTimeCB = nullptr;
		SBP_DOPSCallback _dopsCB = nullptr;
		SBP_positionECEFCallback _pos_ecefCB = nullptr;
		SBP_positionLLHCallback _pos_llhCB = nullptr;
		SBP_baselineECEFCallback _base_ecefCB = nullptr;
		SBP_baselineNEDCallback _base_nedCB = nullptr;
		SBP_velocityECEFCallback _vel_ecefCB = nullptr;
		SBP_velocityNEDCallback _vel_nedCB = nullptr;

		/* thread local vars */
		SerialRW* _rxSerial = nullptr;

	public:
		PiksiManager(const char* const port);
		~PiksiManager();

		/* register startup message callback. Set to nullptr to disable this callback. */
		void setMessageCallback(SBP_StartupCallback cb);
		/* register heartbeat message callback. Set to nullptr to disable this callback. */
		void setMessageCallback(SBP_HeartbeatCallback cb);
		/* register DOPS message callback. Set to nullptr to disable this callback. */
		void setMessageCallback(SBP_DOPSCallback cb);
		/* register GPS time message callback. Set to nullptr to disable this callback. */
		void setMessageCallback(SBP_GPSTimeCallback cb);
		/* register position (ECEF) message callback. Set to nullptr to disable this callback. */
		void setMessageCallback(SBP_positionECEFCallback cb);
		/* register position (LLH) callback. Set to nullptr to disable this callback. */
		void setMessageCallback(SBP_positionLLHCallback cb);
		/* register baseline (ECEF) callback. Set to nullptr to disable this callback. */
		void setMessageCallback(SBP_baselineECEFCallback cb);
		/* register baseline (NED) callback. Set to nullptr to disable this callback. */
		void setMessageCallback(SBP_baselineNEDCallback cb);
		/* register velocity (ECEF) callback. Set to nullptr to disable this callback. */
		void setMessageCallback(SBP_velocityECEFCallback cb);
		/* register velocity (NED) callback. Set to nullptr to disable this callback. */
		void setMessageCallback(SBP_velocityNEDCallback cb);

		/** Poll for messages, send them to the registered callbacks
		 * @returns raw message CRC success ratio */
		float update();

	protected:
		void rx_thread_main();
		void handleRawMessage(SBP_rawMessage* msg);
	};
}
#endif /* PIKSIGPS_H_ */
