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

#include <iostream>
#include <cstdio>
#include <thread>
#include <mutex>
#include <vector>
#include <queue>
#include <cstdlib>
#include <pthread.h>

#include <Serial/SerialRW.h>
#include "piksigps.h"
#include "CRC16_xmodem.h"

namespace piksigps
{
	SBP_rawMessage::~SBP_rawMessage()
	{
		if (payload != nullptr) std::free(payload);
	}

	/**
	 * Set FIFO scheduling for a single thread
	 * @param thread
	 * This thread will be set to FIFO scheduling
	 * @param priority
	 * Scheduling priority to apply
	 */
	static void set_sched_fifo(std::thread::native_handle_type thread, int priority)
	{
		int ret;

		// struct sched_param is used to store the scheduling priority
		struct sched_param params;
		// We'll set the priority to the maximum.
		params.sched_priority = sched_get_priority_max(SCHED_FIFO);

		// Attempt to set thread real-time priority to the SCHED_FIFO policy
		ret = pthread_setschedparam(thread, SCHED_FIFO, &params);
		if (ret != 0)
		{
			// Print the error
			std::cerr << "Unsuccessful in setting thread realtime prio" << std::endl;
			return;
		}
	}

	/**
	 * Construct a new PiksiManager
	 * @param dev
	 * Path to the serial port to be used
	 */
	PiksiManager::PiksiManager(const char* const dev)
	{
		_messages_RX_ok = 0;
		_messages_RX_CRCfail = 0;

		_rx_thread_run = true;
		_rxSerial = new SerialRW();
		_rxSerial->Initialize(dev, 500000);
		_rxThread = new std::thread(&PiksiManager::rx_thread_main, this);

		//really need to set high-priority FIFO scheduling for our RX thread, otherwise it will miss bytes when system load is fairly high
		set_sched_fifo(_rxThread->native_handle(), 20);
	}

	PiksiManager::~PiksiManager()
	{
		_rx_thread_run = false;
		_rxThread->join();
		delete _rxSerial;
		delete _rxThread;
	}

	/**
	 * Main() function for the serial port receiver thread
	 */
	void PiksiManager::rx_thread_main()
	{
		//known piksi serial#
		int32_t _knownID = -1;

		//continue looping until a stop is requested
		while (_rx_thread_run)
		{
			//create FD sets for select() call
			fd_set readSet;
			FD_ZERO(&readSet);
			FD_SET(_rxSerial->_fd, &readSet);

			//create timeout
			timeval readTimeout;
			readTimeout.tv_sec = 5;
			readTimeout.tv_usec = 0;

			//wait until data is available
			int nfds = select(_rxSerial->_fd + 1, &readSet, nullptr, nullptr, &readTimeout);

			//check for timeout/error condition
			if (nfds < 1)
			{
				//timeout: stop the thread (serial connection broken
				_rx_thread_run = false;
				std::cerr << "select() timeout" << std::endl;
				break;
			}

			//scan for framing preamble
			unsigned char input = _rxSerial->Read<unsigned char>();
			if (input == SBP_PREAMBLE)
			{
				//raw message struct
				SBP_rawMessage* msg = new SBP_rawMessage;

				//frame start: read next 5 bytes for type, id and payloadSize
				_rxSerial->readNBytes((unsigned char*) &msg->msgType, 2);
				_rxSerial->readNBytes((unsigned char*) &msg->senderID, 2);

				//check if sender ID matches known previous senderID
				if(_knownID >= 0 && msg->senderID != _knownID)
				{
					//ID mismatch: probably spurious preamble
					continue;
				}

				msg->payloadSize = _rxSerial->Read<unsigned char>();

				//read payload and CRC
				msg->payload = (uint8_t*) std::malloc(msg->payloadSize * sizeof(unsigned char));
				_rxSerial->readNBytes(msg->payload, msg->payloadSize);
				_rxSerial->readNBytes((unsigned char*) &msg->CRC, 2);

				//calculate CRC of received message
				uint16_t crc = crc16_ccitt(reinterpret_cast<uint8_t*>(msg), 5, 0);
				crc = crc16_ccitt(reinterpret_cast<uint8_t*>(msg->payload), msg->payloadSize, crc);

				//check if calculated CRC matches received CRC
				if (crc == msg->CRC)
				{
					//push the final message to the buffer
					_msgBufMutex.lock();
					_msgBuf.push(msg);
					_msgBufMutex.unlock();

					_messages_RX_ok++;

					//store known ID
					if(_knownID < 0)
					{
						_knownID = msg->senderID;
					}
				}
				else
				{
					//crc mismatch: print error, drop the message
					//std::cerr << "crc failed" << std::endl;
					_messages_RX_CRCfail++;
				}
			}
		}

		std::cout << "Piksi RX thread terminating." << std::endl;
	}

	/**
	 *
	 * @param msg
	 */
	void PiksiManager::handleRawMessage(SBP_rawMessage* msg)
	{
		/* dispatch message to correct callback (when set) */
		switch (msg->msgType)
		{
			case SBP_MSG_STARTUP:
				if (_startupCB != nullptr) _startupCB(*(reinterpret_cast<SBP_startup_message*>(msg->payload)));
				break;
			case SBP_MSG_HARTBEAT:
				if (_heartbeatCB != nullptr) _heartbeatCB(*(reinterpret_cast<SBP_heartbeat_message*>(msg->payload)));
				break;
			case SBP_MSG_DOP:
				if (_dopsCB != nullptr) _dopsCB(*(reinterpret_cast<SBP_DOPS_message*>(msg->payload)));
				break;
			case SBP_MSG_GPSTIME:
				if (_gpsTimeCB != nullptr) _gpsTimeCB(*(reinterpret_cast<SBP_GPSTime_message*>(msg->payload)));
				break;
			case SBP_MSG_POS_ECEF:
				if (_pos_ecefCB != nullptr) _pos_ecefCB(*(reinterpret_cast<SBP_position_ECEF_message*>(msg->payload)));
				break;
			case SBP_MSG_POS_LLH:
				if (_pos_llhCB != nullptr) _pos_llhCB(*(reinterpret_cast<SBP_position_LLH_message*>(msg->payload)));
				break;
			case SBP_MSG_BASELINE_ECEF:
				if (_base_ecefCB != nullptr) _base_ecefCB(*(reinterpret_cast<SBP_baseline_ECEF_message*>(msg->payload)));
				break;
			case SBP_MSG_BASELINE_NED:
				if (_base_nedCB != nullptr) _base_nedCB(*(reinterpret_cast<SBP_baseline_NED_message*>(msg->payload)));
				break;
			case SBP_MSG_VEL_ECEF:
				if (_vel_ecefCB != nullptr) _vel_ecefCB(*(reinterpret_cast<SBP_velocity_ECEF_message*>(msg->payload)));
				break;
			case SBP_MSG_VEL_NED:
				if (_vel_nedCB != nullptr) _vel_nedCB(*(reinterpret_cast<SBP_velocity_NED_message*>(msg->payload)));
				break;
			default:
				break;
		}
		delete msg;
	}

	/**
	 * Process all SBP messages received since the last update() call. For each message the corresponding callback will be called (when set).
	 * @return
	 * Fraction of correctly received messages since start of connection
	 */
	float PiksiManager::update()
	{
		//pull all messages from the message buffer
		int msgBufSize = _msgBuf.size();
		while (msgBufSize > 0)
		{
			//pop a message from the buffer
			_msgBufMutex.lock();
			SBP_rawMessage* msg = (_msgBuf.front());
			_msgBuf.pop();
			msgBufSize = _msgBuf.size();
			_msgBufMutex.unlock();

			//handle the message
			handleRawMessage(msg);
		}

		//calculate fraction of messages which have been received correctly
		float numOk = _messages_RX_ok;
		float numFail = _messages_RX_CRCfail;
		float total = numOk + numFail;

		if (total == 0) return 1;
		return numOk / total;
	}

	/*
	 * Setters for callbacks
	 */

	/**
	 * Set callback function for a specific message type
	 * @param cb
	 * Callback function pointer
	 */
	void PiksiManager::setMessageCallback(SBP_StartupCallback cb)
	{
		_startupCB = cb;
	}
	/**
	 * Set callback function for a specific message type
	 * @param cb
	 * Callback function pointer
	 */
	void PiksiManager::setMessageCallback(SBP_HeartbeatCallback cb)
	{
		_heartbeatCB = cb;
	}
	/**
	 * Set callback function for a specific message type
	 * @param cb
	 * Callback function pointer
	 */
	void PiksiManager::setMessageCallback(SBP_DOPSCallback cb)
	{
		_dopsCB = cb;
	}
	/**
	 * Set callback function for a specific message type
	 * @param cb
	 * Callback function pointer
	 */
	void PiksiManager::setMessageCallback(SBP_GPSTimeCallback cb)
	{
		_gpsTimeCB = cb;
	}
	/**
	 * Set callback function for a specific message type
	 * @param cb
	 * Callback function pointer
	 */
	void PiksiManager::setMessageCallback(SBP_positionECEFCallback cb)
	{
		_pos_ecefCB = cb;
	}
	/**
	 * Set callback function for a specific message type
	 * @param cb
	 * Callback function pointer
	 */
	void PiksiManager::setMessageCallback(SBP_positionLLHCallback cb)
	{
		_pos_llhCB = cb;
	}
	/**
	 * Set callback function for a specific message type
	 * @param cb
	 * Callback function pointer
	 */
	void PiksiManager::setMessageCallback(SBP_baselineECEFCallback cb)
	{
		_base_ecefCB = cb;
	}
	/**
	 * Set callback function for a specific message type
	 * @param cb
	 * Callback function pointer
	 */
	void PiksiManager::setMessageCallback(SBP_baselineNEDCallback cb)
	{
		_base_nedCB = cb;
	}
	/**
	 * Set callback function for a specific message type
	 * @param cb
	 * Callback function pointer
	 */
	void PiksiManager::setMessageCallback(SBP_velocityECEFCallback cb)
	{
		_vel_ecefCB = cb;
	}
	/**
	 * Set callback function for a specific message type
	 * @param cb
	 * Callback function pointer
	 */
	void PiksiManager::setMessageCallback(SBP_velocityNEDCallback cb)
	{
		_vel_nedCB = cb;
	}
}
