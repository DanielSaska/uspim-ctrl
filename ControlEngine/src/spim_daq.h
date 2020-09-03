#pragma once
#include <string>
#include <vector>
#include <NIDAQmx.h>
#include <atomic>
#include <mutex>
#include "server_socket.h"
#include "instruction_consumer.h"

namespace spim
{
	class DaqCommander : public InstructionConsumer
	{
	public:
		enum ZMode : unsigned char
		{
			bidirectional = 0,
			acquireup = 1,
			acquiredown = 2
		};

		DaqCommander
			( bool countFramesInternally = true
			, float repositionSpeed = 600 //um/s
			, bool edge = 0 //0=Rising, 1=Falling
			, std::string xMirror1Channel = "/Dev1/AO0"
			, std::string xMirror2Channel = "/Dev1/AO1"
			, std::string laser1Channel = "/Dev1/AO2"
			, std::string laser2Channel = "/Dev1/AO3"
			, std::string zMirror1Channel = "/Dev1/AO4"
			, std::string zMirror2Channel = "/Dev1/AO5"
			, std::string pifocChannel = "/Dev1/AO6"
			, std::string shutterChannel = "/Dev1/AO7"
			, std::string counterSource = "/Dev1/Ctr0InternalOutput"
			, std::string triggerSource = "/Dev1/PFI0"
			, std::string counterChannel = "/Dev1/Ctr0"
			, std::string clockSignalChannel = "/Dev1/Ctr1"
			, std::string clockSignalTerminal = "/Dev1/PFI1"
			);
		~DaqCommander();

		void setFrequency(float64 frequency, float64 rate);
		void setRate(float64 rate);
		void setOutput(std::vector<float64> outputx
			, double xWidth1
			, double xWidth2
			, double galvoDelay1
			, double galvoDelay2
			, double zStart, double zEnd
			, int down, int up
			, ZMode zMode);

		void execute()
		{
			if (_thr != nullptr && _thr->joinable()) {
				_thr->join();
				delete _thr;
				_thr = nullptr;
			}
			if (_thr == nullptr) { _thr = new std::thread(&DaqCommander::_execute, this); }
		}
		void deploy() { _mutex.lock(); _deploy = true; _mutex.unlock(); }
		void setOutputDynamic(std::vector<float64> outputSpim
			, double xWidth1
			, double xWidth2
			, double galvoDelay1
			, double galvoDelay2
			, double zStart, double zEnd
			, int down, int up
			, ZMode zMode
			, double xMirrorScale1
			, double xMirrorScale2
			, double zMirrorScale1
			, double zMirrorScale2
			, double pifocScale);
		void setFramerateDynamic(float64 frequency, float64 rate);
		void setBlankReposition(bool blankReposition) { _blankReposition = blankReposition; }

		bool isRunning() { return _running.load(std::memory_order_acquire); }
		bool halt() { //Returns true if thread was running, false otherwise
			if (_running.load(std::memory_order_acquire))
			{ _halt.store(true, std::memory_order_release); return true; } 
			else 
			{ return false; }
		}
		void pause() {
			if (_running.load(std::memory_order_acquire))
			{
				setTriggerOut(false);
				this->setFramerateDynamic(1,1);
				this->setOutputDynamic({ 0.0, 0.0,0.0, 0.0, -1.0, -1.0, -1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
					, 0, 0, 0, 0, 0, 0, 0, 0, ZMode(0),1,1,1,1,100);
				this->deploy();
			}
		}

		virtual bool consumeInstruction(std::string instruction)
		{
			if (instruction == "halt")
			{
				halt(); return true;
			}
			else if (instruction == "pause")
			{
				pause(); return true;
			}
			return false; 
		};

		void setTriggerOut(bool shouldTrigger);
	private:
		void sendTcpStatus(std::string status) 
		{ if (serverSocket != nullptr) { std::thread(&ServerSocket::sendStatus, serverSocket,status).detach(); } }
		void sendTcpCustom(std::string msg) 
		{ if (serverSocket != nullptr) { std::thread(&ServerSocket::sendCustom, serverSocket,msg).detach(); } }

		void _execute();
		void _setTriggerOut();
		void reposition(double form, double to);
		std::thread * _thr; //DAQ Command thread

		TaskHandle _spimTask = 0;

		TaskHandle _counterTask = 0;
		TaskHandle _countFrames = 0;
		TaskHandle _counterTaskPIFOC = 0;
		TaskHandle _clockSignal = 0;

		//Init variables, not allowed to change on fly atm
		float64 _rate;
		float64 _clockSignalFrequency;
		bool _countFramesInternally;
		bool _edge;
		std::string _xMirror1Channel;
		std::string _xMirror2Channel;
		std::string _laser1Channel;
		std::string _laser2Channel;
		std::string _zMirror1Channel;
		std::string _zMirror2Channel;
		std::string _pifocChannel;
		std::string _shutterChannel;

		std::string _counterSource;
		std::string _triggerSource;
		std::string _triggerSourcePIFOC;
		std::string _countFramesChannel;
		std::string _counterChannel;
		std::string _counterSourceZ;
		std::string _clockSignalChannel;
		std::string _clockSignalTerminal;

		std::atomic<bool> _running;
		std::atomic<bool> _halt;

		float _repositionSpeed;
		size_t _cycle;
		size_t _length;
		double _zStart;
		double _zEnd;
		double _xWidth1;
		double _xWidth2;
		double _galvoDelay1;
		double _galvoDelay2;
		int _down;
		int _up;
		ZMode _zMode;
		std::atomic<bool>  _triggerOut;

		 std::mutex _mutex; //Must be locked when accessing the below
		 std::vector<float64> _outputSpim;
		 double _pendingXWidth1;
		 double _pendingXWidth2;
		 double _pendingGalvoDelay1;
		 double _pendingGalvoDelay2;
		 double _pendingZStart;
		 double _pendingZEnd;
		 int _pendingDown;
		 int _pendingUp;
		 ZMode _pendingZMode;
		 bool _blankReposition; //Always true unless setBlankReposition() is set prior to deploy()


		 float64 _frequency;
		 float64 _pendingRate;
		 double _xMirrorScale1;
		 double _xMirrorScale2;
		 double _zMirrorScale1;
		 double _zMirrorScale2;
		 double _pifocScale; 



		 std::atomic<bool> _deploy;
		 size_t _repositioning;

	};
}
