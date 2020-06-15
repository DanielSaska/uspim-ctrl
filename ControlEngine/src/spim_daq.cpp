#include <NIDAQmx.h>
#include <string>
#include <iostream>
#include "spim_daq.h"

#ifdef DAQmxErrChk
#undef DAQmxErrChk
#endif
#define DAQmxErrChk(functionCall) {int error; if( DAQmxFailed(error=(functionCall)) ) { char errBuff[2048] = {'\0'};  DAQmxGetExtendedErrorInfo(errBuff, 2048); printf("DAQmx Error: %s\n", errBuff); sendTcpStatus("DAQmx Error:"+std::string(errBuff)); } }

spim::DaqCommander::DaqCommander
(bool countFramesInternally
, float repositionSpeed
, bool edge
, std::string xMirror1Channel
, std::string xMirror2Channel
, std::string laser1Channel
, std::string laser2Channel
, std::string zMirror1Channel
, std::string zMirror2Channel
, std::string pifocChannel
, std::string shutterChannel
, std::string counterSource
, std::string triggerSource
, std::string counterChannel
, std::string clockSignalChannel
, std::string clockSignalTerminal)
: _repositionSpeed(repositionSpeed)
, _xMirror1Channel(xMirror1Channel)
, _xMirror2Channel(xMirror2Channel)
, _laser1Channel(laser1Channel)
, _laser2Channel(laser2Channel)
, _zMirror1Channel(zMirror1Channel)
, _zMirror2Channel(zMirror2Channel)
, _pifocChannel(pifocChannel)
, _shutterChannel(shutterChannel)
, _counterSource(counterSource)
, _triggerSource(triggerSource)
, _counterChannel(counterChannel)
, _clockSignalChannel(clockSignalChannel)
, _clockSignalTerminal(clockSignalTerminal)
, _clockSignalFrequency(0.0)
, _edge(edge)
, _rate(0.0)
, _spimTask(nullptr)
, _counterTask(nullptr)
, _counterTaskPIFOC(nullptr)
, _clockSignal(nullptr)
, _cycle(0)
, _length(1)
, _triggerOut(true)
, _deploy(false)
, _blankReposition(true)
, _frequency(1.0)
, _pendingRate(0.0)
, _galvoDelay1(0.0)
, _galvoDelay2(0.0)
, _zMode(ZMode::acquiredown)
, _repositioning(0)
, _pifocScale(100)
, _thr(nullptr)
{
	_halt.store(false, std::memory_order_release);
	_running.store(false, std::memory_order_release);
	DAQmxErrChk(DAQmxCreateTask("xMirror1ControlTask", &_spimTask));


	DAQmxCreateTask("counterOutTaskZ", &_counterTaskPIFOC);


	DAQmxErrChk(DAQmxCreateAOVoltageChan(_spimTask, _xMirror1Channel.c_str(), "xMirror1Channel", -10.0, 10.0, DAQmx_Val_Volts, NULL));
	DAQmxErrChk(DAQmxCreateAOVoltageChan(_spimTask, _xMirror2Channel.c_str(), "xMirror2Channel", -10.0, 10.0, DAQmx_Val_Volts, NULL));
	DAQmxErrChk(DAQmxCreateAOVoltageChan(_spimTask, _laser1Channel.c_str(), "laser1Channel", -10.0, 10.0, DAQmx_Val_Volts, NULL));
	DAQmxErrChk(DAQmxCreateAOVoltageChan(_spimTask, _laser2Channel.c_str(), "laser2Channel", -10.0, 10.0, DAQmx_Val_Volts, NULL));
	DAQmxErrChk(DAQmxCreateAOVoltageChan(_spimTask, _zMirror1Channel.c_str(), "zMirror1Channel", -10.0, 10.0, DAQmx_Val_Volts, NULL));
	DAQmxErrChk(DAQmxCreateAOVoltageChan(_spimTask, _zMirror2Channel.c_str(), "zMirror2Channel", -10.0, 10.0, DAQmx_Val_Volts, NULL));
	DAQmxErrChk(DAQmxCreateAOVoltageChan(_spimTask, _pifocChannel.c_str(), "pifocChannel", -10.0, 10.0, DAQmx_Val_Volts, NULL));
	DAQmxErrChk(DAQmxCreateAOVoltageChan(_spimTask, _shutterChannel.c_str(), "shutterChannel", -10.0, 10.0, DAQmx_Val_Volts, NULL));





	/*********************************************/
	// DAQmx Start Code
	//First reserve resources for all but oner of the tasks, and then start the last (unreserved) task <--- What does this mean???
	/*********************************************/

	DAQmxErrChk(DAQmxTaskControl(_spimTask, DAQmx_Val_Task_Reserve));

	//DAQmxErrChk(DAQmxReadCounterScalarU32(countFrames, 0.0, &framecounterOld, NULL));


}

spim::DaqCommander::~DaqCommander()
{
	halt();

	if (_thr != nullptr && _thr->joinable()) {
		_thr->join();
		delete _thr;
	}
	if (_spimTask != nullptr)
	{
		DAQmxStopTask(_spimTask);
		DAQmxTaskControl(_spimTask, DAQmx_Val_Task_Unreserve);
		DAQmxClearTask(_spimTask);
		_spimTask = nullptr;
	}
	if (_counterTaskPIFOC != nullptr)
	{
		DAQmxStopTask(_counterTaskPIFOC);
		DAQmxTaskControl(_counterTaskPIFOC, DAQmx_Val_Task_Unreserve);
		DAQmxClearTask(_counterTaskPIFOC);
		_counterTaskPIFOC = nullptr;
	}
	if (_counterTask != nullptr)
	{
		DAQmxStopTask(_counterTask);
		DAQmxTaskControl(_counterTask, DAQmx_Val_Task_Unreserve);
		DAQmxClearTask(_counterTask);
		_counterTask = nullptr;
	}
	if (_clockSignal != nullptr)
	{
		DAQmxStopTask(_clockSignal);
		DAQmxTaskControl(_clockSignal, DAQmx_Val_Task_Unreserve);
		DAQmxClearTask(_clockSignal);
		_clockSignal = nullptr;
	}
}


void spim::DaqCommander::setFrequency(float64 frequency, float64 rate)
{
	_clockSignalFrequency = frequency;
	sendTcpCustom("DAQ,FPS," + std::to_string(_clockSignalFrequency) + "," + std::to_string(_rate));

	if (_clockSignal != nullptr)
	{
		DAQmxTaskControl(_clockSignal, DAQmx_Val_Task_Unreserve);
		DAQmxClearTask(_clockSignal);
		_clockSignal = nullptr;
	}

}

void spim::DaqCommander::setRate(float64 rate)
{
	_rate = rate;
	sendTcpCustom("DAQ,FPS," + std::to_string(_clockSignalFrequency) + "," + std::to_string(_rate));

	if (_counterTask != nullptr)
	{
		DAQmxTaskControl(_counterTask, DAQmx_Val_Task_Unreserve);
		DAQmxClearTask(_counterTask);
		_counterTask = nullptr;
	}

	DAQmxCreateTask("counterOutTask", &_counterTask);
	DAQmxCreateCOPulseChanFreq(_counterTask, _counterChannel.c_str(), "coChannel", DAQmx_Val_Hz, DAQmx_Val_Low, 0, _rate, 0.5);

	DAQmxErrChk(DAQmxTaskControl(_counterTask, DAQmx_Val_Task_Reserve));
}

void spim::DaqCommander::setOutput(std::vector<float64> outputSpim, double xWidth1, double xWidth2, double galvoDelay1, double galvoDelay2, double zStart, double zEnd, int down, int up, ZMode zMode)
{
	_length = outputSpim.size()/8;
	_galvoDelay1 = galvoDelay1;
	_galvoDelay2 = galvoDelay2;
	_xWidth1 = xWidth1;
	_xWidth2 = xWidth2;
	sendTcpCustom("DAQ,X," + std::to_string(_xWidth1) + "," + std::to_string(_xWidth2) + "," + std::to_string(_galvoDelay1) + "," + std::to_string(_galvoDelay2));

	DAQmxErrChk(DAQmxCfgSampClkTiming(_spimTask, _counterSource.c_str(), _rate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, NULL));

	_zStart = zStart;
	_zEnd = zEnd;
	_down = down;
	_up = up;
	_zMode = zMode;
	sendTcpCustom("DAQ,Z," + std::to_string(_zStart) + "," + std::to_string(_zEnd) + "," +
		std::to_string(_down) + "," + std::to_string(_up) + "," + std::to_string((int)_zMode));

	DAQmxErrChk(DAQmxSetWriteAttribute(_spimTask, DAQmx_Write_RegenMode, DAQmx_Val_AllowRegen));

	DAQmxCfgOutputBuffer(_spimTask, _length);
	//Might have to set it staticallu up to now to a large static value just like the original code

	DAQmxCfgImplicitTiming(_counterTask, DAQmx_Val_ContSamps, _length);

	DAQmxErrChk(DAQmxWriteAnalogF64(_spimTask, _length, 0, 20.0, DAQmx_Val_GroupByChannel, &outputSpim[0], NULL, NULL));
}


void spim::DaqCommander::setTriggerOut(bool shouldTrigger)
{
	_triggerOut = shouldTrigger;
}

void spim::DaqCommander::_execute()
{
	sendTcpStatus("Starting DAQ command...");
	_halt.store(false, std::memory_order_release);
	_running.store(true, std::memory_order_release);
	bool halted = false;
	bool lastTriggerOut = true;
	uInt32	  framecounter = 0;
	uInt32	  framecounterPrev = 0;
	_cycle = framecounterPrev;
	DAQmxErrChk(DAQmxStartTask(_spimTask));
	DAQmxErrChk(DAQmxStartTask(_counterTask));
	//DAQmxErrChk(DAQmxStartTask(_clockSignal));
	auto prev = std::chrono::high_resolution_clock::now();
	double dur = (1000.0 / _frequency) * 1000 * 1000;//nanoseconds


	sendTcpStatus("DAQ task started.");

	while (true)
	{
		//Software frame counter


		//Handle halt
		if (_halt.load(std::memory_order_acquire) == true) 
		{ halted = true; sendTcpStatus("DAQ halt was received. Halting."); break; }

		auto now = std::chrono::high_resolution_clock::now();
		if (std::chrono::duration_cast<std::chrono::nanoseconds>(now - prev).count() > dur)
		{ prev = now; ++framecounter; }

		//New frame?
		if (framecounterPrev != framecounter) 
		{
			/*
			if (_cycle % (nImages / 20) == 0) 
			{
				sendTcpStatus("DAQ task execution: " + std::to_string(_cycle * 100 / nImages) + "%% done (" + std::to_string(_cycle) + "/" + std::to_string(nImages) + ").");
			}
			*/
			framecounterPrev = framecounter;
			_cycle = framecounter;

			{
				size_t const& pos = ((_up + _down) == 0) ? 0 : _cycle % (_up+_down);
				//DAQ=info identifier,PD=pos+direction info,D/U=direction,R/-=recording or not,plane position
				if (pos < _down) 
				{
					if (_zMode != ZMode::acquireup)
					{ sendTcpCustom("DAQ,PD,D,R,"+std::to_string(pos)); }
					else
					{ sendTcpCustom("DAQ,PD,D,-,"+std::to_string(pos)); }
				}
				else
				{
					if (_zMode != ZMode::acquiredown)
					{ sendTcpCustom("DAQ,PD,U,R,"+std::to_string(pos-_down)); }
					else
					{ sendTcpCustom("DAQ,PD,U,-,"+std::to_string(pos-_down)); }
				}
			}

			if (_deploy)
			{
				if (_zMirrorScale1 != 0 && _zMirrorScale2 != 0)
				{
					int const& pos = ((_up + _down) == 0) ? 0 : _cycle % (_up + _down);
					double nowPos = _zStart + ((pos < _down)
						? (_zEnd - _zStart) * double(pos) / double(_down)
						: (_zEnd - _zStart) * -1 * double(pos - _down) / double(_up));
					if (pos == 0 && _zStart == 0) { nowPos = 0; }
					dur = (1000.0 / (_repositionSpeed)) * 1000 * 1000;//nanoseconds
					reposition(nowPos, _pendingZStart);
					_mutex.lock();
					std::cout << _frequency << std::endl;
					framecounter = 0;
					_deploy = false;
					_mutex.unlock();
				}
				else //We skip repositioning if the scale is piecewise linear for now
				{
					_repositioning = 1;
				}
			}
			if (_repositioning > 0 && _repositioning <= _cycle) //We successfuly repositioned
			{
				_mutex.lock();
				DAQmxErrChk(DAQmxStopTask(_spimTask));
				DAQmxStopTask(_counterTask);
				//DAQmxStopTask(_clockSignal);
				setRate(_pendingRate);
				setFrequency(_frequency, _rate);
				setOutput(_outputSpim, _pendingXWidth1, _pendingXWidth2, _pendingGalvoDelay1, _pendingGalvoDelay2, _pendingZStart, _pendingZEnd, _pendingDown, _pendingUp, _pendingZMode);
				_cycle = 0;
				DAQmxErrChk(DAQmxStartTask(_spimTask));
				DAQmxErrChk(DAQmxStartTask(_counterTask));
				//DAQmxErrChk(DAQmxStartTask(_clockSignal));
				_repositioning = 0;
				_mutex.unlock();
				dur = (1000.0 / _frequency) * 1000 * 1000;//nanoseconds
				framecounter = 0;
			}
			if (lastTriggerOut != _triggerOut) { lastTriggerOut = _triggerOut; _setTriggerOut();  }
		}
	}
	DAQmxErrChk(DAQmxStopTask(_spimTask));

	DAQmxStopTask(_counterTask);
	//DAQmxStopTask(_clockSignal);
	_halt.store(false, std::memory_order_release);
	_running.store(false, std::memory_order_release);
	std::cout << "DAQ task has finished." << std::endl;
	sendTcpStatus("DAQ task has finished.");
}

void spim::DaqCommander::_setTriggerOut()
{
	sendTcpCustom(_triggerOut ? "DAQ,TRIG,1" : "DAQ,TRIG,0");
	//uInt32	  framecounter = 0;
	//DAQmxErrChk(DAQmxStopTask(_spimTask));
	//DAQmxStopTask(_counterTask);
	//DAQmxStopTask(_clockSignal);
	//if (_triggerOut)
	//{
	//	DAQmxErrChk(DAQmxSetCOPulseTerm(_clockSignal, _clockSignalChannel.c_str(), _clockSignalTerminal.c_str()));
	//}
	//else
	//{
	//	DAQmxErrChk(DAQmxResetCOPulseTerm(_clockSignal, _clockSignalChannel.c_str()));
	//}
	//DAQmxErrChk(DAQmxStartTask(_spimTask));
	//DAQmxErrChk(DAQmxStartTask(_counterTask));
	//DAQmxErrChk(DAQmxStartTask(_clockSignal));
}

void spim::DaqCommander::reposition(double from, double to)
{
	if (from == to) // No need for repositioning
	{
		_blankReposition = true;
		_repositioning = 1;
		_cycle = 1;
		return;
	}
	std::vector<float64> zO;
	size_t const& n_steps = (int(std::abs(to - from)) + 2)*100;
	zO.reserve((n_steps+ _repositionSpeed)*8);

	//Blank during reposition (or not)
	if (_blankReposition)
	{
		//X is zero during reposition
		for (size_t i = 0; i < (_repositionSpeed + n_steps) * 2; ++i)
		{
			zO.push_back(0);
		}
		for (size_t i = 0; i < (_repositionSpeed + n_steps) * 2; ++i)
		{
			zO.push_back(0);
		}
	}
	else
	{
		double period = 100*_repositionSpeed/200.0;
		for (size_t i = 0; i < (_repositionSpeed + n_steps); ++i)
		{
			double pos = ((int)i + 1) % (int)period;
			pos = pos < 0.0 ? pos + period : pos;
			zO.push_back(_pendingXWidth1 / _xMirrorScale1 / 2.0 * (std::abs(pos - period / 2) / (double)period * 4 - 1));
		}
		for (size_t i = 0; i < (_repositionSpeed + n_steps); ++i)
		{
			double pos = ((int)i + 1) % (int)period;
			pos = pos < 0.0 ? pos + period : pos;
			zO.push_back(_pendingXWidth2 / _xMirrorScale2 / 2.0 * (std::abs(pos - period / 2) / (double)period * 4 - 1));
		}

		for (size_t i = 0; i < (_repositionSpeed + n_steps) * 2; ++i)
		{
			zO.push_back(5.0);
		}
	}

	//Set Z mirror to their pending start positions
	for (size_t i = 0; i < (_repositionSpeed + n_steps); ++i)
	{
		zO.push_back(_pendingZStart / _zMirrorScale1);
	}
	for (size_t i = 0; i < (_repositionSpeed + n_steps); ++i)
	{
		zO.push_back(_pendingZStart / _zMirrorScale2);
	}

	for (size_t i = 0; i < n_steps; ++i)
	{
		zO.push_back((from + (to - from) / double(n_steps) * double(i)) / _pifocScale);
	}
	for (size_t i = 0; i < _repositionSpeed; ++i)
	{
		zO.push_back(to / _pifocScale);
	}
	for (size_t i = 0; i < (n_steps+ _repositionSpeed); ++i)
	{
		zO.push_back(0);
	}


	DAQmxErrChk(DAQmxStopTask(_spimTask));
	DAQmxStopTask(_counterTask);
	//DAQmxStopTask(_clockSignal);
	setRate(_repositionSpeed*100);
	setFrequency(_repositionSpeed*100, _repositionSpeed*100);
	//TODO
	setOutput(zO, 0, 0, 0, 0
		, (from < to) ? from : to, (from > to) ? from : to
		, (from <= to) ? n_steps/100 : 0
		, (from > to) ? n_steps/100 : 0, ZMode::bidirectional);
	_repositioning = n_steps/100;
	_cycle = 0;
	uInt32	  framecounter = 0;
	//DAQmxErrChk(DAQmxResetCOPulseTerm(_clockSignal, _clockSignalChannel.c_str()));
	DAQmxErrChk(DAQmxStartTask(_spimTask));
	DAQmxErrChk(DAQmxStartTask(_counterTask));
	//DAQmxErrChk(DAQmxStartTask(_clockSignal));

	_blankReposition = true;
}

void spim::DaqCommander::setOutputDynamic(std::vector<float64> outputSpim, double xWidth1, double xWidth2
	, double galvoDelay1, double galvoDelay2
	, double zStart, double zEnd, int down, int up, ZMode zMode
	, double xMirrorScale1
	, double xMirrorScale2
	, double zMirrorScale1
	, double zMirrorScale2
	, double pifocScale)
{
	if (_running.load(std::memory_order_acquire) == false) { return; }
	_mutex.lock();

	_pendingZStart = zStart;
	_pendingZEnd = zEnd;
	_pendingDown = down;
	_pendingUp = up;
	_pendingZMode = zMode;
	_pendingXWidth1 = xWidth1;
	_pendingXWidth2 = xWidth2;
	_pendingGalvoDelay1 = galvoDelay1;
	_pendingGalvoDelay2 = galvoDelay2;
	_outputSpim = outputSpim;

	_xMirrorScale1 = xMirrorScale1;
	_xMirrorScale2 = xMirrorScale2;
	_zMirrorScale1 = zMirrorScale1;
	_zMirrorScale2 = zMirrorScale2;
	_pifocScale = pifocScale;
	_mutex.unlock();
}


void spim::DaqCommander::setFramerateDynamic(float64 frequency, float64 rate)
{
	if (_running.load(std::memory_order_acquire) == false) { return; }
	_mutex.lock();
	if (_frequency != frequency || _rate != rate || _pendingRate != rate)
	{
		_frequency = frequency;
		_pendingRate = rate;
	}
	_mutex.unlock();
}
