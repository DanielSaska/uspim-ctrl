#include <numeric>
#include <utility>
#include "spim_daq.h"
#include "calibration_mode_controller.h"
#include "socket_listener.h"

spim::CalibraitonModeController::CalibraitonModeController
(double lightSheetWidth1
	, double lightSheetWidth2
	, double xMirrorScale1
	, double xMirrorScale2
	, double pifocScale
	, size_t nCalibrationSamples
	, double locationStart
	, double locationEnd
	, double framerate
	, size_t samples
	, double cyclesPerFrame
	, std::string masks1
	, std::string masks2)
	: _lightSheetWidth1(lightSheetWidth1)
	, _lightSheetWidth2(lightSheetWidth2)
	, _pifocScale(pifocScale)
	, _samples((samples == 0) ? (400000 / framerate / cyclesPerFrame) : samples)
	, _framerate(framerate)
	, _cyclesPerFrame(cyclesPerFrame)
	, _xMirrorScale1(xMirrorScale1)
	, _xMirrorScale2(xMirrorScale2)
	, _nCalibrationSamples((nCalibrationSamples < 2) ? 2 : nCalibrationSamples)
	, _zStepSize((nCalibrationSamples <= 2) ? 0 : ((locationEnd - locationStart) / (nCalibrationSamples - 1)))
	, _zStep(0)
	, _location(locationStart)
	, _voltage(0)
	, _locationStart(locationStart)
	, _locationEnd(locationEnd)
	, _outputx((_samples * cyclesPerFrame) * 8, 0.0)
	, _outputsz(4, 0.0)
	, _done(false)
	, _halt(false)
	, _masks1(masks1)
	, _masks2(masks2)
{
	// Populate the X Mirror Instruction Buffer
	const int wavePeriod = int(_samples);
	const int xSliceLength = _samples * _cyclesPerFrame;
	//X MIRROR 1
	{
		float64* xmirror = &_outputx[0];
		for (size_t i = 0; i < xSliceLength; ++i)
		{
			double pos = ((int)i + 1) % wavePeriod;
			pos = pos < 0.0 ? pos + wavePeriod : pos;
			xmirror[i] = _lightSheetWidth1 / _xMirrorScale1 / 2.0 * (std::abs(pos - wavePeriod / 2) / (double)wavePeriod * 4 - 1); //WTF is that -1?
		}
	}
	//X MIRROR 2
	{
		float64* xmirror2 = &_outputx[xSliceLength];
		for (size_t i = 0; i < xSliceLength; ++i)
		{
			double pos = ((int)i + 1) % wavePeriod;
			pos = pos < 0.0 ? pos + wavePeriod : pos;
			xmirror2[i] = _lightSheetWidth2 / _xMirrorScale2 / 2.0 * (std::abs(pos - wavePeriod / 2) / (double)wavePeriod * 4 - 1); //WTF is that -1?
		}
	}
	//Masks 1
	{
		float64* mask = &_outputx[xSliceLength*2];
		for (size_t i = 0; i < xSliceLength; ++i)
		{
			mask[i] = 5.0;
		}
		if (_masks1 == "disable") {
			for (size_t i = 0; i < xSliceLength; ++i)
			{
				mask[i] = -1;
			}
		}
	}
	//Masks 2
	{
		float64* mask = &_outputx[xSliceLength*3];
		for (size_t i = 0; i < xSliceLength; ++i)
		{
			mask[i] = 5.0;
		}
		if (_masks2 == "disable") {
			for (size_t i = 0; i < xSliceLength; ++i)
			{
				mask[i] = -1;
			}
		}
	}
	//Z+PIFOC
	{
		float64* z = &_outputx[xSliceLength * 4];
		float64* z2 = &_outputx[xSliceLength * 5];
		float64* pifoc = &_outputx[xSliceLength * 6];
		for (size_t i = 0; i < xSliceLength; ++i)
		{
			z[i] = _voltage.load(std::memory_order_acquire) / 1000.0;
			pifoc[i] = _location.load(std::memory_order_acquire) / _pifocScale;
		}
	}
	//Camera trigger
	{
		float64* camTrig = &_outputx[xSliceLength * 7];
		for (size_t i = 0; i < xSliceLength; ++i)
		{
			camTrig[i] = 0.0;
		}
		for (size_t j = 0; j < xSliceLength*0.1; ++j)
		{
			camTrig[j] = 5.0;
		}
	}
	// Send calibration status message over TCP (client can update the UI to show this message to indicate progress of the calibration)
	std::thread(&SocketSender::sendCalibrationStatus, serverSocket->getSender()
		, std::to_string(_zStep + 1) + "/" + std::to_string(_nCalibrationSamples)).detach();
}

bool spim::CalibraitonModeController::run(DaqCommander * daq)
{
	_daq = daq;
	// Refocus the PIFOC and reposition the Z plane by moving the Z  mirror iif we are not done with calibration and the halt instruction has not been issued

		refocus();
	while (!_done.load(std::memory_order_acquire) && !_halt.load(std::memory_order_acquire))
	{
		Sleep(100);
	}
	// Return true if calibration has been halted, false otherwise
	return _halt.load(std::memory_order_acquire);
}

void spim::CalibraitonModeController::commit()
{
	// Add Location-ZMirrorVoltage pair to the data collected
	_dataPointsX.push_back(_location.load(std::memory_order_acquire));
	_dataPointsY.push_back(_voltage.load(std::memory_order_acquire) / 1000.0);
	if (_zStep < _nCalibrationSamples - 1) //More samples need to be acquired
	{
		//Change the PIFOC focus plane (Z plane location) for the next sample collection
		double tmp = _locationEnd - (double)_zStep * _zStepSize;
		_location.store(tmp, std::memory_order_release);
		++_zStep;
		//If we have collected sufficient amount of samples for  linear fit (N>=2), set the Z mirror voltage to the estimate, 0.0 otherwise
		double est = estimateZMirrorScale();
		if (est > tmp / 10) { setVoltage(tmp / est * 1000); }
		else { setVoltage(0.0); }


		// Send calibration status message over TCP (client can update the UI to show this message to indicate progress of the calibration)
		if (estimateZMirrorScale() >= 2)
		{
			std::thread(&SocketSender::sendCalibrationStatus, serverSocket->getSender()
				, std::to_string(_zStep + 1) + "/" + std::to_string(_nCalibrationSamples) + "(" + std::to_string(est) + ")").detach();
		}
		else
		{
			std::thread(&SocketSender::sendCalibrationStatus, serverSocket->getSender()
				, std::to_string(_zStep + 1) + "/" + std::to_string(_nCalibrationSamples)).detach();
		}
		refocus();
	}
	else // Enough samples have been acquired
	{
		// Send calibration status message over TCP (client can update the UI to show this message to indicate progress of the calibration)
		std::thread(&SocketSender::sendCalibrationStatus, serverSocket->getSender()
			, "Done, zMirrorScale=" + std::to_string(estimateZMirrorScale()) + ", zMirrorScalePc="+ estimateZMirrorScalePc()).detach();
		_done.store(true, std::memory_order_release); //We are done
		_daq->pause(); //Halt the DAQ operation
	}
}

//Helper method for summing pairs
template <typename T1, typename T2> struct pair_sum : public std::binary_function< std::pair<T1, T2>, std::pair<T1, T2>, std::pair<T1, T2> >
{
	std::pair<T1, T2> operator()(const std::pair<T1, T2>& lhs, const std::pair<T1, T2>& rhs)
	{ return std::pair<T1, T2>(lhs.first + rhs.first, lhs.second + rhs.second); }
};


double spim::CalibraitonModeController::estimateZMirrorScale()
{
	if (_dataPointsX.size() >= 2) //If we have enough data points, calculate linear fit
	{
		const auto n = _dataPointsX.size();
		const auto s_x = std::accumulate(_dataPointsX.begin(), _dataPointsX.end(), 0.0);
		const auto s_y = std::accumulate(_dataPointsY.begin(), _dataPointsY.end(), 0.0);
		const auto s_xx = std::inner_product(_dataPointsX.begin(), _dataPointsX.end(), _dataPointsX.begin(), 0.0);
		const auto s_xy = std::inner_product(_dataPointsX.begin(), _dataPointsX.end(), _dataPointsY.begin(), 0.0);
		const auto a = (n * s_xy - s_x * s_y) / (n * s_xx - s_x * s_x);
		return 1.0/a;
	}
	return 0.0; //Otherwise just return 0
}

std::string spim::CalibraitonModeController::estimateZMirrorScalePc()
{
	if (_dataPointsX.size() >= 2) //If we have enough data points, calculate linear fit
	{
		std::string ret = "{scales: [";
		std::vector<double> dataPointsX;
		std::vector<double> dataPointsY;
		for (size_t i = 0; i < _dataPointsX.size(); ++i)
		{
			size_t j = 0;
			if (i > 0)
			{
				j = _dataPointsX.size() - i;
			}
			dataPointsX.push_back(_dataPointsX[j]);
			dataPointsY.push_back(_dataPointsY[j]);
		}

		for (size_t i = 0; i < dataPointsX.size() - 1; ++i) 
		{
			double x1 = dataPointsX[i];
			double x2 = dataPointsX[i+1];
			double y1 = dataPointsY[i];
			double y2 = dataPointsY[i+1];
			ret += "[" + std::to_string(x1) + ", " + std::to_string((x2-x1)/(y2-y1)) + ", " + std::to_string(y1) + "]";
			if (i < dataPointsX.size() - 2) { ret += ","; }
		}
		return ret+"]}";
	}
	return "{scales: []}"; //Otherwise just return 0
}

bool spim::CalibraitonModeController::consumeInstruction(std::string instruction)
{
	static std::string const& setv{ "calibrationSetVoltage" };
	static std::string const& incr{ "calibrationIncrementVoltage" };
	static std::string const& decr{ "calibrationDecrementVoltage" };

	if (instruction == "halt") // Halt instruction received
	{
		_halt.store(true, std::memory_order_release); //Flag the halt instruciton
		_daq->pause(); // Interrupt the DAQ
		return true;
	}
	if (instruction == "calibrationCommit") // Confirm the current Z Mirror voltage as correct
	{
		commit();
		return true;
	}
	else if (instruction.compare(0, incr.length(), incr) == 0) //Increment instruction received
	{
		instruction = instruction.erase(0, incr.length()); //Increment value immediately follows
		incrementVoltage(std::stod(instruction)); //Increment by the value received
		return true;
	}
	else if (instruction.compare(0, decr.length(), decr) == 0) //Decrement instruction received
	{
		instruction = instruction.erase(0, decr.length()); //Decrement value immediately follows
		decrementVoltage(std::stod(instruction)); //Decrement by the value received
		return true;
	}
	else if (instruction.compare(0, setv.length(), setv) == 0) //Set Voltage instruction received
	{
		instruction = instruction.erase(0, setv.length()); //Votlage value immediately follows
		setVoltage(std::stod(instruction)); //Set the voltage value
		return true;
	}
	return false;
}

void spim::CalibraitonModeController::refocus()
{
	//Send the voltage over TCP (Client can use this to display the current voltage level in the UI)
	std::thread(&SocketSender::sendCalibrationOffset, serverSocket->getSender()
		, std::to_string(_voltage.load(std::memory_order_acquire))).detach();
	//Send status over TCP (Client can log this)
	sendTcpStatus("Refocusing on Z plane at "
		+ std::to_string(_location.load(std::memory_order_acquire)) + "um and repositioning beam with Z Mirror Voltage of "
		+ std::to_string(_voltage.load(std::memory_order_acquire)) + "mV.");

	//Z+PIFOC
	{
		const int xSliceLength = _samples * _cyclesPerFrame;
		float64* z = &_outputx[xSliceLength * 4];
		float64* z2 = &_outputx[xSliceLength * 5];
		float64* pifoc = &_outputx[xSliceLength * 6];
		for (size_t i = 0; i < xSliceLength; ++i)
		{
			z[i] = _voltage.load(std::memory_order_acquire) / 1000.0;
			z2[i] = _voltage.load(std::memory_order_acquire) / 1000.0;
			pifoc[i] = _location.load(std::memory_order_acquire) / _pifocScale;
		}
	}
	callDaq(); //Start scanning with these parameters
}

void spim::CalibraitonModeController::callDaq()
{
	_daq->setFramerateDynamic(_framerate, 400000);
	_daq->setOutputDynamic(_outputx, _lightSheetWidth1, _lightSheetWidth2, 0, 0, _location.load(std::memory_order_acquire), _location.load(std::memory_order_acquire), 1, 0, spim::DaqCommander::ZMode(1)
		, _xMirrorScale1, _xMirrorScale2, 10000, 10000, _pifocScale);
	_daq->setTriggerOut(true);
	_daq->deploy();
}
