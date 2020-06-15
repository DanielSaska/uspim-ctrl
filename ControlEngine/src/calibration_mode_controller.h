#pragma once
#include "mode_controller.h"
#include "instruction_consumer.h"
#include <string>
#include <vector>
#include <NIDAQmx.h>
#include <atomic>

namespace spim
{
	class CalibraitonModeController : public ModeController, public InstructionConsumer
	{
	public:
		///Constructor, creates necessary buffers which can be later used for the illumination control
		CalibraitonModeController
		(double lightSheetWidth1	 ///Width of the light sheet
			, double lightSheetWidth2	 ///Width of the light sheet
			, double xMirrorScale1 ///The scale constant to transform distances in micrometers to voltage of the X mirror control
			, double xMirrorScale2 ///The scale constant to transform distances in micrometers to voltage of the X mirror control
			, double pifocScale  ///The scale constant to transform distances in micrometers to voltage of the PIFOC control
			, size_t nCalibrationSamples = 4 ///N >= 2 points used for the linear fit of the resulting Z mirror scale
			, double locationStart = 0 ///Start of the volume used for calibraton (in micrometers)
			, double locationEnd = 400 ///Endo fot he volume used for calibraiton (in micrometers)
			, double framerate = 100 ///Framerate used for the camera capture
			, size_t samples = 0 ///(Rarely needs change) X movement for each cycle is is stored in a buffer, this variable controles its is size
			, double cyclesPerFrame = 3  ///(Rarely needs change) For each cycle (frame), control show man times the X mirror will be cycled
			, std::string masks1 = "" //Laser 1 Mask
			, std::string masks2 = "" //Laser 2 Mask
		);

		///Universal methods providing informaiton about the current class instance
		virtual std::string details() { return "Calibration Mode Controller"; };

		///Executes the calibration modes using the provided DAQ Commander
		///Returns true if calibration was finished correctly, false if halted
		virtual bool run(DaqCommander * daq);

		///Provides means to set the Z Mirror voltage to absolute level defined by the voltage parameter in milliVolts
		///THREADSAFE
		virtual void setVoltage(double voltage)
		{ _voltage.store(voltage, std::memory_order_release); refocus(); };

		///Provides means to obtain current Z Mirror voltage in milliVolts
		///THREADSAFE
		virtual double getVoltage() 
		{ return _voltage.load(std::memory_order_acquire); };

		///Increments the Z Mirror voltage by provided increment in milliVolts
		///THREADSAFE
		virtual void incrementVoltage(double increment = 10)
		{ _voltage.store(getVoltage()+increment, std::memory_order_release); refocus(); };

		///Decrements the Z Mirror voltage by provided decrement in milliVolts
		///THREADSAFE
		virtual void decrementVoltage(double decrement = 10)
		{ _voltage.store(getVoltage()-decrement, std::memory_order_release); refocus();};

		///Indicates to Calibraiton Controller that current Z Mirror voltage is optimal for current Z plane.
		///If enough samples is obtained, the calibration is finished and the estimated Z Mirror scale value is ready
		///otherwise the value is commited and calibration mmoves to the next Z plane
		///THREADSAFE
		virtual void commit();

		///Provides estimated scale for the Z Mirror which can be used to transform distance to control voltage for the mirror
		///in form of a linear fit over _dataPoints or returns 0.0 if number of data points is insufficient (N<2)
		virtual double estimateZMirrorScale();

		///Provides estimated scale for the Z Mirror which can be used to transform distance to control voltage for the mirror
		///in form of a piecewise linear fit over _dataPoints or returns 0.0 if number of data points is insufficient (N<2)
		virtual std::string estimateZMirrorScalePc();

		///Reads instructtion from string and executes it (see implementation for further details)
		virtual bool consumeInstruction(std::string instruction);
	private:
		///Proxy method which invokes slave thread. reporting the result over TCP
		///THREADSAFE
		void sendTcpStatus(std::string status) 
		{ if (serverSocket != nullptr) { std::thread(&ServerSocket::sendStatus, serverSocket,status).detach(); } }
		
		///Recalculates the Z mirror control buffers to reposition the Z mirror
		virtual void refocus();
		///Internal helper which calls methods necessary to populate the DAQ and start illumination and scanning
		virtual void callDaq();

		DaqCommander * _daq; ///Pointer to DAQ Commander to be used

		double _locationStart; ///Start of the volume to be calibrated over in micrometers
		double _locationEnd; ///End  of the volume to be calibrated over in micrometers
		size_t _nCalibrationSamples; ///N>=2 samples used for the linear fit of the Z Mirror distage-voltage scale  parameter estimate
		double _zStepSize; ///Internal variable representing the step size between the calibration samples within the calibrated volume

		std::atomic<double> _voltage; ///Atomic value of the current Z mirror voltage
		std::atomic<double> _location; ///Atomic value of the current PIFOC location (i.e. Z plane)
		std::atomic<size_t> _zStep; ///Atomic value representing the sample used for calibration currently being taken

		double _lightSheetWidth1; ///Width of the light sheet in the X dimension
		double _lightSheetWidth2; ///Width of the light sheet in the X dimension
		size_t _samples; ///Number of samples for the X mirror signal
		double _cyclesPerFrame; ///Number of cycles completed by the X mirror every frame
		double _xMirrorScale1; ///The scale constant to transform distances in micrometers to voltage of the X mirror control
		double _xMirrorScale2; ///The scale constant to transform distances in micrometers to voltage of the X mirror control
		double _pifocScale; ///The scale constant to transform distances in micrometers to voltage of the PIFOC control
		double _framerate; ///Framerate used for the camera capture

		std::atomic<bool> _halt; ///Atomic value true iif the calibration is to be halted, false otherwise
		std::atomic<bool> _done; ///Atomic value true iif the calibraiton is done, false otherwise

		std::vector<double> _dataPointsX; ///Location data for the acquired datapoints
		std::vector<double> _dataPointsY; ///Z-Voltage for the acquired datapoients

		std::vector<float64> _outputx; ///X Mirror signal buffer
		std::vector<float64> _outputsz; ///Z Mirro singal buffer

		std::string _masks1;
		std::string _masks2;
	};
}
