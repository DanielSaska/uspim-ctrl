#pragma once
#include "mode_controller.h"
#include <string>
#include <vector>
#include <NIDAQmx.h>
#include "json.hpp"
using json = nlohmann::json;

namespace spim
{
	class DaqCommander;
	class NormalModeController : public ModeController
	{
	public:
		enum ZMode : unsigned char
		{
			bidirectional = 0,
			acquireup = 1,
			acquiredown = 2
		};

		NormalModeController
			( double lightSheetWidth1
			, double lightSheetWidth2
			, double xMirrorScale1
			, double xMirrorScale2
			, json const& zMirrorScale1
			, json const& zMirrorScale2
			, double pifocScale
			, double nReturnSlices
			, ZMode zMode
			, int nSlices
			, double zStep
			, double zOffset = 0.0
			, double framerate = 98.5
			, double galvoDelay1 = 0.0 //Difference between in (signal) and out (physical position) of the galvo mirror in ms
			, double galvoDelay2 = 0.0 //Difference between in (signal) and out (physical position) of the galvo mirror in ms
			, double blankReposition = true
			, double triggerOut = true
			, size_t samples = 0
			, size_t cyclesPerFrame = 3
			, std::string masks1 = ""
			, std::string masks2 = "");

		virtual std::string details(){ return ""; };

		virtual bool run(DaqCommander * daq, size_t nVolumes);

	private:
		size_t _samples;
		size_t _cyclesPerFrame;
		double _lightSheetWidth1;
		double _lightSheetWidth2;
		double _xMirrorScale1;
		double _xMirrorScale2;
		json const& _zMirrorScale1;
		json const& _zMirrorScale2;
		double _pifocScale;
		double _zStep;
		double _framerate;
		double _galvoDelay1;
		double _galvoDelay2;
		int _zElements;
		int _nSlices;
		double _nReturnSlices;
		double _zOffset;
		bool _blankReposition;
		bool _triggerOut;
		std::string _masks1;
		std::string _masks2;
		ZMode _zMode;

		std::vector<float64> _output;
			
	};
}
