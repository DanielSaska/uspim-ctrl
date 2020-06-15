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
	class FindPlaneModeController : public ModeController
	{
	public:
		FindPlaneModeController
			( double lightSheetWidth1
			, double lightSheetWidth2
			, double xMirrorScale1
			, double xMirrorScale2
			, json const& zMirrorScale1 = 1
			, json const& zMirrorScale2 = 1
			, double pifocScale = 1
			, double zOffset = 0 //PIFOC and Z Mirror scales are necessary if zOffset is used
			, double framerate = 100
			, double galvoDelay1 = 0.5 //Difference between in (signal) and out (physical position) of the galvo mirror in ms
			, double galvoDelay2 = 0.5 //Difference between in (signal) and out (physical position) of the galvo mirror in ms
			, size_t samples = 0
			, double cyclesPerFrame = 3
			, std::string masks1 = ""
			, std::string masks2 = "");

		virtual std::string details(){ return ""; };

		virtual bool run(DaqCommander * daq, size_t nVolumes);
	private:
		double _lightSheetWidth1;
		double _lightSheetWidth2;
		size_t _samples;
		double _cyclesPerFrame;
		double _xMirrorScale1;
		double _xMirrorScale2;
		json const& _zMirrorScale1;
		json const& _zMirrorScale2; 
		double _pifocScale;
		double _zOffset;
		double _framerate;
		double _galvoDelay1;
		double _galvoDelay2;
		std::string _masks1;
		std::string _masks2;

		std::vector<float64> _output;
			
	};
}
