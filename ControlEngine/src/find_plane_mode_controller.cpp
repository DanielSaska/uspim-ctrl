#include <cmath>
#include "find_plane_mode_controller.h"
#include "spim_daq.h"

spim::FindPlaneModeController::FindPlaneModeController
 (double lightSheetWidth1
, double lightSheetWidth2
, double xMirrorScale1
, double xMirrorScale2
, json const& zMirrorScale1
, json const& zMirrorScale2
, double pifocScale
, double zOffset
, double framerate
, double galvoDelay1
, double galvoDelay2
, size_t samples
, double cyclesPerFrame
, std::string masks1
, std::string masks2)
: _lightSheetWidth1(lightSheetWidth1)
, _lightSheetWidth2(lightSheetWidth2)
, _samples((samples == 0) ? (400000 / framerate / cyclesPerFrame) : samples)
, _framerate(framerate)
, _galvoDelay1(galvoDelay1)
, _galvoDelay2(galvoDelay2)
, _cyclesPerFrame(cyclesPerFrame)
, _xMirrorScale1(xMirrorScale1)
, _xMirrorScale2(xMirrorScale2)
, _zMirrorScale1(zMirrorScale1)
, _zMirrorScale2(zMirrorScale2)
, _pifocScale(pifocScale)
, _zOffset(zOffset)
, _output(size_t(_samples * cyclesPerFrame) * 8, 0.0)
, _masks1(masks1)
, _masks2(masks2)

{
	const int wavePeriod = int(_samples);
	const int xSliceLength = _samples * cyclesPerFrame;
	//X MIRROR 1
	{
		const int gDelay1 = _galvoDelay1 / (1000.0 / double(_samples * framerate));
		float64* xmirror = &_output[0];
		for (size_t i = 0; i < xSliceLength; ++i)
		{
			double pos = ((int)i + 1 - gDelay1) % wavePeriod;
			pos = pos < 0.0 ? pos + wavePeriod : pos;
			xmirror[i] = _lightSheetWidth1 / _xMirrorScale1 / 2.0 * (std::abs(pos - wavePeriod / 2) / (double)wavePeriod * 4 - 1);
		}
	}
	//X MIRROR 2
	{
		const int gDelay2 = _galvoDelay2 / (1000.0 / double(_samples * framerate));
		float64* xmirror2 = &_output[xSliceLength];
		for (size_t i = 0; i < xSliceLength; ++i)
		{
			double pos = ((int)i + 1 - gDelay2) % wavePeriod;
			pos = pos < 0.0 ? pos + wavePeriod : pos;
			xmirror2[i] = _lightSheetWidth2 / _xMirrorScale2 / 2.0 * (std::abs(pos - wavePeriod / 2) / (double)wavePeriod * 4 - 1);
		}
	}

	//Masks
	{
		float64* mask = &_output[xSliceLength*2];
		for (size_t i = 0; i < xSliceLength; ++i)
		{
			mask[i] = 5.0;
		}

		if (_masks1 == "disable")
		{
			for (size_t i = 0; i < xSliceLength; ++i)
			{
				mask[i] = -1;
			}
		}
		else if (_masks1 == "adjustment_mask")
		{
			for (size_t i = 0; i < xSliceLength; ++i)
			{
				const double tmpx = std::abs((int)((i + 1) % wavePeriod) - wavePeriod / 2);
				if (!(tmpx <(_samples / 2)*0.1 || tmpx >(_samples / 2)*0.9) &&
					!(tmpx >(_samples / 2)*0.2 && tmpx < (_samples / 2)*0.25) &&
					!(tmpx >(_samples / 2)*0.75 && tmpx < (_samples / 2)*0.8) &&
					!(tmpx >(_samples / 2)*0.35 && tmpx < (_samples / 2)*0.37) &&
					!(tmpx >(_samples / 2)*0.63 && tmpx < (_samples / 2)*0.65) &&
					!(tmpx >(_samples / 2)*0.495 && tmpx < (_samples / 2)*0.505))
				{
					mask[i] = -1.0;
				}
			}
		}
	}

	//Mask 2
	{
		float64* mask2 = &_output[xSliceLength * 3];
		for (size_t i = 0; i < xSliceLength; ++i)
		{
			mask2[i] = 5.0;
		}

		if (_masks2 == "disable")
		{
			for (size_t i = 0; i < xSliceLength; ++i)
			{
				mask2[i] = -1;
			}
		}
		else if (_masks2 == "adjustment_mask")
		{
			for (size_t i = 0; i < xSliceLength; ++i)
			{
				const double tmpx = std::abs((int)((i + 1) % wavePeriod) - wavePeriod / 2);
				if (!(tmpx <(_samples / 2)*0.1 || tmpx >(_samples / 2)*0.9) &&
					!(tmpx >(_samples / 2)*0.2 && tmpx < (_samples / 2)*0.25) &&
					!(tmpx >(_samples / 2)*0.75 && tmpx < (_samples / 2)*0.8) &&
					!(tmpx >(_samples / 2)*0.35 && tmpx < (_samples / 2)*0.37) &&
					!(tmpx >(_samples / 2)*0.63 && tmpx < (_samples / 2)*0.65) &&
					!(tmpx >(_samples / 2)*0.495 && tmpx < (_samples / 2)*0.505))
				{
					mask2[i] = -1.0;
				}
			}
		}
	}

	//Z+PIFOC
	{
		float64* z = &_output[xSliceLength * 4];
		float64* z2 = &_output[xSliceLength * 5];
		float64* pifoc = &_output[xSliceLength*6];

		for (size_t i = 0; i < xSliceLength; ++i)
		{
			pifoc[i] = zOffset / pifocScale;
		}

		if (_zMirrorScale1.is_object())
		{
			std::vector<std::tuple<double, double, double>> zMirrorScales; //Z, scale, voltage on Z
			auto const& scales = _zMirrorScale1["scales"];
			for (auto it = scales.begin(); it < scales.end(); ++it)
			{
				zMirrorScales.push_back({ (double)(*it)[0], (double)(*it)[1], (double)(*it)[2] });
			}
			auto currentScale = zMirrorScales.begin();
			while (currentScale < zMirrorScales.end() && zOffset > std::get<0>(*currentScale))
			{
				++currentScale;
			}
			if (currentScale == zMirrorScales.end()) { --currentScale; }
			for (size_t i = 0; i < xSliceLength; ++i)
			{
				z[i] = std::get<2>(*currentScale) + (zOffset - std::get<0>(*currentScale)) / std::get<1>(*currentScale);
			}
		}
		else
		{
			double zMirrorScaleLinear = (double)_zMirrorScale2;
			for (size_t i = 0; i < xSliceLength; ++i)
			{
				z[i] = zOffset / zMirrorScaleLinear;
			}
		}


		if (_zMirrorScale2.is_object())
		{
			std::vector<std::tuple<double, double, double>> zMirrorScales; //Z, scale, voltage on Z
			auto const& scales = _zMirrorScale2["scales"];
			for (auto it = scales.begin(); it < scales.end(); ++it)
			{
				zMirrorScales.push_back({ (double)it[0], (double)it[1], (double)it[2] });
			}
			auto currentScale = zMirrorScales.begin();
			while (currentScale < zMirrorScales.end() && zOffset > std::get<0>(*currentScale))
			{
				++currentScale;
			}
			for (size_t i = 0; i < xSliceLength; ++i)
			{
				z2[i] = std::get<2>(*currentScale) + (zOffset - std::get<0>(*currentScale)) * std::get<1>(*currentScale);
			}
		}
		else
		{
			double zMirrorScaleLinear = (double)_zMirrorScale2;
			for (size_t i = 0; i < xSliceLength; ++i)
			{
				z2[i] = zOffset / zMirrorScaleLinear;
			}
		}
	}


	//Camera trigger
	{
		float64* camTrig = &_output[xSliceLength * 7];
		for (size_t i = 0; i < xSliceLength; ++i)
		{
			camTrig[i] = 0.0;
		}
		for (size_t j = 0; j < xSliceLength*0.1; ++j)
		{
			camTrig[j] = 5.0;
		}
	}
}

bool spim::FindPlaneModeController::run(DaqCommander * daq, size_t nVolumes)
{
	//if (daq->isRunning())
	//{
	daq->setFramerateDynamic(_framerate, 400000);

	if (_zMirrorScale1.is_object() || _zMirrorScale2.is_object())
	{
		daq->setOutputDynamic(_output, _lightSheetWidth1, _lightSheetWidth2, _galvoDelay1, _galvoDelay2, _zOffset, _zOffset, 1, 0, spim::DaqCommander::ZMode(1)
			, _xMirrorScale1, _xMirrorScale2, 10000, 10000, _pifocScale);
	}
	else
	{
		daq->setOutputDynamic(_output, _lightSheetWidth1, _lightSheetWidth2, _galvoDelay1, _galvoDelay2, _zOffset, _zOffset, 1, 0, spim::DaqCommander::ZMode(1)
			, _xMirrorScale1, _xMirrorScale2, _zMirrorScale1, _zMirrorScale2, _pifocScale);
	}
	daq->setTriggerOut(true);
	daq->deploy();
	return true;
	//}
	//else
	//{
	//	daq->setRate(_framerate * _outputx.size()/2);
	//	daq->setFrequency(_framerate);
	//	daq->setOutputX(_outputx,_lightSheetWidth,_galvoDelay);
	//	daq->setOutputsZ(_outputsz, _zOffset, _zOffset, 1, 0, spim::DaqCommander::ZMode(1));
	//	daq->execute();// (nVolumes);
	//	return true;
	//}
}
