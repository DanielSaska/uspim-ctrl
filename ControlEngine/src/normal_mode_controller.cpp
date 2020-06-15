#include <iostream>
#include <cmath>
#include <fstream>
#include "spim_daq.h"
#include "normal_mode_controller.h"
#include "string_util.h"

float zTransform(std::vector<std::tuple<double, double, double>> const& zMirrorScales, float zOffset)
{
	auto currentScale = zMirrorScales.begin();
	while (currentScale < zMirrorScales.end() && zOffset > std::get<0>(*currentScale))
	{
		++currentScale;
	}
	if (currentScale == zMirrorScales.end()) { --currentScale; }
	return std::get<2>(*currentScale) + (zOffset - std::get<0>(*currentScale)) / std::get<1>(*currentScale);
}

spim::NormalModeController::NormalModeController
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
  , double zOffset
  , double framerate
  , double galvoDelay1
  , double galvoDelay2
  , double blankReposition
  , double triggerOut
  , size_t samples
  , size_t cyclesPerFrame
  , std::string masks1
  , std::string masks2)
	: _lightSheetWidth1( lightSheetWidth1 )
	, _lightSheetWidth2(lightSheetWidth2)
	, _samples((samples == 0) ? (400000 / framerate / cyclesPerFrame) : samples)
	, _framerate(framerate)
	, _galvoDelay1(galvoDelay1)
	, _galvoDelay2(galvoDelay2)
	, _cyclesPerFrame(cyclesPerFrame)
	, _pifocScale( pifocScale )
	, _xMirrorScale1(xMirrorScale1)
	, _xMirrorScale2(xMirrorScale2)
	, _zMirrorScale1(zMirrorScale1)
	, _zMirrorScale2(zMirrorScale2)
	, _nReturnSlices( nReturnSlices )
	, _zMode( zMode )
	, _nSlices( nSlices )
	, _zStep( zStep )
	, _zOffset( zOffset )
	, _blankReposition(blankReposition)
	, _triggerOut(triggerOut)
	, _output((_samples * cyclesPerFrame)*nSlices*8, 0.0 )
	, _masks1(masks1)
	, _masks2(masks2)
{
	if (_nReturnSlices == 0) { _nReturnSlices = 1; _zStep = 1; } //Hotfix for plane recording
	const int xSliceLength = (_samples * _cyclesPerFrame);
	const int xSignalLength = xSliceLength * _nSlices;
	//X Mirror + Laser
	{
		const int xSliceLength = (_samples * _cyclesPerFrame);
		const int xSignalLength = xSliceLength * _nSlices;
		const int wavePeriod = int(_samples);

		//X MIRROR 1
		{
			const int gDelay1 = _galvoDelay1 / (1000.0 / double(framerate * _samples));
			float64* xmirror = &_output[0];
			for (size_t j = 0; j < _nSlices; ++j)
			{
				for (size_t i = 0; i < _samples * _cyclesPerFrame; ++i)
				{
					double pos = ((int)i + 1 - gDelay1) % wavePeriod;
					pos = pos < 0.0 ? pos + wavePeriod : pos;
					xmirror[j * xSliceLength + i] = _lightSheetWidth1 / _xMirrorScale1 / 2.0 * (std::abs(pos - wavePeriod / 2) / (double)wavePeriod * 4 - 1);
				}
			}
		}
		//X MIRROR 2
		{
			const int gDelay2 = _galvoDelay2 / (1000.0 / double(framerate * _samples));
			float64* xmirror2 = &_output[xSignalLength];
			for (size_t j = 0; j < _nSlices; ++j)
			{
				for (size_t i = 0; i < _samples * _cyclesPerFrame; ++i)
				{
					double pos = ((int)i + 1 - gDelay2) % wavePeriod;
					pos = pos < 0.0 ? pos + wavePeriod : pos;
					xmirror2[j * xSliceLength + i] = _lightSheetWidth2 / _xMirrorScale2 / 2.0 * (std::abs(pos - wavePeriod / 2) / (double)wavePeriod * 4 - 1);
				}
			}
		}
		/*
		for (size_t i = 0; i < _samples * cyclesPerFrame - 1; ++i)
		{
			_outputx[channelOffset+i] = _lightSheetWidth / _xMirrorScale / 2.0 * (std::abs((int)((i + 1) % wavePeriod) - wavePeriod / 2) / (double)wavePeriod * 4 - 1); //WTF is that -1?
		}
		*/
		//Masks
		{
			float64* mask = &_output[xSignalLength*2];
			for (size_t i = 0; i < xSignalLength; ++i)
			{ mask[i] = 5.0; }
			auto maskprots = split(masks1,';');

			for (auto const& maskprot : maskprots) 
			{
				auto maskparams = split(maskprot, ',');
				if (maskparams[0] == "disable")
				{
					for (size_t j = 0; j < _nSlices; ++j)
					{
						for (size_t i = 0; i < _samples * _cyclesPerFrame; ++i)
						{
							mask[j * xSliceLength + i] = -1.0;
						}
					}
				}
				else if (maskparams[0] == "adjustment_mask")
				{
					for (size_t j = 0; j < _nSlices; ++j)
					{
						for (size_t i = 0; i < _samples * _cyclesPerFrame; ++i)
						{
							const double tmpx = std::abs((int)((i + 1) % wavePeriod) - wavePeriod / 2);
							if (!(tmpx <(_samples / 2)*0.1 || tmpx >(_samples / 2)*0.9) &&
								!(tmpx >(_samples / 2)*0.2 && tmpx < (_samples / 2)*0.25) &&
								!(tmpx >(_samples / 2)*0.75 && tmpx < (_samples / 2)*0.8) &&
								!(tmpx >(_samples / 2)*0.35 && tmpx < (_samples / 2)*0.37) &&
								!(tmpx >(_samples / 2)*0.63 && tmpx < (_samples / 2)*0.65) &&
								!(tmpx >(_samples / 2)*0.4975 && tmpx < (_samples / 2)*0.5025))
							{
								mask[j * xSliceLength + i] = -1.0;
							}
						}
					}
				}
				else if (maskparams[0] == "left_end_cut")
				{
					for (size_t j = 0; j < _nSlices; ++j)
					{
						for (size_t i = 0; i < _samples * _cyclesPerFrame; ++i)
						{
							const double tmpx = std::abs((int)((i + 1) % wavePeriod) - wavePeriod / 2);
							if (tmpx < (_samples / 2)*(std::stod(maskparams[1])/100.0))
							{
								mask[j * xSliceLength + i] = -1.0;
							}
						}
					}
				}
				else if (maskparams[0] == "right_end_cut")
				{
					for (size_t j = 0; j < _nSlices; ++j)
					{
						for (size_t i = 0; i < _samples * _cyclesPerFrame; ++i)
						{
							const double tmpx = std::abs((int)((i + 1) % wavePeriod) - wavePeriod / 2);
							if (tmpx > (_samples / 2)*(1.0 - (std::stod(maskparams[1]) / 100.0)))
							{
								mask[j * xSliceLength + i] = -1.0;
							}
						}
					}
				}
				else if (maskparams[0] == "eye_mask")
				{
					double const& cx = stod(maskparams[1]);
					double const& cz = stod(maskparams[2]);
					double const& cr = stod(maskparams[3]);
					double appliedZOffset = _zOffset;
					double appliedZStep = _zStep;
					if (maskparams.size() > 4) 
					{ appliedZOffset = stod(maskparams[4]); }
					if (maskparams.size() > 5) 
					{ appliedZStep = stod(maskparams[5]); }


					for (size_t j = 0; j < _nSlices; ++j)
					{
						double pz = 0.0; appliedZOffset + j * appliedZStep;
						//if (_nSlices / 2 <= 1) { break; }
						if (_zMode == ZMode::bidirectional)
						{
							const double incer = ((j < _nSlices / 2) ? j : (_nSlices / 2 - j)) / (double)(_nSlices / 2);
							pz = (zStep / 2 * _nSlices) * incer + _zOffset;
						}
						else if (_zMode == ZMode::acquiredown)
						{
							double incer = j;
							if (j >= _nSlices - nReturnSlices) // Moving up (reset phase).
							{ incer = j * (_nSlices - j) / double(nReturnSlices); }
							pz = (zStep) * incer + _zOffset;
						}
						else if (_zMode == ZMode::acquireup)
						{
							double incer = 0;
							if (j >= nReturnSlices)
							{ incer = _nSlices - j; }
							else // Moving down (reset phase).
							{ incer = j * (_nSlices) / double(nReturnSlices); }
							pz = (zStep) * incer + _zOffset;
						}
						for (size_t i = 0; i < _samples * _cyclesPerFrame; ++i)
						{
							double pos = ((int)i + 1) % wavePeriod;
							pos = pos < 0.0 ? pos + wavePeriod : pos;
							const double px = _lightSheetWidth1 / 2.0 * (std::abs(pos - wavePeriod / 2) / (double)wavePeriod * 4 - 1); //WTF is that -1?
							//if (pow(px - cx, 2) + pow(pz - cz, 2) < pow(cr, 2))
							if (std::abs(px - cx) < cr)
							{
								mask[j * xSliceLength + i] = 0;
							}
						}
					}
				}
				else if (maskparams[0] == "blank_return")
				{
					for (size_t j = 0; j < _nSlices; ++j)
					{
						if ((_zMode == ZMode::acquiredown && j >= _nSlices - _nReturnSlices)
							|| (_zMode == ZMode::acquireup && !(j >= nReturnSlices)))
						{
							for (size_t i = 0; i < _samples * _cyclesPerFrame; ++i)
							{
								mask[j * xSliceLength + i] = 0;
							}
						}
					}
				}
			}
		}

		//Mask 2
		{
			float64* mask2 = &_output[xSignalLength * 3];
			for (size_t i = 0; i < xSignalLength; ++i)
			{
				mask2[i] = 5.0;
			}
			auto maskprots = split(masks2, ';');

			for (auto const& maskprot : maskprots)
			{
				auto maskparams = split(maskprot, ',');

				if (maskparams[0] == "disable")
				{
					for (size_t j = 0; j < _nSlices; ++j)
					{
						for (size_t i = 0; i < _samples * _cyclesPerFrame; ++i)
						{
							mask2[j * xSliceLength + i] = -1.0;
						}
					}
				}
				else if (maskparams[0] == "adjustment_mask")
				{
					for (size_t j = 0; j < _nSlices; ++j)
					{
						for (size_t i = 0; i < _samples * _cyclesPerFrame; ++i)
						{
							const double tmpx = std::abs((int)((i + 1) % wavePeriod) - wavePeriod / 2);
							if (!(tmpx <(_samples / 2)*0.1 || tmpx > (_samples / 2)*0.9) &&
								!(tmpx >(_samples / 2)*0.2 && tmpx < (_samples / 2)*0.25) &&
								!(tmpx >(_samples / 2)*0.75 && tmpx < (_samples / 2)*0.8) &&
								!(tmpx >(_samples / 2)*0.35 && tmpx < (_samples / 2)*0.37) &&
								!(tmpx >(_samples / 2)*0.63 && tmpx < (_samples / 2)*0.65) &&
								!(tmpx >(_samples / 2)*0.495 && tmpx < (_samples / 2)*0.505))
							{
								mask2[j * xSliceLength + i] = -1.0;
							}
						}
					}
				}
				else if (maskparams[0] == "left_end_cut")
				{
					for (size_t j = 0; j < _nSlices; ++j)
					{
						for (size_t i = 0; i < _samples * _cyclesPerFrame; ++i)
						{
							const double tmpx = std::abs((int)((i + 1) % wavePeriod) - wavePeriod / 2);
							if (tmpx < (_samples / 2)*(std::stod(maskparams[1]) / 100.0))
							{
								mask2[j * xSliceLength + i] = -1.0;
							}
						}
					}
				}
				else if (maskparams[0] == "right_end_cut")
				{
					for (size_t j = 0; j < _nSlices; ++j)
					{
						for (size_t i = 0; i < _samples * _cyclesPerFrame; ++i)
						{
							const double tmpx = std::abs((int)((i + 1) % wavePeriod) - wavePeriod / 2);
							if (tmpx >(_samples / 2)*(1.0 - (std::stod(maskparams[1]) / 100.0)))
							{
								mask2[j * xSliceLength + i] = -1.0;
							}
						}
					}
				}
				else if (maskparams[0] == "eye_mask")
				{
					double const& cx = stod(maskparams[1]);
					double const& cz = stod(maskparams[2]);
					double const& cr = stod(maskparams[3]);
					double appliedZOffset = _zOffset;
					double appliedZStep = _zStep;
					if (maskparams.size() > 4)
					{
						appliedZOffset = stod(maskparams[4]);
					}
					if (maskparams.size() > 5)
					{
						appliedZStep = stod(maskparams[5]);
					}


					for (size_t j = 0; j < _nSlices; ++j)
					{
						double pz = 0.0; appliedZOffset + j * appliedZStep;
						//if (_nSlices / 2 <= 1) { break; }
						if (_zMode == ZMode::bidirectional)
						{
							const double incer = ((j < _nSlices / 2) ? j : (_nSlices / 2 - j)) / (double)(_nSlices / 2);
							pz = (zStep / 2 * _nSlices) * incer + _zOffset;
						}
						else if (_zMode == ZMode::acquiredown)
						{
							double incer = j;
							if (j >= _nSlices - nReturnSlices) // Moving up (reset phase).
							{
								incer = j * (_nSlices - j) / double(nReturnSlices);
							}
							pz = (zStep)* incer + _zOffset;
						}
						else if (_zMode == ZMode::acquireup)
						{
							double incer = 0;
							if (j >= nReturnSlices)
							{
								incer = _nSlices - j;
							}
							else // Moving down (reset phase).
							{
								incer = j * (_nSlices) / double(nReturnSlices);
							}
							pz = (zStep)* incer + _zOffset;
						}
						for (size_t i = 0; i < _samples * _cyclesPerFrame; ++i)
						{
							double pos = ((int)i + 1) % wavePeriod;
							pos = pos < 0.0 ? pos + wavePeriod : pos;
							const double px = _lightSheetWidth2 / 2.0 * (std::abs(pos - wavePeriod / 2) / (double)wavePeriod * 4 - 1); //WTF is that -1?
																																	  //if (pow(px - cx, 2) + pow(pz - cz, 2) < pow(cr, 2))
							if (std::abs(px - cx) < cr)
							{
								mask2[j * xSliceLength + i] = 0;
							}
						}
					}
				}
				else if (maskparams[0] == "blank_return")
				{
					for (size_t j = 0; j < _nSlices; ++j)
					{
						if ((_zMode == ZMode::acquiredown && j >= _nSlices - _nReturnSlices)
							|| (_zMode == ZMode::acquireup && !(j >= nReturnSlices)))
						{
							for (size_t i = 0; i < _samples * _cyclesPerFrame; ++i)
							{
								mask2[j * xSliceLength + i] = 0;
							}
						}
					}
				}
			}
		}
	}

	//Z Mirror
	{
		float64* z = &_output[xSignalLength * 4];
		float64* z2 = &_output[xSignalLength * 5];
		float64* pifoc = &_output[xSignalLength * 6];
		int debugCounter = 0;

		std::vector<std::tuple<double, double, double>> zMirrorScales1; //Z, scale, voltage on Z
		if (_zMirrorScale1.is_object())
		{
			auto const& scales = _zMirrorScale1["scales"];
			for (auto it = scales.begin(); it < scales.end(); ++it)
			{
				zMirrorScales1.push_back({ (double)(*it)[0], (double)(*it)[1], (double)(*it)[2] });
			}
		}
		std::vector<std::tuple<double, double, double>> zMirrorScales2; //Z, scale, voltage on Z
		if (_zMirrorScale2.is_object())
		{
			auto const& scales = _zMirrorScale2["scales"];
			for (auto it = scales.begin(); it < scales.end(); ++it)
			{
				zMirrorScales2.push_back({ (double)(*it)[0], (double)(*it)[1], (double)(*it)[2] });
			}
		}

		//Create slice offsets
		for (int i = 0; i < _nSlices; ++i)
		{
			for (int j = 0; j < xSliceLength; ++j)
			{
				if (_nSlices / 2 < 0) { break; }

				if (_zMode == ZMode::bidirectional)
				{
					//Moving down : Moving up
					const double incer = ((i < _nSlices / 2) ? i : (_nSlices - i)) / double(_nSlices / 2);

					debugCounter++;

					pifoc[i*xSliceLength+j] = (zStep / 2 * _nSlices / _pifocScale) * incer + _zOffset / _pifocScale;
					
					float zPos = _zOffset + zStep / 2 * _nSlices * incer;
					if (_zMirrorScale1.is_object()) { z[i * xSliceLength + j] = zTransform(zMirrorScales1, zPos); }
					else { z[i * xSliceLength + j] = zPos / _zMirrorScale1; }
					if (_zMirrorScale2.is_object()) { z2[i * xSliceLength + j] = zTransform(zMirrorScales2, zPos); }
					else { z2[i * xSliceLength + j] = zPos / _zMirrorScale2; }
				}
				else if (_zMode == ZMode::acquiredown)
				{
					double incer = i;
					if (i >= _nSlices - nReturnSlices) // Moving up (reset phase).
					{
						incer = i * (_nSlices - i) / double(nReturnSlices);
					}
					pifoc[i*xSliceLength + j] = (zStep / _pifocScale) * incer + _zOffset / _pifocScale;

					float zPos = zStep * incer + _zOffset;
					if (_zMirrorScale1.is_object()) { z[i * xSliceLength + j] = zTransform(zMirrorScales1, zPos); }
					else { z[i * xSliceLength + j] = zPos / _zMirrorScale1; }
					if (_zMirrorScale2.is_object()) { z2[i * xSliceLength + j] = zTransform(zMirrorScales2, zPos); }
					else { z2[i * xSliceLength + j] = zPos / _zMirrorScale2; }
				}
				else if (_zMode == ZMode::acquireup)
				{
					int const& tmpi = _nSlices - i - 1;
					double incer = tmpi;
					if (tmpi >= _nSlices - nReturnSlices) // Moving up (reset phase).
					{
						incer = tmpi * (_nSlices - tmpi) / double(nReturnSlices);
					}
					pifoc[i*xSliceLength + j] = (zStep / _pifocScale) * incer + _zOffset / _pifocScale;

					float zPos = zStep * incer + _zOffset;
					if (_zMirrorScale1.is_object()) { z[i * xSliceLength + j] = zTransform(zMirrorScales1, zPos); }
					else { z[i * xSliceLength + j] = zPos / _zMirrorScale1; }
					if (_zMirrorScale2.is_object()) { z2[i * xSliceLength + j] = zTransform(zMirrorScales2, zPos); }
					else { z2[i * xSliceLength + j] = zPos / _zMirrorScale2; }
				}

			}
		}
	}

	//Camera trigger
	{
		float64* camTrig = &_output[xSignalLength * 7];
		for (size_t i = 0; i < xSignalLength; ++i)
		{ camTrig[i] = 0.0; }
		if (_triggerOut) 
		{
			for (size_t j = 0; j < _nSlices; ++j)
			{
				for (size_t i = 0; i < (_samples * _cyclesPerFrame)*0.1; ++i)
				{
					camTrig[j * xSliceLength + i] = 5.0;
				}
			}
		}
	}
}

bool spim::NormalModeController::run(DaqCommander * daq, size_t nVolumes)
{
	/*
	std::ofstream fl("control.csv");
	for (auto const& spl : _output) 
	{
		fl << spl << "\n";
	}
	fl.close();*/
	//if (daq->isRunning())
	//{
	const int xSliceLength = (_samples * _cyclesPerFrame);
	daq->setFramerateDynamic(_framerate, 400000);
	if (_zMode == ZMode::acquireup)
	{
		daq->setOutputDynamic
			(_output, _lightSheetWidth1, _lightSheetWidth2, _galvoDelay1, _galvoDelay2
			, _zOffset, _zOffset + _zStep * (_nSlices - _nReturnSlices)
			, _nReturnSlices, _nSlices - _nReturnSlices
			, spim::DaqCommander::ZMode(_zMode)
			, _xMirrorScale1, _xMirrorScale2, _zMirrorScale1, _zMirrorScale2, _pifocScale);
	}
	else
	{
		daq->setOutputDynamic
			( _output, _lightSheetWidth1, _lightSheetWidth2, _galvoDelay1, _galvoDelay2
			, _zOffset, _zOffset + _zStep * (_nSlices - _nReturnSlices)
			, _nSlices - _nReturnSlices, _nReturnSlices
			, spim::DaqCommander::ZMode(_zMode)
			, _xMirrorScale1, _xMirrorScale2, _zMirrorScale1, _zMirrorScale2, _pifocScale);
	}
	daq->setTriggerOut(_triggerOut);
	daq->setBlankReposition(_blankReposition);
	daq->deploy();
	return true;
	//}
	//else
	//{
	//	daq->setRate(_framerate * _outputx.size() / 2);
	//	daq->setFrequency(_framerate);
	//	daq->setOutputX(_outputx,_lightSheetWidth,_galvoDelay);
	//	if (_zMode == ZMode::acquireup)
	//	{
	//		daq->setOutputsZ
	//		(_outputsz
	//			, _zOffset, _zOffset + _zStep * (_nSlices - _nReturnSlices)
	//			, _nReturnSlices, _nSlices - _nReturnSlices
	//			, spim::DaqCommander::ZMode(_zMode));
	//	}
	//	else
	//	{
	//		daq->setOutputsZ
	//		(_outputsz
	//			, _zOffset, _zOffset + _zStep * (_nSlices - _nReturnSlices)
	//			, _nSlices - _nReturnSlices, _nReturnSlices
	//			, spim::DaqCommander::ZMode(_zMode));
	//	}
	//	daq->execute();// (nVolumes * _nSlices);
	//	return true;
	//}
}
