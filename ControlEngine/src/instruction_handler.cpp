#include "instruction_handler.h"
#include "calibration_mode_controller.h"
#include "find_plane_mode_controller.h"
#include "server_socket.h"
#include "socket_sender.h"
#include "spim_daq.h"

#include <fstream>
#include <map>

void spim::InstructionHandler::handleInstruction(std::string instruction)
{
	// If the main execution thread exists, is joinable and the task on said thread has finished
	if (thr != nullptr && thr->joinable() && _slavedone)
	{
		thr->join(); //Join the thread
		delete thr; //and dispose
		thr = nullptr; //and bookkeeping
	}
	if (thr == nullptr) //If the execution thread is ready for (another) task execution
	{
		_slavedone = false; //Execution thread busy with new task now
		thr = new std::thread(&InstructionHandler::handleInstructionSlave, this, instruction);//Send the instruction to the exec. thraed
	}
	else //Task is still running on the main execution thraed
	{
		//Report this over TCP
		serverSocket->sendStatus("The Better Control Engine is still in a middle of task execution. Could not execute your request.");
	}
}

spim::InstructionHandler::InstructionHandler(std::string cfgFile)
	: thr(nullptr)
	, _auxConsumer(nullptr)
	, _slavedone(true)
{
	std::map<std::string, std::string> cfg {
		{"dev", "Dev1"},
		{"xMirror1Channel", "AO0"},
		{"xMirror2Channel", "AO1"},
		{"laser1Channel", "AO2"},
		{"laser2Channel", "AO3"},
		{"zMirror1Channel", "AO4"},
		{"zMirror2Channel", "AO5"},
		{"pifocChannel", "AO6"},
		{"shutterChannel", "AO7"},

		{"counterSource", "Ctr0InternalOutput"},
		{"triggerSource", "PFI0"},
		{"counterChannel", "Ctr0"},
		{"clockSignalChannel", "Ctr1"},
		{"clockSignalTerminal", "PFI1" }
	};

	try {
		std::ifstream cfgF(cfgFile);
		std::string cfgStr((std::istreambuf_iterator<char>(cfgF)), std::istreambuf_iterator<char>());
		json jCfg = json::parse(cfgStr);
		for (auto & it : cfg)
		{
			try {
				it.second = jCfg.at(it.first).get<std::string>();
			}
			catch (std::exception e) {} //Just ignore anything we did not find in the config file
		}
	}
	catch (std::exception e) {
		std::cout << "Error reading config file" << std::endl;
		std::cout << "Press any key to exit...";
		std::cin.ignore();
		std::cin.get();
	}

#if _DEBUG
	std::cout << "DAQ Conifguration:" << std::endl;
	for (auto const& it : cfg)
	{
		std::cout << it.first << "=" << it.second << std::endl;
	}
#endif

	_daqCommander = new DaqCommander( true, 600.0f, false
		, "/" + cfg["dev"] + "/" + cfg["xMirror1Channel"]
		, "/" + cfg["dev"] + "/" + cfg["xMirror2Channel"]
		, "/" + cfg["dev"] + "/" + cfg["laser1Channel"]
		, "/" + cfg["dev"] + "/" + cfg["laser2Channel"]
		, "/" + cfg["dev"] + "/" + cfg["zMirror1Channel"]
		, "/" + cfg["dev"] + "/" + cfg["zMirror2Channel"]
		, "/" + cfg["dev"] + "/" + cfg["pifocChannel"]
		, "/" + cfg["dev"] + "/" + cfg["shutterChannel"]
		, "/" + cfg["dev"] + "/" + cfg["counterSource"]
		, "/" + cfg["dev"] + "/" + cfg["triggerSource"]
		, "/" + cfg["dev"] + "/" + cfg["counterChannel"]
		, "/" + cfg["dev"] + "/" + cfg["clockSignalChannel"]
		, "/" + cfg["dev"] + "/" + cfg["clockSignalTerminal"] );

	_daqCommander->setTriggerOut(false);
	_daqCommander->setRate(1);
	_daqCommander->setFrequency(1,1);
	_daqCommander->setOutput({ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 , 0.0, 0.0 }
		, 0, 0, 0, 0, 0, 0, 0, 0, spim::DaqCommander::ZMode(0));
	_daqCommander->execute();
}

void spim::InstructionHandler::handleAsyncInstruction(std::string instruction)
{
	//Launch a thread to excute the async command on the DAQ
	std::thread(&InstructionConsumer::consumeInstruction, _daqCommander, instruction).detach();
	if (_auxConsumer != nullptr) { //If we have another consumer
		//Launch that as well
		std::thread(&InstructionConsumer::consumeInstruction, _auxConsumer, instruction).detach();
	}
	//Note: one thing I haven't mentioned is that this is probably a really pig way to do multi-threading.
	//Essentially we are creating the thread and letting it run wild without any intention of joining it again
	//So if something goes bad we will slowly start accumulating zombie threads. 
	//Accumulating things we cannot get rid of = BAD BAD BAD thing
	//Oh well ¯\_(ツ)_/¯
}

spim::InstructionHandler::~InstructionHandler()
{
	//Clean up memory, noone ain't gonna do it for us
	if (_auxConsumer != nullptr)
	{
		delete _auxConsumer;
		_auxConsumer = nullptr;
	}
	if (thr != nullptr && thr->joinable()) {
		thr->join();
		delete thr;
	}
	if (_daqCommander != nullptr)
	{
		delete _daqCommander;

	}
}

void spim::InstructionHandler::handleInstructionSlave(std::string instruction)
{
	//Main execution thread instructions are expected to be in json format
	json j = json::parse(instruction); //Load JSON
	std::string const& mode = j["Mode"]; //Determine what mode are executing
	std::cout << mode << std::endl; //Write the mode to the console
	if (mode.compare("NormalMode") == 0) //Normal mode was requested
	{
		std::cout << "Client requested normal mode scan execution." << std::endl;
		json const& normal_mode = j["NormalMode"]; //Settings specific to normal mode
		json const& settings = j["Settings"]; //Global settings
		//Send status with received params over TCP (Client can log this)
		sendTcpStatus("Normal Mode execution was requested with parameters: ");
		sendTcpStatus("Normal Mode: "
			"LightSheetWidth1=" + std::to_string((double)normal_mode["LightSheetWidth1"]) + ", "
			"LightSheetWidth2=" + std::to_string((double)normal_mode["LightSheetWidth2"]) + ", "
			"NumReturn=" + std::to_string((int)normal_mode["NumReturn"]) + ", "
			"ScanMode=" + std::to_string((int)normal_mode["ScanMode"]) + ", "
			"NumPlanes=" + std::to_string((int)normal_mode["NumPlanes"]) + ", "
			"ZOffset=" + std::to_string((double)normal_mode["ZOffset"]) + ", "
			"ZStep=" + std::to_string((double)normal_mode["ZStep"]) + ", "
			"Framerate=" + std::to_string((double)normal_mode["Framerate"]) + ", "
			"GalvoDvelay1=" + std::to_string((double)normal_mode["GalvoDelay1"]) + ", "
			"GalvoDvelay2=" + std::to_string((double)normal_mode["GalvoDelay2"]) + ", "
			"BlankReposition=" + std::to_string((bool)normal_mode["BlankReposition"]) + ", "
			"TriggerOut=" + std::to_string((bool)normal_mode["TriggerOut"]) + ", "
			"Masks1=\"" + normal_mode["Masks1"].get<std::string>() + "\", "
			"Masks2=\"" + normal_mode["Masks2"].get<std::string>() + "\", "
			"NumVolumes=" + std::to_string((int)normal_mode["NumVolumes"]));
		sendTcpStatus("Global: "
			"ScaleXMirror1=" + std::to_string((double)settings["ScaleXMirror1"]) + ", "
			"ScaleXMirror2=" + std::to_string((double)settings["ScaleXMirror2"]) + ", "
			"ScaleZMirror1=" + std::to_string((double)settings["ScaleZMirror1"]) + ", "
			"ScaleZMirror2=" + std::to_string((double)settings["ScaleZMirror2"]) + ", "
			"ScalePifoc=" + std::to_string((double)settings["ScalePifoc"]) + ", "
			"CyclesPerFrame=" + std::to_string((int)settings["CyclesPerFrame"]));

		//Initialize normal mode controller with the received paramters
		spim::NormalModeController * nmc = new spim::NormalModeController
		(normal_mode["LightSheetWidth1"]
			, normal_mode["LightSheetWidth2"]
			, settings["ScaleXMirror1"]
			, settings["ScaleXMirror2"]
			, settings["ScaleZMirror1"]
			, settings["ScaleZMirror2"]
			, settings["ScalePifoc"]
			, normal_mode["NumReturn"]
			, normal_mode["ScanMode"]
			, normal_mode["NumPlanes"]
			, normal_mode["ZStep"]
			, normal_mode["ZOffset"]
			, normal_mode["Framerate"]
			, normal_mode["GalvoDelay1"]
			, normal_mode["GalvoDelay2"]
			, (bool)normal_mode["BlankReposition"]
			, (bool)normal_mode["TriggerOut"]
			, 0
			, settings["CyclesPerFrame"]
			, normal_mode["Masks1"].get<std::string>()
			, normal_mode["Masks2"].get<std::string>());
		//Run for the requseted number of volumes
		bool res = nmc->run(_daqCommander, normal_mode["NumVolumes"]);
		//if (res) { sendTcpStatus("Normal Mode execution finished successfuly. "); } //Report success over TCP
		//else { sendTcpStatus("Normal Mode execution was not fisished properly. Perhaps halted? "); } //Report failure ove TCP
		
		std::thread(&SocketSender::sendTaskDone, serverSocket->getSender()).detach();//Report that we are done over TCP
		delete nmc; //Cleanup
	}
	if (mode.compare("FindPlaneMode") == 0) // Find plane mode was requested
	{
		std::cout << "Client requested find plane mode execution." << std::endl;
		json const& find_plane_mode = j["FindPlaneMode"]; //Settings specific to find plane mode
		json const& settings = j["Settings"]; //Global settings
		std::string scaleZMirror1 = "";
		if (settings["ScaleZMirror1"].is_object())
		{
			std::stringstream ss;
			ss << settings["ScaleZMirror1"];
			scaleZMirror1 = ss.str();
		}
		else
		{
			scaleZMirror1 = std::to_string((double)settings["ScaleZMirror1"]);
		}

		std::string scaleZMirror2 = "";
		if (settings["ScaleZMirror2"].is_object())
		{
			std::stringstream ss;
			ss << settings["ScaleZMirror2"];
			scaleZMirror2 = ss.str();
		}
		else
		{
			scaleZMirror2 = std::to_string((double)settings["ScaleZMirror2"]);
		}

		//Send status with received params over TCP (Client can log this)
		sendTcpStatus("Find Plane Mode execution was requested with parameters: ");
		sendTcpStatus("Find Plane Mode: "
			"LightSheetWidth1=" + std::to_string((double)find_plane_mode["LightSheetWidth1"]) + ", "
			"LightSheetWidth2=" + std::to_string((double)find_plane_mode["LightSheetWidth2"]) + ", "
			"ZOffset=" + std::to_string((double)find_plane_mode["ZOffset"]) + ", "
			"Framerate=" + std::to_string((double)find_plane_mode["Framerate"]) + ", "
			"Masks1=\"" + find_plane_mode["Masks1"].get<std::string>() + "\", "
			"Masks2=\"" + find_plane_mode["Masks2"].get<std::string>() + "\", "
			"GalvoDelay1=" + std::to_string((double)find_plane_mode["GalvoDelay1"]) + ", "
			"GalvoDelay2=" + std::to_string((double)find_plane_mode["GalvoDelay2"]) + ", "
			"NumVolumes=" + std::to_string((double)find_plane_mode["NumVolumes"]));
		sendTcpStatus("Global: "
			"ScaleXMirror1=" + std::to_string((double)settings["ScaleXMirror1"]) + ", "
			"ScaleXMirror2=" + std::to_string((double)settings["ScaleXMirror2"]) + ", "
			"ScaleZMirror1=" + scaleZMirror1 + ", "
			"ScaleZMirror2=" + scaleZMirror2 + ", "
			"ScalePifoc=" + std::to_string((double)settings["ScalePifoc"]) + ", "
			"CyclesPerFrame=" + std::to_string((int)settings["CyclesPerFrame"]));

		//Initialize find plane mode controller with the received paramters
		spim::FindPlaneModeController * fmc = new spim::FindPlaneModeController
		(find_plane_mode["LightSheetWidth1"]
			, find_plane_mode["LightSheetWidth2"]
			, settings["ScaleXMirror1"]
			, settings["ScaleXMirror2"]
			, settings["ScaleZMirror1"]
			, settings["ScaleZMirror2"]
			, settings["ScalePifoc"]
			, find_plane_mode["ZOffset"]
			, find_plane_mode["Framerate"]
			, find_plane_mode["GalvoDelay1"]
			, find_plane_mode["GalvoDelay2"]
			, 0
			, settings["CyclesPerFrame"]
			, find_plane_mode["Masks1"].get<std::string>()
			, find_plane_mode["Masks2"].get<std::string>());
		//Run for the requseted number of volumes
		bool res = fmc->run(_daqCommander, find_plane_mode["NumVolumes"]);
		//if (res) { sendTcpStatus("Find Plane Mode execution finished successfuly. "); } //Report success over TCP
		//else { sendTcpStatus("Find Plane Mode execution was not fisished properly. Perhaps halted? "); } //Report failure ove TCP
		std::thread(&SocketSender::sendTaskDone, serverSocket->getSender()).detach();//Report that we are done over TCP
		delete fmc; //Cleanup
	}
	else if (mode.compare("CalibrationMode") == 0)
	{
		std::cout << "Client requested calibration mode execution." << std::endl;
		json const& calib_mode = j["CalibrationMode"]; //Settings specific to calibration mode
		json const& settings = j["Settings"]; //Global settings
		//Send status with received params over TCP (Client can log this)
		sendTcpStatus("Calibration Mode execution was requested with parameters: ");
		sendTcpStatus("Calibration Mode: "
			"LightSheetWidth1=" + std::to_string((double)calib_mode["LightSheetWidth1"]) + ", "
			"LightSheetWidth2=" + std::to_string((double)calib_mode["LightSheetWidth2"]) + ", "
			"NumCalibrationSamples=" + std::to_string((double)calib_mode["NumCalibrationSamples"]) + ", "
			"LocationStart=" + std::to_string((double)calib_mode["LocationStart"]) + ", "
			"LocationEnd=" + std::to_string((double)calib_mode["LocationEnd"]) + ", "
			"Framerate=" + std::to_string((double)calib_mode["Framerate"]));
		sendTcpStatus("Global: "
			"ScaleXMirror1=" + std::to_string((double)settings["ScaleXMirror1"]) + ", "
			"ScaleXMirror2=" + std::to_string((double)settings["ScaleXMirror2"]) + ", "
			"ScalePifoc=" + std::to_string((double)settings["ScalePifoc"]) + ", "
			"CyclesPerFrame=" + std::to_string((int)settings["CyclesPerFrame"]));

		//Initialize calibration mode controller with the received paramters
		spim::CalibraitonModeController * cmc = new spim::CalibraitonModeController
		(calib_mode["LightSheetWidth1"]
			, calib_mode["LightSheetWidth2"]
			, settings["ScaleXMirror1"]
			, settings["ScaleXMirror2"]
			, settings["ScalePifoc"]
			, calib_mode["NumCalibrationSamples"]
			, calib_mode["LocationStart"]
			, calib_mode["LocationEnd"]
			, calib_mode["Framerate"]
			, 0
			, settings["CyclesPerFrame"]
			, calib_mode["Masks1"].get<std::string>()
			, calib_mode["Masks2"].get<std::string>());
		_auxConsumer = cmc; //Register the calibration controller to receive async instructions
		//Run the calibration mode
		bool res = cmc->run(_daqCommander);
		//if (res) { sendTcpStatus("Calibration Mode execution finished successfuly. "); } //Report success over TCP
		//else { sendTcpStatus("Calibration Mode execution was not fisished properly. Perhaps halted? "); } //Report failure ove TCP
		std::thread(&SocketSender::sendTaskDone, serverSocket->getSender()).detach();//Report that we are done over TCP
		_auxConsumer = nullptr; //Bookkeeping
		delete cmc; //Cleanup
	}
	_slavedone = true; //The task has finsihed
}
