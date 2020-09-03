#include "find_plane_mode_controller.h"
#include "normal_mode_controller.h"
#include "server_socket.h"
#include "instruction_handler.h"
#include "spim_daq.h"
#include "json.hpp"

#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <conio.h>
#include <NIDAQmx.h>
#include <time.h>

#include <iostream>
#include <fstream>

#include <vector>
#include <string>
#include <windows.h>

#include <cstdlib> 
#include <fstream>

using json = nlohmann::json;

ServerSocket * serverSocket = nullptr;

inline bool fexists(const std::string& name) {
	std::ifstream f(name.c_str());
	return f.good();
}

int main(int argc, char* argv[])
{
	std::string cfgDefault = "{\n\t\"dev\": \"Dev1\",\n\n\t\"xMirror1Channel\": \"AO0\",\n\t\"xMirror2Channel\": \"AO1\",\n\t\"laser1Channel\": \"AO2\",\n\t\"laser2Channel\": \"AO3\",\n\t\"zMirror1Channel\": \"AO4\",\n\t\"zMirror2Channel\": \"AO5\",\n\t\"pifocChannel\": \"AO6\",\n\t\"shutterChannel\": \"AO7\",\n\n\t\"counterSource\": \"Ctr0InternalOutput\",\n\t\"triggerSource\": \"PFI0\",\n\t\"counterChannel\": \"Ctr0\",\n\t\"clockSignalChannel\": \"Ctr1\",\n\t\"clockSignalTerminal\": \"PFI1\"\n}";
	std::string cfgFile = "./cfg.json";
	if (!fexists(cfgFile)) {
		std::ofstream out(cfgFile);
		out << cfgDefault;
		out.close();
	}


	// Start server socket listener
	serverSocket = new ServerSocket(cfgFile);  //Create TCP server
	serverSocket->start(); //Start the server
	serverSocket->join(); //Wait for it to finish (never)
	delete serverSocket; //And cleanup
	serverSocket = nullptr; //Bookkeeping


	// If you want to customize The better contorl engine to make it even better (and standalone), you can call Controllers and 
	// DAQ functions directly as hinted by commented code below.

	//spim::DaqCommander * daqCommander = new spim::DaqCommander(true);
	//spim::NormalModeController * nmc = new spim::NormalModeController
	//(500
	//	, 195.5
	//	, -227.205
	//	, 40
	//	, 10
	//	, spim::NormalModeController::ZMode::bidirectional
	//	, 3
	//	, 0
	//	, 100, 998, 3);
	//nmc->run(daqCommander, 10000000);

	//spim::DaqCommander * daqCommander = new spim::DaqCommander(true);
	//spim::FindPlaneModeController * nmc = new spim::FindPlaneModeController
	//(500
	//	, 195.5
	//	, -227.205
	//	, 40
	//	, 10
	//	, 99.92546912378135, 998);
	//nmc->run(daqCommander, 10000000);
	//spim::LightSheetModeController * lsmc = new spim::LightSheetModeController
	//	( 1
	//	, 40
	//	, 195.5
	//	, -227.205
	//	, 10
	//	, spim::LightSheetModeController::ZMode::acquiredown
	//	, 50
	//	, 2, 48, 1000, 0 );
	//lsmc->run(daqCommander, 100000);
	//spim::NormalModeController * nmc = new spim::NormalModeController
	//	( 500
	//	, 195.5
	//	, -227.205
	//	, 40
	//	, 10 
	//	, spim::NormalModeController::ZMode::acquiredown
	//	, 50
	//	, 2, 98.5, 1000);
	//nmc->run(daqCommander,10000000);
	//spim::ResetModeController * rmc = new spim::ResetModeController();
	//rmc->run(daqCommander);
	//spim::ZeroPlaneModeController * zpmc = new spim::ZeroPlaneModeController
	//	(500, 195.5, 200, 1000, 3);
	//zpmc->run(daqCommander,10000);
	//delete daqCommander;
	//delete rmc;
	//delete nmc;
	//delete zpmc;
	//delete lsmc;
}
