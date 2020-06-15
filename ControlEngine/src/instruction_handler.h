#pragma once
#include <iostream>
#include <atomic>
#include "normal_mode_controller.h"
#include "spim_daq.h"
#include "instruction_consumer.h"
#include "json.hpp"
using json = nlohmann::json;

namespace spim
{
	/// Class to handle text and json based instructions usually coming over TCP
	class InstructionHandler
	{
	public:
		///Setup
		InstructionHandler();

		///Handles instructions that are to be executed on the main execution thraed
		void handleInstruction(std::string instruction); 

		///Handles instructions that re to be run asynchronously such that they are not blocked by other instructions,
		///in particular the ones that run on the main execution thread
		void handleAsyncInstruction(std::string instruction);

		///Clearn up
		~InstructionHandler();
	private:
		///Slave running on the main execution thread
		void handleInstructionSlave(std::string instruction);

		///THREADSAFE proxy for reporting stutus over TCP
		void sendTcpStatus(std::string status) 
		{ if (serverSocket != nullptr) { std::thread(&ServerSocket::sendStatus, serverSocket,status).detach(); } }

		///Uusally used for the task that runs on the main execution thread if it is able to consume instructions
		///Such as clibration mode, accepting voltage change instructions
		InstructionConsumer * _auxConsumer;

		std::thread * thr; ///Main execution thread
		spim::DaqCommander * _daqCommander; ///DAQACommander isntance
		std::atomic<bool> _slavedone; ///Atomic value indicating that the task running on main execution thread has been finished
	};
}