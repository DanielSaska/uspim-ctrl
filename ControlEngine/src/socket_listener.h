#pragma once
#include <iostream>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <winnetwk.h>
#include <thread>
#include "socket_sender.h"
#include "instruction_handler.h"

#define DEFAULT_BUFLEN 16384

class SocketListener {
protected:
	WSADATA wsaData;
	SOCKET ClientSocket;
	struct addrinfo *result, hints;
	char recvbuf[DEFAULT_BUFLEN];
	int iResult, iSendResult;
	int recvbuflen;
	BOOL listening;
	SocketSender* sender;
	std::thread* thr;

	spim::InstructionHandler * _instructionHandler;
public:
	SocketListener(SOCKET socket);
	void setSender(SocketSender* sender);
	void m_ThreadFunc();
	void start() {
		thr = new std::thread(&SocketListener::m_ThreadFunc,this);
	}

	~SocketListener() {
		if (thr != nullptr) {
			listening = false;
			thr->join();
			delete thr;
		}
		if (_instructionHandler != nullptr) {
			delete _instructionHandler;
			_instructionHandler = nullptr;
		}
	}
protected:
	void HandleMessage(char* xml);
};
