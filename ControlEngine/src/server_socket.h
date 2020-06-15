#pragma once
#include <winsock2.h>
#include <ws2tcpip.h>
#include <winnetwk.h>
#include <mutex>

#include <thread>
//#include "socket_listener.h"
//#include "socket_sender.h"

#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT_STR "51337"
class SocketListener;
class SocketSender;
class ServerSocket {
protected:
	WSADATA wsaData;
	SOCKET ListenSocket, ClientSocket;
	struct addrinfo *result, hints;
	char recvbuf[DEFAULT_BUFLEN];
	int iResult, iSendResult;
	int recvbuflen;
	BOOL listening;
	SocketSender* sender;
	SocketListener* listener;
	std::thread thr;

public:
	ServerSocket();

	bool sendStatus(std::string status);
	bool sendCustom(std::string msg);

	SocketSender* getSender();
	SocketListener* getListener();
	HANDLE getThread();
	void m_ThreadFunc();
	void start() {
		thr = std::thread(&ServerSocket::m_ThreadFunc, this);
	}
	void join() {
		thr.join();
	}
	bool joinable() {
		return thr.joinable();
	}

	std::mutex _lock;
};


extern ServerSocket * serverSocket;
