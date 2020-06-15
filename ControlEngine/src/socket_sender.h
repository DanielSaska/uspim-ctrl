#pragma once
#include <iostream>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <winnetwk.h>
#include <mutex>
#include <string>

#define DEFAULT_BUFLEN 512


class SocketSender {
protected:
	WSADATA wsaData;
	SOCKET clientSocket;
	struct addrinfo *result, hints;
	char recvbuf[DEFAULT_BUFLEN];
	int iResult, iSendResult;
	int recvbuflen;
public:
	SocketSender(SOCKET client);

	void sendStatusResponse();
	void sendStatusRunning();
	void sendStatusCustom(std::string status);
	void sendCustom(std::string status);
	void sendCalibrationStatus(std::string status);
	void sendCalibrationOffset(std::string status);
	void sendTaskDone();

protected:
	void sendXMLToClient(const char xml[]);
	std::mutex _lock;
};
