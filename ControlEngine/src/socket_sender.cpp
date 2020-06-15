#include <time.h>
#include <iostream>
#include <string>
#include "socket_sender.h"

SocketSender::SocketSender(SOCKET client) {
	this->clientSocket = client;
}

void SocketSender::sendStatusResponse() {

	char buffer[512];
	buffer[0] = 0;

	time_t rawtime;
	struct tm * timeinfo;

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer, sizeof(buffer), "StatusResponse[%Y-%m-%d %H:%M:%S] The Better Control Engine is online.\n", timeinfo);

	sendXMLToClient(buffer);
}

void SocketSender::sendStatusRunning() {

	char buffer[512];
	buffer[0] = 0;

	time_t rawtime;
	struct tm * timeinfo;

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer, sizeof(buffer), "StatusResponse[%Y-%m-%d %H:%M:%S] The Better Control Engine is still in a middle of task execution. Could not execute your request.\n", timeinfo);

	sendXMLToClient(buffer);
}

void SocketSender::sendStatusCustom(std::string status)
{
	char buffer[512];
	buffer[0] = 0;

	time_t rawtime;
	struct tm * timeinfo;

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	status = "StatusResponse[%Y-%m-%d %H:%M:%S] "+status+"\n";
	strftime(buffer, sizeof(buffer), status.c_str(), timeinfo);

	sendXMLToClient(buffer);
}
void SocketSender::sendCustom(std::string status)
{
	status = status+"\n";

	sendXMLToClient(status.c_str());
}


void SocketSender::sendCalibrationStatus(std::string status)
{
	char buffer[512];
	buffer[0] = 0;

	time_t rawtime;
	struct tm * timeinfo;

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	status = "CalibrationStatus" + status + "\n";
	strftime(buffer, sizeof(buffer), status.c_str(), timeinfo);

	sendXMLToClient(buffer);
}

void SocketSender::sendCalibrationOffset(std::string status)
{
	char buffer[512];
	buffer[0] = 0;

	time_t rawtime;
	struct tm * timeinfo;

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	status = "CalibrationOffset" + status + "\n";
	strftime(buffer, sizeof(buffer), status.c_str(), timeinfo);

	sendXMLToClient(buffer);
}

void SocketSender::sendTaskDone()
{
	sendXMLToClient("TASK_DONE\n");
}

void SocketSender::sendXMLToClient(const char xml[]) {
	std::lock_guard<std::mutex> lock(_lock);
	// Send some XML to the client
	int len = strlen(xml);
	if (len == 0) { return; }
	int sent = send( clientSocket, xml, len, 0 );

	if ( sent == SOCKET_ERROR ) {
		printf( "send failed\n");
		closesocket( clientSocket );
		return;
	}

	//printf( "%i bytes sent to client\n", sent);
	//std::cout << std::string("Message was: " + std::string(xml) + "\n").c_str();
}
