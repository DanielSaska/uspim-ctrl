#include "socket_listener.h"
#include "instruction_handler.h"

SocketListener::SocketListener(SOCKET socket, std::string cfgFile)
	: thr( nullptr )
	, _instructionHandler( new spim::InstructionHandler(cfgFile) )
{
	recvbuflen = DEFAULT_BUFLEN;
	this->ClientSocket = socket;
}



void SocketListener::m_ThreadFunc() {
	listening = true;
	while ( listening ) {
		int iResult = recv(ClientSocket, recvbuf, recvbuflen, 0);
		if (iResult > 0) {
			printf("Bytes received: %d\n", iResult);
			recvbuf[iResult-1] = 0; // null terminate the string according to the length
			
			// The message spec indicates the XML will end in a "new line"
			// character which can be either a newline or caraiage feed
			// Search for either and replace with a NULL to terminate
			if ( recvbuf[iResult-2] == '\n' || recvbuf[iResult-2] == '\r' )
				recvbuf[iResult-2] = 0;

			HandleMessage(recvbuf);
		}
		else {
			printf("Client connection closing\n");
			closesocket(ClientSocket);
			return;
			//return 1;
		}
	}

	//return 0;
}

void SocketListener::setSender(SocketSender* sender) {
	this->sender = sender;
}

char* strstri(char* t, char* s) {
	int i, j;

	for ( i = 0; t[i] != '\0'; i++ ) {
		for ( j = 0; s[j] != '\0'; j++ ) {
			if ( toupper(s[j])==toupper(t[i+j]) ) 
				continue; 
			else 
				break;
		}

		// if the whole string was successfully compared, exit the loop
		if (s [j] == '\0' ) 
			break;
	}
	
	if ( s[j] == '\0' ) {
		// returns a pointer to the first occurrence 
		// of s within t, just like the original strstr
		return ( i + t ); 
					  
	}

	// Not found
	return 0; 
}

void SocketListener::HandleMessage(char* xml) {


	if (strstri(xml, "QueryStatus") != 0) {
		printf("Client sent Status request message\n");
		if (sender != NULL)
		{
			sender->sendStatusResponse();
		}
	}
	else if (strstri(xml, "ASYNC") != 0) {
		_instructionHandler->handleAsyncInstruction(std::string(xml).erase(0,5));
	}
	else {
		_instructionHandler->handleInstruction(xml);
	}
}
