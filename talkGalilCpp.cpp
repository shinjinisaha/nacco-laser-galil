#include <iostream>
#include <sstream>
#include <iomanip>
#include <cstring>      
#include <sys/socket.h> 
#include <netdb.h>      
#include "math.h"
#include <stdlib.h>

using namespace std;
int    status;
struct addrinfo host_info;       
struct addrinfo *host_info_list; 
int    socketfd ;

string getResponseForMsg(string msg){
    send(socketfd, msg.c_str() , msg.length(), 0);
    char recvbuf[512];
    ssize_t bytes_received = recv(socketfd, recvbuf,512, 0);
    string receivedString;                        
    receivedString.assign(recvbuf,bytes_received); 
    char ch = *receivedString.rbegin();
    cout << "Last character is " << ch << endl;
    if (bytes_received == 0) {
        cout << "host shut down." << endl ;
    }
    if (bytes_received == -1)  {
        cout << "receive error!" << endl ;
    }
    return receivedString;
}

int main()
{
    memset(&host_info, 0, sizeof host_info);
    host_info.ai_family   = AF_UNSPEC;     
    host_info.ai_socktype = SOCK_STREAM; 
    status = getaddrinfo("10.0.0.7", "1001", &host_info, &host_info_list);
    // getaddrinfo returns 0 on succes, or some other value when an error occured.
    if (status != 0)  cout << "getaddrinfo error" << gai_strerror(status) ;
    cout << "Creating a socket..."  << endl;
    socketfd = socket(host_info_list->ai_family, host_info_list->ai_socktype,
                      host_info_list->ai_protocol);
    if (socketfd == -1)  cout << "socket error " ;
    cout << "Connecting to the sensor's socket..."  << endl;
    status = connect(socketfd, host_info_list->ai_addr, host_info_list->ai_addrlen);
    if (status == -1)  cout << "connect error" ;
    else cout << "Connection successful, lets go grab some data" ;
    string response;
    string sendMessage;
    //keep sending messages in this loop           
    sendMessage  = "QS\r";
    int count = 0;
    while(1)
    {
	count++;
        response = getResponseForMsg(sendMessage);  //, sizeof locMsg );
        cout << "Full Response: " <<response << " and count is " << count << endl;
    }
    cout << "Receiving complete. Closing socket..." << endl;
    freeaddrinfo(host_info_list);
    close(socketfd);
    return 0;
}

