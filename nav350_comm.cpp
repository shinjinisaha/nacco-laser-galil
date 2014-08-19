#include <iostream>
#include <sstream>
#include <iomanip>
#include <cstring>      
#include <sys/socket.h> 
#include <netdb.h>      
#include "math.h"
#include <stdlib.h>

using namespace std;

int status;
struct addrinfo host_info;       
struct addrinfo *host_info_list; 
int socketfd ;

uint8_t* getResponseForMsg(uint8_t *msg, int len){


//    cout << "sending read message..."  << endl;
    //int len=30;
    ssize_t bytes_sent;
    //len = strlen(msg);
    bytes_sent = send(socketfd, msg, len, 0);

//    cout << "Waiting to recieve data..."  << endl;
    ssize_t bytes_recieved;
    uint8_t incoming_data_buffer[1000];
    bytes_recieved = recv(socketfd, incoming_data_buffer,1000, 0);
    //  wait here until some data arrives.
    if (bytes_recieved == 0) {
        cout << "host shut down." << endl ;
        
    }
    if (bytes_recieved == -1) {
        cout << "recieve error!" << endl ;
        
    }

    //cout << bytes_recieved << " bytes recieved :" << endl ;
    //incomming_data_buffer[bytes_recieved] = '\0' ;
    //cout << hex <<incoming_data_buffer << dec << endl;

    return incoming_data_buffer;
}


int main()
{
/*
        FILE *fp;

        if((fp=freopen("OUT", "w" ,stdout))==NULL) {
            printf("Cannot open file.\n");
            exit(1);
        }
*/

    memset(&host_info, 0, sizeof host_info);

    host_info.ai_family = AF_UNSPEC;     
    host_info.ai_socktype = SOCK_STREAM; 

    status = getaddrinfo("10.0.0.201", "2111", &host_info, &host_info_list);
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


//send login message
    uint8_t loginMsg[30] = {0x02, 0x73, 0x4d, 0x4e, 0x20, 0x53, 0x65, 0x74, 0x41, 0x63, 0x63, 0x65, 0x73, 0x73, 0x4d, 0x6f, 0x64, 0x65, 0x20, 0x33, 0x20, 0x46, 0x34, 0x37, 0x32, 0x34, 0x37, 0x34, 0x34, 0x03} ;

    cout << "Logging In" << endl;
    //talk to the sensor
    uint8_t *response;
    response = getResponseForMsg(loginMsg, sizeof loginMsg );

    //check the response for any errors

    //Change Mode to Standby
    uint8_t standbyMsg[24] = {0x02, 0x73, 0x4d, 0x4e, 0x20, 0x6d, 0x4e, 0x45, 0x56, 0x41, 0x43, 0x68, 0x61, 0x6e, 0x67, 0x65, 0x53, 0x74, 0x61, 0x74, 0x65, 0x20, 0x31, 0x03} ;

    cout << "Switch to StandBy Mode" << endl;
    //talk to the sensor
    response = getResponseForMsg(standbyMsg, sizeof standbyMsg );

    //check the response for any errors

    //Set current layer to 0
    uint8_t layerMsg[21] = {0x02, 0x73, 0x57, 0x4e, 0x20, 0x4e, 0x45, 0x56, 0x41, 0x43, 0x75, 0x72, 0x72, 0x4c, 0x61, 0x79, 0x65, 0x72, 0x20, 0x30, 0x03};

    cout << "Setting current layer to 0" << endl;
    //talk to the sensor
    response = getResponseForMsg(layerMsg, sizeof layerMsg );

    //check the response for any errors

    //Set Pose data format
    uint8_t setFormatMsg[28] = {0x02, 0x73, 0x57, 0x4e, 0x20, 0x4e, 0x50, 0x4f, 0x53, 0x50, 0x6f, 0x73, 0x65, 0x44, 0x61, 0x74, 0x61, 0x46, 0x6f, 0x72, 0x6d, 0x61, 0x74, 0x20, 0x31, 0x20, 0x30, 0x03} ;

    cout << "Setting structure for Pose response message" << endl;
    //talk to the sensor
    response = getResponseForMsg(setFormatMsg, sizeof setFormatMsg );

    //check the response for any errors

    //Change Mode to Navigation. Make sure you have done the mapping step successfully through SOPAS ET
    uint8_t navModeMsg[24] = {0x02, 0x73, 0x4d, 0x4e, 0x20, 0x6d, 0x4e, 0x45, 0x56, 0x41, 0x43, 0x68, 0x61, 0x6e, 0x67, 0x65, 0x53, 0x74, 0x61, 0x74, 0x65, 0x20, 0x34, 0x03} ;

    cout << "Changing Mode to Navigation" << endl;
    //talk to the sensor
    response = getResponseForMsg(navModeMsg, sizeof navModeMsg );

    //check the response for any errors

    cout << "We are ready to ask for localizations...lets begin..." << endl;
    //keep asking for localizations in this loop
    while(1){

        usleep(200000); //query every 200ms = 5Hz

        uint8_t locMsg[20] = {0x02, 0x73, 0x4d, 0x4e, 0x20, 0x6d, 0x4e, 0x50, 0x4f, 0x53, 0x47, 0x65, 0x74, 0x50, 0x6f, 0x73, 0x65, 0x20, 0x31, 0x03} ;

        //talk to the sensor
        response = getResponseForMsg(locMsg, sizeof locMsg );
        cout << "Full Response: " <<response <<endl;
        
        int y = 0;
        for(int x = 0; x<100; x++){
            if(response[x] == 0x20){
                y++;

                if(y == 7){
                    char temp[20];
                    int last = 0;
                    x++;

                    //cout << "X === the digits are : ";
                    while(response [x] != 0x20){
                        //cout << response[x] ;
                        temp[last++] = (char) response [x++]; 
                    }
                    temp[last] = '\0';
                    //cout<<endl <<"TEMP array is: "<< temp << endl;
                    int xpos = 0;
                    unsigned int value;
                    std::stringstream ss;
                    ss << std::hex << temp;
                    ss >> value;
                    xpos = static_cast<int>(value);
                    //for(int i=0; i< last; i++)
                    //    xpos = xpos + pow(16, (last-i-1)) * temp[i];
                    cout << "X position: " << xpos << "\t" ;

 //               }
 //               if(y == 8){
                    char temp1[20];
                    int last1 = 0;
                    x++;

                    //cout << "Y === the digits are : ";
                    while(response [x] != 0x20){
                        //cout << response[x] ;
                        temp1[last1++] = response [x++]; 
                    }
                    temp1[last1] = '\0';
                    //cout<< endl <<"TEMP array is: "<< temp1 << endl;
                    int ypos = 0;
                    value=0;
                    std::stringstream ss1;
                    ss1 << std::hex << temp1;
                    ss1 >> value;
                    ypos = static_cast<int>(value);
                    //for(int i=0; i< last1; i++)
                    //    ypos = ypos + pow(16, (last1-i-1)) * temp1[i];
                    cout << "Y position: " << ypos  << "\t";

//                }
//                if(y == 9){
                    uint8_t temp2[20];
                    int last2 = 0;
                    x++;

                    //cout << "THETA === the digits are : ";
                    while(response [x] != 0x20){
                        //cout << response[x] ;
                        temp2[last2++] = response [x++]; 
                    }
                    temp2[last2] = '\0';
                    //cout<<endl<<"TEMP array is: "<< temp2 << endl;
                    int theta = 0;
                    value=0;
                    std::stringstream ss2;
                    ss2 << std::hex << temp2;
                    ss2 >> value;
                    theta = static_cast<int>(value);
                    //for(int i=0; i< last2; i++)
                    //    theta = theta + pow(16, (last2-i-1)) * temp2[i];
                    cout << "Angle : " << theta  <<endl;

                }
            }
        }
    }

    cout << "Receiving complete. Closing socket..." << endl;
    freeaddrinfo(host_info_list);
    close(socketfd);

}

