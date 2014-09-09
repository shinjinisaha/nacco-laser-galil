#include <stdio.h>
#include <stdlib.h>
#include <ctime>
#include <time.h>
#include <string>
#include <cstring> 
#include <iomanip>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <sys/types.h>
#include <stdint.h>
#include <inttypes.h>
#include <ostream>
#include <netdb.h> 
#include <math.h>
 
#define VIEWER 1

#ifdef VIEWER
#include <osgViewer/Viewer>
#include <osg/ShapeDrawable>
#include <osgText/Text>
#include <osg/Geode>
#endif

using namespace std;

#define DRAW_ORIGIN    1
#define DRAW_OUTLINE    1
#define DRAW_RAYS       1
#define DRAW_POINTS     0
#define DRAW_DIFF       0

#define NUM_VERTS       1442

//#include <modbus.h>
#define IP_ADDRESS "10.0.0.100"
#define PORT 1502
#define SAFE_SPEED_REGISTER 14
#define OBSTRUCTION_ANGLE_REGISTER 15
#define BRAKE_COIL 10
//modbus_t *ctx;
//#define VIEWER 1
#ifdef VIEWER
osgViewer::Viewer viewer;
osg::Vec3Array* stopVertices = new osg::Vec3Array(NUM_VERTS);
osg::Vec3Array* brakeVertices = new osg::Vec3Array(NUM_VERTS);
osg::Vec3Array* slowVertices = new osg::Vec3Array(NUM_VERTS);
osg::Vec3Array* outlineVertices = new osg::Vec3Array(NUM_VERTS);
osg::Vec3Array* originVertices = new osg::Vec3Array(NUM_VERTS);
osg::Vec3Array* recorder_vertices = new osg::Vec3Array(NUM_VERTS);
osg::Vec3Array* diff_vertices = new osg::Vec3Array(NUM_VERTS);

osg::ref_ptr<osg::Group> root (new osg::Group);
osg::ref_ptr<osg::Geode> myshapegeode (new osg::Geode);
osg::ref_ptr<osg::Capsule> myCapsule (new osg::Capsule(osg::Vec3f(),1,2));
osg::ref_ptr<osg::ShapeDrawable> capsuledrawable (new osg::ShapeDrawable(myCapsule.get()));
osgText::Text* myText = new osgText::Text();
#endif

inline double deg2rad(const double val) { return val*0.0174532925199432957692369076848861;}
inline double rad2deg(const double val) { return val/0.0174532925199432957692369076848861;}
///////////////////////////////global variables //////
float wheelBase    = 48.0*2.54;   //48 inches
float axleLength   = 26.5*2.54;   //26.5 inches 
float laserToSteer = 11.5*2.54;   //11.5 inches 
float r_lookahead  = 150;         //1.5 meter lookahead


float coefOrder6 = 0.0;
float coefOrder5 = 0.0; 
float coefOrder4 = 0.0;
float coefOrder3 = 2.491;
float coefOrder2 = -2.938;
float coefOrder1 = 1.482;
float coefOrder0 = -0.04665;

float testFloat  = 1.234;
float  x_width   = 83;  // 33 inches in cm
        //distance of the stop field
        // 48 inches in cm
float  y_forward =  122; 
        //top speed is 1.1m/s
float velocity_cm =  110;
        //deceleration rate 
float decel_cm =  40; //20
        //this is the distance it takes to decel from max velocity to zero at given rate
float y_fullspeed =  0.5*pow(velocity_cm,2)/decel_cm;  
        //this is the braking buffer between the decel and stop zones
        // half of the stop size for starters
float y_brakezone = 61;// y_fullspeed; //61
        // this is the total distance of the stop field plus the decel zone 
float y_distance =  (y_forward+y_brakezone+y_fullspeed);

float y_radialBuffer = 20; // 20 cm of brakezone / buffer for radial curves 

float sigmoid(float x)
{
     float exp_value;
     float return_value;
     exp_value = exp((double) -x);
     return_value = 1 / (1 + exp_value);
     return return_value;
}


int fd_serialport;
int byteCounter = 0;
unsigned char inputBuffer[255];
unsigned int scanData[542];
char scanNumber[256];
int totalBytesRead = 0;

int    status;
struct addrinfo host_info;       
struct addrinfo *host_info_list; 
int    socketfd ;

int read(int bytes)
{
    int bytesRead = 0;
    bytesRead = read( fd_serialport, &inputBuffer[0], bytes);
    //printf ("%2d bytesRead: ", bytesRead);
    /*
    for (int i=0; i<bytesRead; i++){
        //printf(" %.2X ", (unsigned int) uc);
        printf(" %.2X ", inputBuffer[i]);
        if (inputBuffer[i] == '\0') {
            printf("\n");
        }
    }
    */
    //printf ("\n");
    if ((bytes == 2) && (bytesRead == 1)) {
        //usleep(100);
        bytesRead += read( fd_serialport, &inputBuffer[1], 1);
    } 

    if (bytesRead != bytes ) {
        printf ("Error, read requested %d bytes but read %d \n", bytes, bytesRead);
    }
    totalBytesRead += bytesRead;
    return bytesRead;
}

void initSerial()
{
    struct termios options;
    fd_serialport = open("/dev/LASER", O_RDWR | O_NOCTTY | O_NDELAY );
    //fd_serialport = open("/dev/ttyr00", O_RDWR | 0 | 0 );

    if(fd_serialport == -1){
        perror("Unable to open /dev/LASER");
    }

    tcgetattr(fd_serialport, &options);
    //int success = cfsetispeed(&options, B460800);
    int success = cfsetispeed(&options, B460800);
    printf ("ispeed success = %d\n", success);
    success = cfsetospeed(&options, B460800);
    printf ("ospeed success = %d\n", success);
   /* 
    options.c_cflag |= (CLOCAL | CREAD);    
    options.c_cflag |= PARENB;
    options.c_cflag |= PARODD;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_iflag |= (INPCK | ISTRIP);
    options.c_cc[VMIN] = 1;  //stop after one byte
    options.c_cc[VTIME] = 100;  //stop after 100ms timeout
*/
    tcsetattr(fd_serialport, TCSANOW, &options);
    fcntl(fd_serialport, F_SETFL, 0);

}

int foundHeader = 0;


string getResponseForMsg(string msg){
    send(socketfd, msg.c_str() , msg.length(), 0);
    char recvbuf[512];
    ssize_t bytes_received = recv(socketfd, recvbuf,512, 0);
    string receivedString;                        
    receivedString.assign(recvbuf,bytes_received); 
    char ch = *receivedString.rbegin();
    printf ("Last character is %c \n", ch);
    if (bytes_received == 0) {
        printf ("host shut down.\n");
    }
    if (bytes_received == -1)  {
        printf ("receive error!\n");
    }
    return receivedString;
}

void updateData() {
    //discard characters until we see four byte header = 00 00 00 00
#define DEBUG_UPDATE 0
    int zeroCounter = 0;
    int byteCounter = 0; 
    if (foundHeader == 0 )
        {
        while (foundHeader == 0)
        {
            int bytesRead = read(1);
            //printf("\nbytesRead: %d \n", bytesRead);
            for (int i=0; i<bytesRead; i++){
                byteCounter++;
                //printf(" %.2X ", inputBuffer[i]);
                if (inputBuffer[i] == '\0') {
                    zeroCounter++;
                } else {
                    zeroCounter = 0;
                }
                if (zeroCounter == 6) {
                    foundHeader = 1;
                    
                    if (foundHeader >= 1) {
                        printf("found initial header after (should be 1108 bytes): %d bytes ", byteCounter);
                        /*
                        if (byteCounter != 1108 ) {
                            printf(" [FAIL]\n" );
                        } else {
                            printf(" [OK]\n" );
                        }
                        */
                    }
                    
                    zeroCounter = 0;
                    byteCounter = 0;
                    if (DEBUG_UPDATE) printf("found header 00 00 00 00 \n");
                    break;
                }
            }
        }
    } else {
        int bytesRead = read(6);
        for (int i=0; i<bytesRead; i++){
            byteCounter++;
            //printf(" repeat check [%d] =  %.2X \n", i, inputBuffer[i]);
            if (inputBuffer[i] == '\0') {
                zeroCounter++;
            } else {
                zeroCounter = 0;
            }
            if (zeroCounter == 6) {
                foundHeader = 1;
                if (DEBUG_UPDATE) printf("found repeat header after (should be 6 bytes): %d bytes \n", byteCounter);
            } else {
                foundHeader = 0;
            }
        }
    }


    //check size of telegram should be 02 28
    read(2);
    if (DEBUG_UPDATE) printf("size of telegram (should be 02 28): %.2X %.2X ", inputBuffer[0], inputBuffer[1] );
    if (inputBuffer[0] != 0x02 || inputBuffer[1] != 0x28 ) {
        if (DEBUG_UPDATE) printf (" [FAIL]\n");
    } else {
        if (DEBUG_UPDATE) printf (" [OK]\n");
    }

    //check coordination flag and device code should be FF 07
    read(2);
    if (DEBUG_UPDATE) printf("coordination flag and device code (should be FF 07): %.2X %.2X ", inputBuffer[0], inputBuffer[1] );
    if (inputBuffer[0] != 0xFF || inputBuffer[1] != 0x07 ) {
        if (DEBUG_UPDATE) printf (" [FAIL]\n");
    } else {
        if (DEBUG_UPDATE) printf (" [OK]\n");
    }

    //check coordination flag and device code should be FF 07
    read(2);
    if (DEBUG_UPDATE) printf("protocol code (should be 02 01): %.2X %.2X ", inputBuffer[0], inputBuffer[1] );
    if (inputBuffer[0] != 0x02 || inputBuffer[1] != 0x01 ) {
        if (DEBUG_UPDATE) printf (" [FAIL]\n");
    } else {
        if (DEBUG_UPDATE) printf (" [OK]\n");
    }

    //check status normal 0x0000 or lockout 0x0001 
    read(2);
    if (DEBUG_UPDATE) printf("status code (should be 00 00): %.2X %.2X ", inputBuffer[0], inputBuffer[1] );
    if (inputBuffer[0] != 0x00 || inputBuffer[1] != 0x00 ) {
        if (DEBUG_UPDATE) printf (" [FAIL]\n");
        if (inputBuffer[1] == 0x01) {
            if (DEBUG_UPDATE) printf (" scanner is in lockout state  [FAIL]\n");
        }
    } else {
        if (DEBUG_UPDATE) printf (" [OK]\n");
    }

    //check scan number (timestamp)
    read(4);
    if (DEBUG_UPDATE) printf("scan number: 0x%.2X%.2X%.2X%.2X \n", inputBuffer[3], inputBuffer[2], inputBuffer[1], inputBuffer[0] );
    sprintf(scanNumber, "0x%.2X%.2X%.2X%.2X", inputBuffer[3], inputBuffer[2], inputBuffer[1], inputBuffer[0] );
    //printf("epoch: %d, scanNumber: %s, totalBytesRead: %d\n", (int)time(0), scanNumber, totalBytesRead);

    read(2);
    if (DEBUG_UPDATE) printf("telegram number: 0x%.2X%.2X \n", inputBuffer[1], inputBuffer[0] );
    char telegramNumber[256];
    sprintf(telegramNumber, "0x%.2X%.2X", inputBuffer[1], inputBuffer[0] );


    //check ID for measurement data should be BB BB
    read(2);
    if (DEBUG_UPDATE) printf("measurement ID (should be BB BB): %.2X %.2X ", inputBuffer[0], inputBuffer[1] );
    if (inputBuffer[0] != 0xBB || inputBuffer[1] != 0xBB ) {
        if (DEBUG_UPDATE) printf (" [FAIL]\n");
    } else {
        if (DEBUG_UPDATE) printf (" [OK]\n");
    }
    
    //check ID for measurement data should be 11 11
    read(2);
    if (DEBUG_UPDATE) printf("measured values for angular range 1 (should be 11 11): %.2X %.2X ", inputBuffer[0], inputBuffer[1] );
    if (inputBuffer[0] != 0x11 || inputBuffer[1] != 0x11 ) {
        if (DEBUG_UPDATE) printf (" [FAIL]\n");
    } else {
        if (DEBUG_UPDATE) printf (" [OK]\n");
    }

    //read angular measurements 1 thru 541
    for (int range=0; range < 541; range++) {
        read(2);
        if (DEBUG_UPDATE) printf("%s, %s, range, %d, 0x%.2X%.2X, ", scanNumber, telegramNumber, range, inputBuffer[1], inputBuffer[0] );
    
        //if (DEBUG_UPDATE) printf("actual unsigned chars: %.2X %.2X \n", inputBuffer[0], inputBuffer[1] );

        //if (DEBUG_UPDATE) printf("16bit endian swap: 0x%.2X%.2X \n", inputBuffer[1], inputBuffer[0] );

        unsigned int num = (inputBuffer[1] << 8) + inputBuffer[0];
        if (DEBUG_UPDATE) printf("16bit num, %u, ", num);

        bool bit15 = num & 0x8000;
        bool bit14 = num & 0x4000;
        bool bit13 = num & 0x2000;

        if (DEBUG_UPDATE) printf("bit15 fieldB, %d, ", bit15);
        if (DEBUG_UPDATE) printf("bit14 fieldA, %d, ", bit14);
        if (DEBUG_UPDATE) printf("bit13 glare, %d, ", bit13);

        unsigned int meas = num & 0x1FFF;
        if (DEBUG_UPDATE) printf("bits 12-0 distance in cm: %u\n", meas);
        scanData[range] = meas;
    
    }

    //check CRC
    read(2);
    if (DEBUG_UPDATE) printf("CRC 0x%.2X%.2X \n", inputBuffer[1], inputBuffer[0] );
}
    
int closest_y_cm; //max distance is 3000cm = 30meters = 30000mm
int stop;
double speed;
double tdist;

//float globalThingAngle[3];

void findRadius(float steerAngle,float x_offsetFromLaserZero,float y_offsetFromLaserZero, float &radius, float &lengthUntilTruck)
{
//cout << "Enter the x_offset from the laser of the truck: ";
//cin  >> x_offsetFromLaserZero;
//cout << "Enter the y_offset from the laser of the truck: " ;
//cin  >> y_offsetFromLaserZero;

steerAngle = deg2rad(steerAngle);
float y_effective       = y_offsetFromLaserZero + wheelBase + laserToSteer;
float x_effective       = x_offsetFromLaserZero - wheelBase/tan(steerAngle); //use only when abs(angle) > 10 , to avoid div by 0 
float x_zero		= - wheelBase/tan(steerAngle);
//cout << "X effective is " << x_effective << " y effective is " << y_effective << endl;
radius                  = sqrt ( pow(y_effective,2) + pow(x_effective,2) );
float thetaToTruckStart = atan2((wheelBase+laserToSteer),x_zero);
//float thetaAtPoint      = asin(y_effective/radius);
float thetaAtPoint      = atan2(y_effective,x_effective);
lengthUntilTruck        = radius*(thetaAtPoint - thetaToTruckStart)*((steerAngle < 0) - (steerAngle > 0));///sqrt(steerAngle*steerAngle)) last part is to get -1 for +ve and +1 for negative
//globalThingAngle[0]     = thetaAtPoint;
//globalThingAngle[1]     = thetaToTruckStart;
//globalThingAngle[2]     = lengthUntilTruck/radius;
//cout << " radius is found to be : " << radius << "  length until truck is " << lengthUntilTruck << " Theta to truck start " << thetaToTruckStart << " Theta at point "  << thetaAtPoint << " and angle is " << lengthUntilTruck/radius << "\n";
//cout << " radius is found to be : " << radius << "  length until truck is " << lengthUntilTruck << " Theta to truck start " << thetaToTruckStart << " Theta at point "  << thetaAtPoint << "\n";
return;
}



/*

void findRadius(float steerAngle,float x_offsetFromLaserZero,float y_offsetFromLaserZero, float &radius, float &lengthUntilTruck)
{
steerAngle = deg2rad(steerAngle);
float y_effective 	= y_offsetFromLaserZero + wheelBase + laserToSteer;
float x_effective       = x_offsetFromLaserZero - wheelBase/tan(steerAngle); //use only when abs(angle) > 10 , to avoid div by 0 
radius                  = sqrt ( pow(y_effective,2) + pow(x_effective,2) );
float thetaToTruckStart = atan((wheelBase + laserToSteer)/x_effective);
//float thetaAtPoint      = asin(y_effective/radius);
float thetaAtPoint      = atan(y_effective/x_effective);
lengthUntilTruck        = radius*(thetaAtPoint - thetaToTruckStart);
globalThingAngle[0]     = thetaAtPoint;
globalThingAngle[1]	= thetaToTruckStart;
globalThingAngle[2]	= lengthUntilTruck/radius;
//cout << " radius is found to be : " << radius << "  length until truck is " << lengthUntilTruck << " Theta to truck start " << thetaToTruckStart << " Theta at point "  << thetaAtPoint << "\n";
return;
}       
*/

void updateVerts()
{
    //vertices = new osg::Vec3Array(NUM_VERTS);
    double angle = 0.0;
    int px = 0, py = 0, pz = 0;
    stop = 541;
    closest_y_cm = 5000; //max distance is 3000cm = 30meters = 30000mm
    int closest_meas = 5000;
    float ratio;
    bool FLAG_STRAIGHT_FIELD = true; 
    float tillerAngle;
    string response;
    int closest_r_cm = 5000;


    response    = getResponseForMsg("MG_ TPB\r");   // the tiller angle,
    tillerAngle = -atof(response.c_str())/27.7778;  // convert to the convention and scale here. (-pi/2 ,pi/2)
    cout << " tillerAngleObserved is " << tillerAngle << endl;

    tillerAngle = 45;
        float radiusRightExtreme;
        float radiusLeftExtreme;
        float dummy;

	findRadius(tillerAngle , axleLength/2 + x_width ,  0 , radiusRightExtreme, dummy );
        findRadius(tillerAngle , -(axleLength/2 + x_width) , 0 , radiusLeftExtreme, dummy);

//cout << " extremes are  " << radiusRightExtreme  << " and " << radiusLeftExtreme << " \n";
    //for (int scanIndex = 0; scanIndex<542; scanIndex++) {
    for (int scanIndex = 0; scanIndex<541; scanIndex++) {
        unsigned int meas = scanData[scanIndex];
        
        int baseVert = scanIndex*2; //takes two vertices to make a line
        int tipVert = baseVert+1;  //even verts will be (0,0,0), odd will be data
        //double degrees = 315.0 - (double)scanIndex/2.0;  //first 45 are blank, then half degrees
        double degrees = (double)scanIndex/2.0 - 45.0;  //first 45 are blank, then half degrees
        if (degrees < 0) {
            degrees += 360.0;
        }

    double rads = deg2rad(degrees);  //convert to radians for trig 
    int x = meas * cos(rads);
    int y = meas * sin(rads);
    int z = 0;
    float radiusObstacle;
    float lengthToObstacle;
    float dummy;


if ((scanIndex > 100) && (scanIndex < 541-100))    //consider only about 160 degrees in front of the truck
{       
        if (abs(tillerAngle) > 12)   //check turning, use straight field is steering is low
        {
		findRadius(tillerAngle , x , y , radiusObstacle, lengthToObstacle); // radius and length to truck of obstacle
		FLAG_STRAIGHT_FIELD = false;
		if  ( (radiusObstacle-radiusLeftExtreme)*(radiusObstacle-radiusRightExtreme) < 0)   //obstacle in field
		{
	        //cout << "value of obstacle is " << radiusObstacle   << " and length is " << lengthToObstacle <<  endl ;	
			if (lengthToObstacle < closest_r_cm)
			{
//				cout << "glbal thing angle is " << globalThingAngle[0] << " angle to truck is " << globalThingAngle[1] << " and the difference is " << globalThingAngle[2]  << endl;
				closest_r_cm = lengthToObstacle;
			}
			
		}
	}
}	


	//saving the angle for the closest measurement
	if((int)meas < closest_meas && y>0){
		closest_meas = meas;
		angle = degrees;
	}

        //stop field
        if (x < x_width && x > -x_width && y < y_forward && y > 0) {
#ifdef VIEWER
            (*stopVertices)[baseVert].set(0,0,0);  //baseVert will always be (0,0,0) for origin
            (*stopVertices)[tipVert].set(x, y, z);  //data point
#endif
            if (y < closest_y_cm) {
                closest_y_cm = y;
            }
        } else {
#ifdef VIEWER
            (*stopVertices)[baseVert].set(0,0,0);  //baseVert will always be (0,0,0) for origin
            (*stopVertices)[tipVert].set(0,0,0);  //data point        
#endif
            stop--;
        }

        //braking zone
        if (x < x_width && x > -x_width && y >= y_forward && y < (y_forward + y_brakezone)) {
#ifdef VIEWER
            (*brakeVertices)[baseVert].set(0,0,0);  //baseVert will always be (0,0,0) for origin
            (*brakeVertices)[tipVert].set(x, y, z);  //data point
#endif
            if (y < closest_y_cm) {
                closest_y_cm = y;
            }
        } else {
#ifdef VIEWER
            (*brakeVertices)[baseVert].set(0,0,0);  //baseVert will always be (0,0,0) for origin
            (*brakeVertices)[tipVert].set(0,0,0);  //data point        
#endif
        }

        //slow field
        if (x < x_width && x > -x_width && y >= (y_forward+y_brakezone) && y < y_distance) {
#ifdef VIEWER
            (*slowVertices)[baseVert].set(0,0,0);  //baseVert will always be (0,0,0) for origin
            (*slowVertices)[tipVert].set(x, y, z);  //data point
#endif
            if (y < closest_y_cm) {
                closest_y_cm = y;
		printf ("closest_y_cm: %d \n",closest_y_cm); 
            }
        } else {
#ifdef VIEWER
            (*slowVertices)[baseVert].set(0,0,0);  //baseVert will always be (0,0,0) for origin
            (*slowVertices)[tipVert].set(0,0,0);  //data point        
#endif
        }
        
#ifdef VIEWER
        (*outlineVertices)[baseVert].set(px, py, pz);  //data point
        (*outlineVertices)[tipVert].set(x, y, z);  //data point
#endif
        px = x;  py = y; pz = z;
        //printf ("scan: %s, index: %d, r: %u, deg: %.1f, rad: %0.4f x: %d, y: %d, z: %d\n", scanNumber, scanIndex, meas, degrees, rads, x, y, z);
    }

//cout << " RE " << radiusRightExtreme << " LE " << radiusLeftExtreme <<  " closest_r_c " << closest_r_cm << endl;
    //if (closest_r_cm > ((radiusLeftExtreme + radiusRightExtreme)*3.14159265359*2/3)) { //look ahead of pi/4 for the current center of rotation 
	cout << "closest_r_ cm si " << closest_r_cm << endl;
    if (closest_r_cm > y_forward + r_lookahead)
    {
	closest_r_cm = y_distance;
    }
    if (FLAG_STRAIGHT_FIELD)
    {
    	tdist = closest_y_cm - (y_forward+y_brakezone);  
	//cout << "tdist observed is " << tdist << endl;
	if (tdist > y_distance){
                tdist = y_distance;}
    	if (tdist < 0){
                tdist = 0;}
    	ratio = tdist/(y_distance - (y_forward+y_brakezone));
    }

    else 
    {
	tdist = closest_r_cm - y_forward- y_radialBuffer ;    //ignore buffer, cause we will go slow in turns anyways

	//cout << "tdist observed is radial is " << tdist << endl;
	if (tdist > r_lookahead ){
                tdist =r_lookahead ;}
    	if (tdist < 0){
                tdist = 0;}
   	ratio = (tdist)/(r_lookahead);
    }
   //cout << " Tdist is " << tdist << " with bool value " << FLAG_STRAIGHT_FIELD << "\n" ;
 
    //speed = sqrt (2.0*tdist * decel_cm);
    //speed = 7*pow (tdist/(y_distance),2)/3;
    //speed = pow(tdist/(2*y_fullspeed),1.5);
//    speed = coefOrder3*pow(ratio,3)+coefOrder2*pow(ratio,2)+coefOrder1*pow(ratio,1)+coefOrder0*pow(ratio,0);
  

    speed =coefOrder6*pow(ratio,6)+coefOrder5*pow(ratio,5)+coefOrder4*pow(ratio,4)+ coefOrder3*pow(ratio,3)+coefOrder2*pow(ratio,2)+coefOrder1*pow(ratio,1)+coefOrder0*pow(ratio,0);
    cout << "tdist is " << tdist << " and ratio is " << ratio << " and speed value is " << speed <<   "\n";
    if (speed > 1) speed = 1;
    if (speed < 0) speed = 0;
    if (stop) speed = 0;
    if (speed != speed){
       speed = 0;}

    string sendCommand,sendGalilFullCommand;
    string secondCommand;
    //keep sending messages in this loop           
    sendCommand = "p408=";
    // secondCommand = ""
    // sendMessage  = speed;
    char speedChar[21];

    float percent_speed = speed;
    sprintf(speedChar,"%f",percent_speed);
    sendGalilFullCommand = sendCommand+speedChar+"\r"; 
    //printf("The sent command is %s\n",sendGalilFullCommand);
    //cout << "Sent command is " << sendGalilFullCommand << "\n" ; 
//    response = getResponseForMsg("MG_ TPB\r");  //, sizeof locMsg );
    response = getResponseForMsg(sendGalilFullCommand);  //, sizeof locMsg );
    //cout << "Full Response: " <<response << " and count is " << count << endl;
//    cout << "The response is lalalalla" << std::string::stof(response) << " and the value after scaling is "  << std::stof(response)*27.77 << "\n" ; 
     printf("Full Response: %s \n", response.c_str()); 
    
    printf("epoch: %d, scanNumber: %s, totalBytesRead: %d, stop: %d, closest_y: %d, closest_reading:%d angle: %f speed: %f percent_speed: %f \n", (int)time(0), scanNumber, totalBytesRead, stop, closest_y_cm, closest_meas, angle, speed, percent_speed);


    char output[256] = "";
    sprintf(output,"speed: %f, percent_speed: %f\n", speed, percent_speed);

#ifdef VIEWER
    myText->setText(output);
#endif
    //viewer.setSceneData( root.get() );

 
    //modbus_write_register(ctx, SAFE_SPEED_REGISTER, percent_speed);
    //modbus_write_register(ctx, OBSTRUCTION_ANGLE_REGISTER, ((int)(180 - (int)angle)/10) * 10) ;
    //modbus_write_bit(ctx, BRAKE_COIL, percent_speed);

   /* 
    (*originVertices)[0].set(0,0,0);  //baseVert will always be (0,0,0) for origin
    (*originVertices)[1].set(4000, 0, 0);  //data point
    (*originVertices)[2].set(0,0,0);  //baseVert will always be (0,0,0) for origin
    (*originVertices)[3].set(0, 8000, 0);  //data point
    (*originVertices)[4].set(0,0,0);  //baseVert will always be (0,0,0) for origin
    (*originVertices)[5].set(0, 0, 2000);  //data point
*/
    //lc footprint - must stop outside of this
#ifdef VIEWER
    (*originVertices)[6].set(-x_width, 0,0);  //baseVert will always be (0,0,0) for origin
    (*originVertices)[7].set(x_width,  0, 0);  //data point
    (*originVertices)[8].set(-x_width, 0, 0);  //baseVert will always be (0,0,0) for origin
    (*originVertices)[9].set(-x_width, y_forward, 0);  //data point
    (*originVertices)[10].set(x_width, 0,0);  //baseVert will always be (0,0,0) for origin
    (*originVertices)[11].set(x_width, y_forward, 0);  //data point
    (*originVertices)[12].set(-x_width, y_forward, 0);  //baseVert will always be (0,0,0) for origin
    (*originVertices)[13].set(x_width, y_forward, 0);  //data point
    
    (*originVertices)[14].set(-x_width, 0,0);  //baseVert will always be (0,0,0) for origin
    (*originVertices)[15].set(x_width,  0, 0);  //data point
    (*originVertices)[16].set(-x_width, 0, 0);  //baseVert will always be (0,0,0) for origin
    (*originVertices)[17].set(-x_width, y_distance, 0);  //data point
    (*originVertices)[18].set(x_width, 0,0);  //baseVert will always be (0,0,0) for origin
    (*originVertices)[19].set(x_width, y_distance, 0);  //data point
    (*originVertices)[20].set(-x_width, y_distance, 0);  //baseVert will always be (0,0,0) for origin
    (*originVertices)[21].set(x_width, y_distance, 0);  //data point

    (*originVertices)[22].set(-x_width, y_forward+y_brakezone, 0);  //baseVert will always be (0,0,0) for origin
    (*originVertices)[23].set(x_width, y_forward+y_brakezone, 0);  //data point
#endif

/*   
    (*originVertices)[6].set(0,0,0);  //baseVert will always be (0,0,0) for origin
    (*originVertices)[7].set(0, 0, 2000);  //data point
    (*originVertices)[8].set(0,0,0);  //baseVert will always be (0,0,0) for origin
    (*originVertices)[9].set(0, 0, 2000);  //data point
    (*originVertices)[10].set(0,0,0);  //baseVert will always be (0,0,0) for origin
    (*originVertices)[11].set(0, 0, 2000);  //data point
  */  

    /*
    (*vertices)[0].set(0,0,0);
    (*vertices)[1].set(0,0,5000);
    (*vertices)[2].set(0,0,0);
    (*vertices)[3].set(10000,10000,10000);
    (*vertices)[4].set(0,0,0);
    (*vertices)[5].set(-10000,-10000,-10000);
    (*vertices)[6].set(0,0,0);
    (*vertices)[7].set(-10000,300,-10000);
    */

}

#ifdef VIEWER
void updateVerts_old()
{
    int reading = 0;
    for (int degree = 1262; degree > 180; degree-=2) {
        //cout << "Degree: " << degree << " Radian: " << deg2rad(0.5*degree) << " sin: " << 1.0*sin(deg2rad(0.5 * degree)) << " cos: " << 1.0*cos(deg2rad(0.5 * degree)) << endl;
        //(*vertices)[degree].set(0,0,0);
        //(*vertices)[degree+1].set(1.0*sin(deg2rad(0.5 * degree)), 0, 1.0*cos(deg2rad(0.5 * degree)));

        unsigned int meas = scanData[reading] * 10;

        //draw the connected readings outline like the sopas tool
        if (DRAW_OUTLINE) {
            (*stopVertices)[degree].set(meas*sin(deg2rad(0.25 * (degree+720))), 0, meas*cos(deg2rad(0.25 * (degree+720))));
            (*stopVertices)[degree-1].set(meas*sin(deg2rad(0.25 * (degree+720))), 0, meas*cos(deg2rad(0.25 * (degree+720))));
        }

        //draw the current ray from the origin
        if (DRAW_RAYS) {
            (*stopVertices)[degree].set(0,0,0);
            //(*vertices)[degree].set(d.data16bit[i].range[reading]*sin(deg2rad(0.25 * (degree+720))), 0, d.data16bit[i].range[reading]*cos(deg2rad(0.25 * (degree+720))));
            //(*vertices)[degree-1].set(meas*sin(deg2rad(0.25 * (degree+720))), 0, meas*cos(deg2rad(0.25 * (degree+720))));
            (*stopVertices)[degree-1].set(reading, 0, meas);
        }
        
        reading++;
    }
}
#endif

#ifdef VIEWER
class redrawCallback : public osg::NodeCallback
{
public:
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
        updateData();
        updateVerts();
        traverse(node, nv);
        //usleep(1);
    }
};
#endif

#ifdef VIEWER

int initViewer()
{
    //Creating the root node
    //osg::ref_ptr<osg::Group> root (new osg::Group);

    //The geode containing our shpae
    //osg::ref_ptr<osg::Geode> myshapegeode (new osg::Geode);

    //Our shape: a capsule, it could have been any other geometry (a box, plane, cylinder etc.)
    //osg::ref_ptr<osg::Capsule> myCapsule (new osg::Capsule(osg::Vec3f(),1,2));

    //Our shape drawable
    //osg::ref_ptr<osg::ShapeDrawable> capsuledrawable (new osg::ShapeDrawable(myCapsule.get()));

    //myshapegeode->addDrawable(capsuledrawable.get());

    // create Geometry object to store all the vertices and lines primitive.
    osg::Geometry* stopLinesGeom = new osg::Geometry();
    osg::Geometry* brakeLinesGeom = new osg::Geometry();
    osg::Geometry* slowLinesGeom = new osg::Geometry();
    osg::Geometry* outlineGeom = new osg::Geometry();
    osg::Geometry* originGeom = new osg::Geometry();
    stopLinesGeom->setUseDisplayList( false );
    brakeLinesGeom->setUseDisplayList( false );
    slowLinesGeom->setUseDisplayList( false );
    outlineGeom->setUseDisplayList( false );
    originGeom->setUseDisplayList( false );

    // pass the created vertex array to the points geometry object.
    stopLinesGeom->setVertexArray(stopVertices);
    brakeLinesGeom->setVertexArray(brakeVertices);
    slowLinesGeom->setVertexArray(slowVertices);
    outlineGeom->setVertexArray(outlineVertices);
    originGeom->setVertexArray(originVertices);
    
    // set the colors as before, plus using the above
    osg::Vec4Array* stopColors = new osg::Vec4Array;
    osg::Vec4Array* brakeColors = new osg::Vec4Array;
    osg::Vec4Array* slowColors = new osg::Vec4Array;
    osg::Vec4Array* outlineColors = new osg::Vec4Array;
    osg::Vec4Array* originColors = new osg::Vec4Array;
    
    stopLinesGeom->setColorArray(stopColors);
    brakeLinesGeom->setColorArray(brakeColors);
    slowLinesGeom->setColorArray(slowColors);
    outlineGeom->setColorArray(outlineColors);
    originGeom->setColorArray(originColors);

    stopLinesGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
    brakeLinesGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
    slowLinesGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
    outlineGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
    originGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
    
    stopColors->push_back(osg::Vec4(1.0f,0.0f,0.0f,1.0f)); //red
    brakeColors->push_back(osg::Vec4(1.0f,1.0f,0.0f,1.0f)); //yellow
    slowColors->push_back(osg::Vec4(0.0f,1.0f,0.0f,1.0f)); //green
    //outlineColors->push_back(osg::Vec4(1.0f,0.0f,0.0f,1.0f)); //red
    outlineColors->push_back(osg::Vec4(0.0f,1.0f,0.0f,1.0f)); //green
    originColors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f)); //white

    // set the normal in the same way color.
    osg::Vec3Array* normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0.0f,0.0f,1.0f));

    stopLinesGeom->setNormalArray(normals);
    stopLinesGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);
    brakeLinesGeom->setNormalArray(normals);
    brakeLinesGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);
    slowLinesGeom->setNormalArray(normals);
    slowLinesGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);
    
    outlineGeom->setNormalArray(normals);
    outlineGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);
    
    originGeom->setNormalArray(normals);
    originGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);

    // This time we simply use primitive, and hardwire the number of coords to use
    // since we know up front,
    if (DRAW_OUTLINE) {
        outlineGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,NUM_VERTS));
        myshapegeode->addDrawable(outlineGeom);
    }

    if (DRAW_RAYS) {
        stopLinesGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,NUM_VERTS));
        brakeLinesGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,NUM_VERTS));
        slowLinesGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,NUM_VERTS));
        myshapegeode->addDrawable(stopLinesGeom);
        myshapegeode->addDrawable(brakeLinesGeom);
        myshapegeode->addDrawable(slowLinesGeom);
    }

    if (DRAW_ORIGIN) {
        originGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,NUM_VERTS));
        myshapegeode->addDrawable(originGeom);
    }

    // add the points geometry to the geode.
    myshapegeode->setDataVariance(osg::Object::DYNAMIC);

    root->addChild(myshapegeode.get());

    //The geode containing our shpae
    osg::ref_ptr<osg::Geode> myTextGeode (new osg::Geode);
    
    //osgText::Text* myText = new osgText::Text();

    // Geode - Since osgText::Text is a derived class from drawable, we 
    // must add it to an osg::Geode before we can add it to our ScenGraph.
    myTextGeode->addDrawable(myText);

    //Set the screen alignment - always face the screen
    myText->setAxisAlignment(osgText::Text::SCREEN);

    //Set the text to our default text string
    myText->setText("Default Text");


    //myText->setPosition(osg::Vec3d(25, 75, 0));
    myText->setPosition(osg::Vec3d(0, 0, 0));
    myText->setColor(osg::Vec4d(1.0f, 1.0f, 1.0f, 1.0f));
    myText->setCharacterSize(48);
    //myText->setFont("./fonts/Vera.ttf");
   
    char output[256] = "";
    sprintf(output, "epoch: %d, scanNumber: %s, totalBytesRead: %d, stop: %d, closest_y: %d, speed: %f\n", (int)time(0), scanNumber, totalBytesRead, stop, closest_y_cm, speed);

    myText->setText(output);

    root->addChild(myTextGeode.get());


    root->setUpdateCallback(new redrawCallback);

    viewer.setSceneData( root.get() );
    //viewer.setThreadingModel(osgViewer::Viewer::ThreadingModel::SingleThreaded);

    //Stats Event Handler s key
    //viewer.addEventHandler(new osgViewer::StatsHandler);

    //Windows size handler
    //viewer.addEventHandler(new osgViewer::WindowSizeHandler);

    // add the state manipulator
    //viewer.addEventHandler( new       osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );

    //The viewer.run() method starts the threads and the traversals.
    return (viewer.run());
}
#endif



int main ()
{
/*    
//MODBUS INIT
    ctx = modbus_new_tcp(IP_ADDRESS, PORT);

    if (modbus_connect(ctx) == -1) {
        //fprintf(stderr, "Connexion failed: %s\n",modbus_strerror(errno));
        printf("\n\n MODBUS CONNECTION FAILED");
        modbus_free(ctx);
        return -1;
    }
    else
    printf("\n MODBUS CONNECTED!!");
*/

ifstream fin("param.txt");
string param_name;
int param_value;

cout << "before  " <<coefOrder3 << "\n" << coefOrder2 << "\n" << coefOrder1 << "\n" << coefOrder0 << "\n";

while ( fin >> param_name >> param_value )
{
cout << "The param name is "<<param_name<<"\n The param value is "<<param_value << "\n";
  if (!param_name.compare("coefOrder3")){
  coefOrder3 = param_value;}

  if (!param_name.compare("coefOrder2")){
  coefOrder2 = param_value;}

  if (!param_name.compare("coefOrder1")){
  coefOrder1 = param_value;}

  if (!param_name.compare("coefOrder0")){
  coefOrder0 = param_value;}

  if (!param_name.compare("x_width")){
  x_width = param_value;}

  if (!param_name.compare("y_forward")){
  y_forward = param_value;}

  if (!param_name.compare("velocity_cm")){
  velocity_cm = param_value;}

  if (!param_name.compare("decel_cm")){
  decel_cm = param_value;}

  if (!param_name.compare("y_fullspeed")){
  y_fullspeed = param_value;}

  if (!param_name.compare("y_brakezone")){
  y_brakezone = param_value;}

  if (!param_name.compare("y_distance")){
  y_distance = param_value;}

}
fin.close();
cout << "after  " << "Coef3" << coefOrder3 << "\n" << "coef2" << coefOrder2 << "\n" <<"coef1" << coefOrder1 << "\n" << "coef0" << coefOrder0 << "\n" <<"x_wodth" <<  x_width << "\n" <<"y_forward" <<  y_forward << "\n" << "velocoity cm" <<  velocity_cm << "\n" <<"decel cm" <<  decel_cm << "\n" << " y_full sped" << y_fullspeed << "\n" ;

    y_distance = y_forward + y_fullspeed + y_brakezone;


    memset(&host_info, 0, sizeof host_info);
    host_info.ai_family   = AF_UNSPEC;     
    host_info.ai_socktype = SOCK_STREAM; 
    status = getaddrinfo("10.0.0.7", "1001", &host_info, &host_info_list);
    // getaddrinfo returns 0 on succes, or some other value when an error occured.
    if (status != 0)  printf ("getaddrinfo ", gai_strerror(status));
    printf ("Creating a socket...\n");
    socketfd = socket(host_info_list->ai_family, host_info_list->ai_socktype,
                      host_info_list->ai_protocol);
    if (socketfd == -1)  printf ("socket error ");
    printf ("Connecting to the sensor's socket...\n");
    status = connect(socketfd, host_info_list->ai_addr, host_info_list->ai_addrlen);
    if (status == -1)  printf ("connect error\n") ;
    else printf ("Connection successful, lets go grab some data\n") ;

#define SERIAL_ONLY_TEST 0
#if SERIAL_TEST
    while (1) {    
        updateData();
        for (int range=1; range<542; range++) {
            printf("%d %u ", range, scanData[range]);
        }
        printf("\n");
    }
    return 0;
#endif 

    initSerial();
    updateData();
    updateVerts();
#ifdef VIEWER
    initViewer();
#endif
while(1){
    updateData();
    updateVerts();
}
    return 0;
}

