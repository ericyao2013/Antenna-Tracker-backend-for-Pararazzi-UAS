/*
This project is an intent to create an anthenna tracker system for paparazzi uas systems running on a BeagleBone black. 

This 
*/


#include <glib.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <sys/types.h>
#include <unistd.h>

#include "calculator.h"

#define TRACKER_ID 255

#ifdef USE_BBB_PWM
  #include "bbb_cape_servo_control.cpp"
#else
  #include "bbb_ob_control.h"
#endif

#define MANUAL 0
#define AUTO 1

#define MaxNumDevices 25
#define MaxNumConfNames 500
#define MaxNumConfNameLength 100

//Change these values if there will be more than one tracker
#define TRACKER_NAME "BBB_Tracker"


#ifdef __APPLE__
char defaultIvyBus[] = "224.255.255.255:2010";
#else
char defaultIvyBus[] = "127.255.255.255:2010";
#endif
char* IvyBus;

// verbose flag
int verbose = 0;

int StartParamUsed = 0;
int tracker_mode;

float tilt_angle;
float pan_angle;

int SelTrackerID = 0;
int SelAircraftID = 0;

float PanDirectionOffset;

void Send_ContList_Msg (int ContListNumb);

void Refresh_Sliders (void);
void print_help();

int Check_Controlled_Aircraft_List (int Aircraft_Id);
int Check_Controlled_Tracker_List (int Tracker_Id);
int Get_Dev_Ind(int device_id);
void Get_Device_Name( int dev_status_index );
void Load_Device_Names (void);
int check_device_name(int dev_id);
void request_ac_config(int ac_id_req);
void save_device_name(int dev_id, char *dev_name);
void calculate_joint_degrees(int TrackerId, int AcID);
int set_track_list(int TrackerId, int AcId);


int ProcessID;
int RequestID;


typedef struct {
	int used;
	int device_id;
	double latitude;
	double longitude;
	double altitude;
	float roll;
	float pitch;
	float heading;
  float speed;
	char *name;
}DevStatus_s ;
DevStatus_s DevStatus [2 * MaxNumDevices];  //Holds all status of devices

typedef struct{
	int used;
	int mode;
	int tracker_id;
	int aircraft_id;
	float pan_angle;
	float tilt_angle;
}d_s;
d_s d [MaxNumDevices];

typedef struct {
	int device_id;
	char name[MaxNumConfNameLength];
}DevNames_s ;
DevNames_s DevNames [MaxNumConfNames];



int program_started = 0;
//ERRORS
int MaxNumDevicesReached = 1 ;   //will be -1 if error flag raised

int TrackerCoordinatesConverted = 0; 

void on_FLIGHT_PARAM_STATUS(IvyClientPtr app, void *user_data, int argc, char *argv[]){

  //check if device id is in controlled aircrafts.
  int IncDevId;
  IncDevId = Get_Dev_Ind ( atoi(argv[0]) );

  if (IncDevId >= 0 ) {
    
    //Fill device status values
    DevStatus[IncDevId].altitude = atof(argv[8]);
    DevStatus[IncDevId].longitude = atof(argv[5]);
    DevStatus[IncDevId].latitude = atof(argv[4]);
    DevStatus[IncDevId].roll = atof(argv[1]);
    DevStatus[IncDevId].pitch = atof(argv[2]);
    DevStatus[IncDevId].heading = atof(argv[3]);
     DevStatus[IncDevId].speed = atof(argv[5]); 
    //If device is in control list send data over ivybus

    int pre_ind;
    pre_ind = Check_Controlled_Aircraft_List(atoi(argv[0]));

    if ( pre_ind >= 0 ){    
      Send_ContList_Msg (pre_ind); 
      calculate_joint_degrees(d[pre_ind].tracker_id, d[pre_ind].aircraft_id );
    }

  }
  else return;
}


void on_TRACKER_PARAM_STATUS(IvyClientPtr app, void *user_data, int argc, char *argv[]){

  //Fill device status values if this is the msg we are looking for.. 
  if atoi(atof(argv[0]) == TRACKER_ID) {
    DevStatus[0].altitude = atof(argv[8]);
    DevStatus[0].longitude = atof(argv[5]);
    DevStatus[0].latitude = atof(argv[4]);
    DevStatus[0].roll = atof(argv[1]);
    DevStatus[0].pitch = atof(argv[2]);
    DevStatus[0].heading = atof(argv[3]);

    if (verbose) {
      printf("Tracker Position:  Lat: %f \tLon: %f \tAlt:%f \n",DevStatus[0].latitude, DevStatus[0].longitude, DevStatus[0].altitude);
    }

  }

  return;
}


void calculate_joint_degrees(int TrackerId, int AcID) {

  float pan_angle;
  float tilt_angle;  
  int TrackerInd;
  int AcInd;

  //Get tracker index
  TrackerInd = Get_Dev_Ind(TrackerId);
  //Get Aircraft index  
  AcInd = Get_Dev_Ind(AcID);
  
  //calculate joint angles
  calc_joint_ang( DevStatus[TrackerInd].latitude, DevStatus[TrackerInd].longitude, DevStatus[AcInd].latitude, DevStatus[AcInd].longitude, DevStatus[AcInd].altitude, DevStatus[TrackerInd].altitude, &pan_angle, &tilt_angle );

  //Add offset to pan angle
  pan_angle = pan_angle + PanDirectionOffset; 
  if (pan_angle<0) pan_angle += 360.;
  if (verbose) {
    printf("Required joint values: Tilt= %.1f \tPan= %.1f \tACspeed= %.1f \tAChead= %.1f\n",tilt_angle,pan_angle,DevStatus[AcInd].speed,DevStatus[AcInd].heading );
  }

  if (TrackerInProgress) set_tracker_motors(pan_angle,100,tilt_angle,100);   
}

void on_tracker_ANT_TRACK(IvyClientPtr app, void *user_data, int argc, char *argv[]) {

  int TrackerId= atoi(argv[0]);

  if (check_device_name(TrackerId) < 0) return;

  int IncDevId = Check_Controlled_Tracker_List (TrackerId );

  if (IncDevId >= 0 ) {

    if (d[IncDevId].mode == AUTO) {
      d[IncDevId].pan_angle= atof(argv[2]);
      d[IncDevId].tilt_angle = atof(argv[3]);
    }
  }
  else return;

}


void on_tracker_NEW_AC(IvyClientPtr app, void *user_data, int argc, char *argv[]) {

  //Request config
  printf("on_tracker_NEW_AC");
  request_ac_config(atoi(argv[0]));
}

void request_ac_config(int ac_id_req) {

  RequestID++;
  IvySendMsg("anttrUI %d_%d CONFIG_REQ %d" ,ProcessID, RequestID ,ac_id_req );

  if (verbose) {
    printf("AC(id= %d) config requested.\n",ac_id_req);
  }
}

void on_tracker_GET_CONFIG(IvyClientPtr app, void *user_data, int argc, char *argv[]) {

  //Check request_id
  int i=0;

  int RmId[2];
 
  //Split arg0 to get process id and request id
  char * mtok;
  mtok = strtok (argv[0],"_");
  while (mtok != NULL || i>2)
  {
    RmId[i]= atoi(mtok);
    i++;
    mtok = strtok (NULL, "_");
  }

  //Check whether process id and request id matches or not
  if ( RmId[0]== ProcessID && RmId[1]==RequestID) {

    save_device_name(atoi(argv[1]),argv[7]);

  }
}


void on_set_tracker(IvyClientPtr app, void *user_data, int argc, char *argv[]) {

  //Check request_id
  SelTrackerID = atoi(argv[1]);
  SelAircraftID = atoi(argv[2]);

  //Uncomment these lines if there is no onboard gps.. 
  /*
  DevStatus[0].latitude = atof(argv[3]);
  DevStatus[0].longitude = atof(argv[4]);
  DevStatus[0].altitude = atof(argv[5]);
  */

  if (verbose) printf("SetTracker, Lat: %.7f \t Lon: %.7f \t Alt:%f \n",DevStatus[0].latitude, DevStatus[0].longitude, DevStatus[0].altitude );  

  int tracker_ind;
  tracker_ind = set_track_list(SelTrackerID, SelAircraftID);

  if (verbose) printf("Tracker id: %d is now tracking: %d\n",  d[tracker_ind].tracker_id, d[tracker_ind].aircraft_id);

  TrackerInProgress = 1;

}

void on_stop_tracker(IvyClientPtr app, void *user_data, int argc, char *argv[]) {

  if (atoi(argv[1]) == TRACKER_ID) {
    TrackerInProgress = 0 ;
    if (verbose) printf("Tracking stopped by user!\n");
  }

}

int set_track_list(int TrackerId, int AcId) {

  int i=0;
  //check if device is already in control list
  while ( (d[i].used != 0) && (i < MaxNumDevices) ) {
    if ( d[i].tracker_id == TrackerId ) {
    	d[i].aircraft_id = AcId;
    	return i;
    } 
      i +=1;
  }
  //if MaxNumDevices reached return -1
  if ( i >= (MaxNumDevices) ) { 
    MaxNumDevicesReached =-1;
    return -1;
  }
  //Add it to control list & return index if it
  d[i].used = 1;
  d[i].tracker_id = TrackerId;
  d[i].aircraft_id = AcId;
  d[i].mode = 0;
  
  if (verbose) printf("New tracker added.. (ID=%d)\n",TrackerId);

  return i;

}



void save_device_name(int dev_id, char *dev_name) {

  if (verbose) printf("Saving device name id= %d name=%s\n",dev_id,dev_name);
  
  //Search DevNames
  int i=0;
  while ( (i< MaxNumConfNames) && (DevNames[i].device_id > 0) ) {
      if ( DevNames[i].device_id == dev_id ) {
        //Rewrite device name
        strcpy(DevNames[i].name, dev_name);
        if (verbose) printf("Device saved DevNames (RW). id= %d name=%s\n",dev_id,dev_name);
        return;

      }
      
      i++;
  }

  //save new item
  while ( i< MaxNumConfNames) {
    if ( DevNames[i].device_id == 0 ) {
	    //save device name
	    strcpy(DevNames[i].name, dev_name);
	    if (verbose) printf("Device saved DevNames. (new) id= %d name=%s index=%d\n",dev_id,dev_name,i);

      DevNames[i].device_id=dev_id;
      return;
    }
    
    i++;
  }
  //No country for new device!!
  if (verbose) printf("Device cannot be saved. MaxNumConfNames reached!\n");

  return;
}

// Print help message
void print_help() {
  printf("Usage: ant_tracker [options]\n");
  printf(" Options :\n");
  printf("   -o Pan direction offset. \tdefault is %f\n", PanDirectionOffset);
  printf("   -b <Ivy bus>\tdefault is %s\n", defaultIvyBus);
  printf("   -v\tverbose\n");
  printf("   -h --help show this help\n");
}



int main(int argc, char **argv) {

  
  PanDirectionOffset = 0;

  ProcessID= getpid();

  
  printf("PPRZ anthenna tracker running..\n");
  
  // try environment variable first, set to default if failed
  IvyBus = getenv("IVYBUS");
  if (IvyBus == NULL) IvyBus = defaultIvyBus;

  // Parse options
  int i;
  for (i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
      print_help();
      exit(0);
    }
    else if (strcmp(argv[i], "-b") == 0) {
      IvyBus = argv[++i];
    }
    else if (strcmp(argv[i], "-o") == 0) {
      PanDirectionOffset = atof(argv[++i]);
      printf("PanDirectionOffset: %f\n",PanDirectionOffset );
    }
    else if (strcmp(argv[i], "-v") == 0) {
      verbose = 1;
    }
    else {
      printf("App Server: Unknown option\n");
      print_help();
      exit(0);
    }
  }


  #ifdef USE_BBB_PWM
  printf("Using servo motors.. \n");
  #endif

  
  IvyInit ("anttrUI", "Anthenna Tracker READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_FLIGHT_PARAM_STATUS, NULL, "ground FLIGHT_PARAM (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_TRACKER_PARAM_STATUS, NULL, "ground TRACKER_GPS (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_tracker_ANT_TRACK, NULL, "(\\S*) ANT_TRACKER (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_tracker_NEW_AC, NULL, "ground NEW_AIRCRAFT (\\S*)");
  IvyBindMsg(on_tracker_GET_CONFIG, NULL, "(\\S*) ground CONFIG (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_set_tracker, NULL, "(\\S*) SET_ANT_TR (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_stop_tracker, NULL, "(\\S*) STOP_ANT_TR (\\S*)");
  


  #ifndef USE_BBB_PWM
  IvyBindMsg(on_tracker_set_zero, NULL, "(^\\S*) ANT_TRACKER_SET_ZERO (\\S*)");
  #endif
  init_bbb_onboard_modules(verbose);


  //This is temporary 
  DevStatus[0].used = 1;
  DevStatus[0].device_id = 255;
  //43.564617, 1.481031
  DevStatus[0].altitude = 438;
  DevStatus[0].longitude = 33.0556205;
  DevStatus[0].latitude = 35.0560208;
  //End of temp code
  
  //Need to
  strcpy(DevNames[0].name, TRACKER_NAME);
  DevNames[0].device_id=TRACKER_ID;

  //Start ivy bus
  IvyStart(IvyBus);
  
  //Create & Start main loop
  GMainLoop *loop = g_main_loop_new(NULL, FALSE);
  g_main_loop_run(loop);
  
  return 0;
}

int check_device_name(int dev_id) {

  //Search DevNames struct
  int i=0;
  while ( (i< MaxNumConfNames) && (DevNames[i].device_id > 0) ) {
        if ( DevNames[i].device_id == dev_id ) {

	  return 1;
        }
      i++;
      //if () break;
      }

  //no device found request for device config
  if (dev_id>0) request_ac_config(dev_id);
  return -1;

}

// Returns DevStatus array index of send device id (if device exists).
// If device not found, adds it to the list and then send its new array index. If MaxNumDevices reached returns -1
int Get_Dev_Ind(int device_id) {

  //check whether device is recorded in DevNames
  if (check_device_name(device_id) < 0) {
    //device not in list
    return -1;
  }

  int i=0;


  //Search list
  while ( (DevStatus[i].used != 0) && (i < (2*MaxNumDevices)) ) {
    if ( DevStatus[i].device_id == device_id ) return i;
  i +=1;
  }

  if (i >= (2*MaxNumDevices) ) { MaxNumDevicesReached =-1; return -1;}	//MaxNumDevices reached return -1
  //Add new entry to list and return entry index;
  DevStatus[i].device_id = device_id;
  //raise .used flag
  DevStatus[i].used=1;  
  //Return index
  return i;
}

//Returns d array index of send device id if device exist in controlled aircrafts. If MaxNumDevices reached returns -1
int Check_Controlled_Aircraft_List (int Aircraft_Id) {

  int i=0;
  //Search tracker control list
  while ( (d[i].used != 0) && (i < MaxNumDevices) ) {
    if ( d[i].aircraft_id == Aircraft_Id ) return i;
  i +=1;
  }
  //return -1 if max numb of devices reached
  if ( i >= (MaxNumDevices) )  MaxNumDevicesReached =-1;
  return -1;
}

//Sends ivy msg for given index of d
void Send_ContList_Msg (int ContListNumb) {
  //get aircraft id
  int AircraftID = d[ContListNumb].aircraft_id;
  //get tracker id
  int TrackerID  = d[ContListNumb].tracker_id;
  //cancel if max numb of device error raised
  if ( TrackerID <0 || AircraftID <0 ) return;
  //send ivy msg
  IvySendMsg("ground ANT_TRACKER_DATA %d %d %f %f %f %f %f %f" ,
        TrackerID, d[ContListNumb].mode ,
        d[ContListNumb].pan_angle, d[ContListNumb].tilt_angle,
        DevStatus[Get_Dev_Ind(AircraftID)].latitude, DevStatus[Get_Dev_Ind(AircraftID)].longitude, DevStatus[Get_Dev_Ind(AircraftID)].altitude,
        DevStatus[Get_Dev_Ind(TrackerID)].altitude );

}

//Returns d array index of send device id, if device exist in trackers.
//If MaxNumDevices reached returns -1
//If device is not at tracker list, adds it to list and send new id,
int Check_Controlled_Tracker_List (int Tracker_Id) {
  int i=0;
  //check if device is already in control list
  while ( (d[i].used != 0) && (i < MaxNumDevices) ) {
    if ( d[i].tracker_id == Tracker_Id ) return i;
      i +=1;
    }
  //if MaxNumDevices reached return -1
  if ( i >= (MaxNumDevices) ) { MaxNumDevicesReached =-1; return -1; }
  //Add it to control list & return index if it
  d[i].used = 1;
  d[i].tracker_id = Tracker_Id;
  d[i].aircraft_id = -1;
  d[i].mode = 0;
  
  //Debug purposes
  if (verbose) {
  printf("New tracker added.. (ID=%d)\n",Tracker_Id);
  }

  return i;
}

