/*
If anthenna tracker will be used with a DC motor & encoder pair this file should be modified accordingly.
This library reads encoder and drives the DC motor (with pwm)
See corresponding libraries for documentation. 

Library to generate pwm output.. 
Black lib: 
Yigit Yuce
https://github.com/yigityuce/BlackLib

Interfacing Beaglebone Black's eQEP modules. 
BBB-eQEP
James Zapico
https://github.com/jadedanemone/BBB-eQEP
*/

#include <iostream>
#include <string>
#include <unistd.h>
#include "libraries/BlackLib/BlackLib.cpp"
#include "include/bbb-eqep.h"
using namespace BBB;

//Optic encoders
#define TILT_ENC_PATH eQEP1	    //eQEP1 (A=P8_35 B=P8_33)
#define TILT_DIR_PIN 	GPIO_48   //GPIO1_16 = (1x32) + 16 = 48 (P9_15)
#define TILT_PWMOUT_PIN	P9_14   //See BlackLib documentation

#define PAN_ENC_PATH 	eQEP0     //eQEP2 (A=P8_41  B=P8_42)
#define PAN_DIR_PIN 	GPIO_60   //GPIO1_28 = (1x32) + 28 = 60 (P9_12)
#define PAN_PWMOUT_PIN	P9_16   //See Blacklib documentation

#define PWM_OUT_DCYCLE	5000000 //in nanoseconds

#define PAN 1
#define TILT 0

#define CW high    //clockwise
#define CCW low    //Conter clockwise

#define PAN_ENC_COUNTS	942803     //number of encoder counts per turn
#define TILT_ENC_COUNTS	502828     //number of encoder counts per turn

#define GOTO_TOLERANCE	1          //Tolerance for target angle. 
#define MANUAL_TURN_STEP 1

//Safety margins for tilt engine
#define TILT_MIN 0
#define TILT_MAX 180


int ob_verbose = 0;
int TrackerInProgress = 0; 
int PanInited = 0; 
int TiltInited = 0; 

//initialization of control BBB-eQEP pins
eQEP TiltEncoder(TILT_ENC_PATH);
eQEP PanEncoder(PAN_ENC_PATH);

BlackGPIO	TILT_DIR_GPIO(TILT_DIR_PIN,output);
BlackPWM 	TILT_PWM_GPIO(TILT_PWMOUT_PIN); // initialization pwm with second led

BlackGPIO	PAN_DIR_GPIO(PAN_DIR_PIN,output);
BlackPWM	PAN_PWM_GPIO(PAN_PWMOUT_PIN); // initialization pwm with second led



float PanAxisReqAng, TiltAxisReqAng;
int32_t PanAxisReqPos, TiltAxisReqPos;
int32_t PanAxisPosMargin,TiltAxisPosMargin;
float PanAxisReqSpeed, TiltAxisReqSpeed;
int PanInProgress,TiltInProgress;

int32_t TiltAxisPosition,PanAxisPosition;


int32_t read_axis_pos(int axis) {
  if ( axis == TILT ) {
    return TiltEncoder.getPosition();
  }
  else if ( axis == PAN ) {
    int32_t PanEncoderPrePosition;
    PanEncoderPrePosition = PanEncoder.getPosition();

    //reset encoders if they are out of 0-360 range
    if (PanEncoderPrePosition > PAN_ENC_COUNTS) {
      PanEncoderPrePosition = PanEncoderPrePosition - PAN_ENC_COUNTS;
      PanEncoder.setPosition(PanEncoderPrePosition);
      if (ob_verbose) printf("Pan encoder reseted. (+)\n");
    }

    if ( PanEncoderPrePosition < ((-1)*PAN_ENC_COUNTS)) {
      PanEncoderPrePosition = PanEncoderPrePosition + PAN_ENC_COUNTS;
      PanEncoder.setPosition(PanEncoderPrePosition);
      if (ob_verbose) printf("Pan encoder reseted. (-)\n");
    }

    if (PanEncoderPrePosition < 0 ) {
      PanEncoderPrePosition = PAN_ENC_COUNTS - abs(PanEncoderPrePosition) ;
    }

    return PanEncoderPrePosition;
  }
  else return -1;
}

int set_axis_dir(int axis, gpio_value direction) {
  if ( axis == TILT ) {
    TILT_DIR_GPIO.setValue(direction);
    return 0;
  }
  else if ( axis == PAN ) {
    PAN_DIR_GPIO.setValue(direction);
    return 0;
  }
  return -1;
}

int set_motor_speed(int axis,float speed) {
  if ( axis == TILT ) {
    TILT_PWM_GPIO.setDutyPercent(speed);
    return 0;
  }
  else if ( axis == PAN ) {
    PAN_PWM_GPIO.setDutyPercent(speed);
    return 0;
  }
  return -1;
}


int go_to(int axis, float angle, float speed) {

  //read position of axis

  if  (!TrackerInProgress && PanInited && TiltInited) {
    set_motor_speed(PAN,0);
    set_motor_speed(TILT,0);
    if (ob_verbose) printf("Not TrackerInProgress!!\n");    
    return 1;
  }

  int32_t RequiredPosition;
  int32_t AxisMargin;
  int32_t AxisPosition;
  AxisPosition= read_axis_pos(axis);

  if ( axis == TILT ) {
    //Convert angle to encoder count
    RequiredPosition= (TILT_ENC_COUNTS/360)*angle;
    //Axis margin
    AxisMargin = (TILT_ENC_COUNTS/360) * GOTO_TOLERANCE;
    }
  else if ( axis == PAN ) {
    //Convert angle to encoder count
    RequiredPosition= (PAN_ENC_COUNTS/360)*angle;
    //Axis margin
    AxisMargin = (PAN_ENC_COUNTS/360) * GOTO_TOLERANCE;
    }
  else {
    return -1;
  }

  
  if (axis == TILT) {

    if ( !( (AxisPosition <= (RequiredPosition + AxisMargin) ) && (AxisPosition >= ( RequiredPosition - AxisMargin) ) ) )
      {
    
          //Set motor direction    
          if (AxisPosition>RequiredPosition) {
            set_axis_dir(axis,CCW);
          }
          else {
            set_axis_dir(axis,CW);
          }
        
        //Set motor speed
        set_motor_speed(axis,speed);
        AxisPosition= read_axis_pos(axis);
        //printf("\rTilt Position:%d \t Pan Position:%d", TiltEncoder.getPosition(), PanEncoder.getPosition());
        //std::cout << "[eQEP " << axis << "] Position = " << AxisPosition << std::endl;
        
        return 0;
      }
      else {
        //Stop motor
        TiltInited = 1;
      set_motor_speed(axis,0);      
      return 1;
      }
  }else {
  //**********************PAN DIY
    //printf("Axis Pos:%d \tRequired:%d \n",AxisPosition,RequiredPosition);
    if ( !( (AxisPosition <= (RequiredPosition + AxisMargin) ) && (AxisPosition >= ( RequiredPosition - AxisMargin) ) ) )
        {
           
            //Set motor direction    
            if (AxisPosition>RequiredPosition) {
              if (( (AxisPosition-RequiredPosition) >= (PAN_ENC_COUNTS/2)) ) {
                set_axis_dir(axis,CW);
              }
              else {
                set_axis_dir(axis,CCW);
              }
              
            }
            else {
              if (( (RequiredPosition - AxisPosition) >= (PAN_ENC_COUNTS/2) )) {
                set_axis_dir(axis,CCW);
              }
              else {
                set_axis_dir(axis,CW);
              }
      
            }
          
          //Set motor speed
          set_motor_speed(axis,speed);
          AxisPosition= read_axis_pos(axis);
          //printf("\rTilt Position:%d \t Pan Position:%d", TiltEncoder.getPosition(), PanEncoder.getPosition());
          //std::cout << "[eQEP " << axis << "] Position = " << AxisPosition << std::endl;
          
          return 0;
        }
        else {
          //Stop motor
          PanInited = 1; 
          set_motor_speed(axis,0);
          return 1;
        }


  //***********************END PANDIY
  }


}


gboolean tilt_go_to(gpointer data) {
 // printf("here comes the butterflies\n");

  TiltInProgress=1;

  TiltAxisPosition = read_axis_pos(TILT);
  
  if (ob_verbose) {
    float ReqA; 
    ReqA = TiltAxisPosition / (TILT_ENC_COUNTS /360);
    if (ob_verbose) printf("Tilt pos (int) :\t%d \t Angle:\t %f Required:\t %f \n",TiltAxisPosition, ReqA, TiltAxisReqAng);
    fflush(stdout);
  }

  if (TiltAxisReqAng < TILT_MIN ) {
    if (ob_verbose) printf("Tilt at min limit!. TiltAxisReqAng: %f \n", TiltAxisReqAng);
    TiltAxisReqAng=TILT_MIN;    
  }

  if (TiltAxisReqAng > TILT_MAX ) {
    if (ob_verbose) printf("Tilt at max limits!. TiltAxisReqAng: %f \n", TiltAxisReqAng);
    TiltAxisReqAng=TILT_MAX;    
  }
  
  /*
  if (TiltAxisReqAng < TILT_MIN ) {
    printf("Tilt at min limit!. \n");
    return FALSE;
  }

  if (TiltAxisReqAng > TILT_MAX ) {
    printf("Tilt at max limits!. \n");
    return FALSE;
  }
  */


  //Check Tilt Position
  if ( go_to( TILT, TiltAxisReqAng, TiltAxisReqSpeed ) )  {
  	TiltInProgress=0;
    
    if (ob_verbose) {
     printf("Tilt reached dest. \n");
     fflush(stdout);
    }
  	
  	return FALSE;
   
  }else {
  	return TRUE;
  }
    
}

gboolean pan_go_to(gpointer data) {

  PanInProgress=1;

  //go_to(PAN, PanAxisReqAng, PanAxisReqSpeed );

  PanAxisPosition = read_axis_pos(PAN);
  
  if (ob_verbose) {
    float ReqA; 
    ReqA = PanAxisPosition / (PAN_ENC_COUNTS/360);
    printf("Pan pos (int) :\t%d \t Angle:\t %f Required:\t %f \n",PanAxisPosition, ReqA, PanAxisReqAng);
    fflush(stdout);
  }
  

  //Check Tilt Position
  if ( go_to( PAN, PanAxisReqAng, PanAxisReqSpeed ) )  {
  	PanInProgress=0;
    if (ob_verbose) {
    printf("Pan reached dest. \n");
    fflush(stdout);
    }
  	
  	return FALSE;
   
  }else {
  	return TRUE;
  }
 
}


void on_ant_tr_goto(IvyClientPtr app, void *user_data, int argc, char *argv[]) {
 //ANT_DRIVER_GOTO <axis> <angle> <speed>
  
  if (ob_verbose) {
  printf("Axis:\t%d\n",atoi(argv[1]));
  printf("Angle:\t%f\n",atof(argv[2]));
  printf("Speed:\t%f\n",atof(argv[3]));
  fflush(stdout);
  }
  
  if (atoi(argv[1])==PAN){
    PanAxisReqAng=  atof(argv[2]);
    PanAxisReqPos= (PAN_ENC_COUNTS/360)*PanAxisReqAng;
    PanAxisReqSpeed= atof(argv[3]);
    if (!PanInProgress) {
      //marche marche!!!!!!
      g_timeout_add(100, pan_go_to, NULL);
    }
  }

  if (atoi(argv[1])==TILT){
    TiltAxisReqAng=  atof(argv[2]);
    TiltAxisReqPos= (TILT_ENC_COUNTS/360)*TiltAxisReqAng;
    TiltAxisReqSpeed= atof(argv[3]);
    if (!TiltInProgress) {
      //marche marche!!!!!!
      g_timeout_add(100, tilt_go_to, NULL);
    }
  }
  //set_motor_speed(atoi(argv[1]),atof(argv[2]));
  //go_to(atoi(argv[0]),atof(argv[1]),atof(argv[2]));
}


void on_turn_tracker(IvyClientPtr app, void *user_data, int argc, char *argv[]) {
 //ANT_DRIVER_GOTO <axis> <angle> <speed>
  
  int mAxis;
  float mDir; 
  mAxis = atoi(argv[1]);
  mDir = atof(argv[2]);

  TrackerInProgress = 1;
  
  if (ob_verbose) {
    printf("\nAxis:\t%d\n",mAxis);
    printf("Direction:\t%f\n",mDir);
    fflush(stdout);
  }

  if ( mAxis == PAN){
    PanAxisReqAng =  PanAxisReqAng + mDir;
    PanAxisReqPos = (PAN_ENC_COUNTS/360)*PanAxisReqAng;
    
    if (!PanInProgress) {
      //marche marche!!!!!!
      g_timeout_add(100, pan_go_to, NULL);
    }
  }

  if ( mAxis ==TILT){
    TiltAxisReqAng=  TiltAxisReqAng + mDir;
    TiltAxisReqPos= (TILT_ENC_COUNTS/360)*TiltAxisReqAng;
    
    if (!TiltInProgress) {
      //marche marche!!!!!!
      g_timeout_add(100, tilt_go_to, NULL);
    }
  }
}

void on_tracker_set_zero(IvyClientPtr app, void *user_data, int argc, char *argv[]) {

  
  if (atoi(argv[0])== TRACKER_ID) {
    if (ob_verbose) {
      printf("Ant tracker setting zero position for encoders:\n");
      fflush(stdout);      
    }
  
    int32_t ZeroPos;
    ZeroPos = 0;
    TiltEncoder.setPosition(ZeroPos);
    PanEncoder.setPosition(ZeroPos);

    if (ob_verbose) {
      printf("Encoders reseted.\nTilt pos: %d \tPan pos: %d\n",TiltEncoder.getPosition(), PanEncoder.getPosition());
      fflush(stdout);
      }
  }
 
}


void set_bbb_modules() {
 // printf("here comes the butterflies\n");
  
  if (ob_verbose) {
    printf("Setting set_bbb_modules.. \n");
    fflush(stdout);
  }


  TILT_DIR_GPIO.setValue(high);
  TILT_PWM_GPIO.setDutyPercent(0);
  
  PAN_DIR_GPIO.setValue(high);
  PAN_PWM_GPIO.setDutyPercent(0);

  if (ob_verbose) {
    printf("StartUp encoder positions= Tilt pos: %d \tPan pos: %d\n",TiltEncoder.getPosition(), PanEncoder.getPosition());
    fflush(stdout);
  }

  
  g_timeout_add(100, pan_go_to, NULL);
  g_timeout_add(100, tilt_go_to, NULL);
     
}

void init_bbb_onboard_modules(int vb) {

  //Initial setup of control pins
  ob_verbose = vb;

  if (ob_verbose) {
  printf("Init_bbb_onboard_modules.. \n");
  fflush(stdout);
  }


  TILT_PWM_GPIO.setPeriodTime(PWM_OUT_DCYCLE);
  PAN_PWM_GPIO.setPeriodTime(PWM_OUT_DCYCLE);

  PanAxisReqAng = 0;
  TiltAxisReqAng = 0;
  PanAxisReqSpeed = 100;
  TiltAxisReqSpeed = 100;

  set_bbb_modules();


}


void set_tracker_motors(float PanDegree, float PanSpeed, float TiltDegree, float TiltSpeed ) {

    PanAxisReqAng =  PanDegree;    
    PanAxisReqPos= (PAN_ENC_COUNTS/360)*PanAxisReqAng;
    PanAxisReqSpeed= PanSpeed;  
    if (!PanInProgress) {
      //marche marche!!!!!!
      g_timeout_add(100, pan_go_to, NULL);
    }
 
    TiltAxisReqAng=  TiltDegree;
    TiltAxisReqPos= (TILT_ENC_COUNTS/360)*TiltAxisReqAng;
    TiltAxisReqSpeed= TiltSpeed;
    if (!TiltInProgress) {
      //marche marche!!!!!!
      g_timeout_add(100, tilt_go_to, NULL);
    }


}



