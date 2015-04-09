#include <iostream>
#include <string>
#include <unistd.h>
#include "libraries/BlackLib/BlackLib.cpp"
#include "include/bbb-eqep.h"

using namespace BBB;
using namespace std;

int TrackerInProgress = 0; 
float PanAxisReqAng, TiltAxisReqAng;
/*FOR eQEO definations  see readme file
 * At this point we need to be sure that these encoders are loaded in the device three
 */

//See & change libraries/eQEP/eqep.h for eQEP definations..
#define TILT_DIR_PIN 	GPIO_48							//GPIO1_16 = (1x32) + 16 = 48 (P9_15)
#define TILT_PWMOUT_PIN	P9_14							//See BlackLib documentation

#define PAN_DIR_PIN 	GPIO_60							//GPIO1_28 = (1x32) + 28 = 60 (P9_12)
#define PAN_PWMOUT_PIN	P9_16							//See Blacklib documentation

// #define PWM_OUT_DCYCLE	5000000							//in nanoseconds
#define PWM_OUT_DCYCLE 20000000

#define PAN 1
#define TILT 0

#define PAN_SERVO_MIN_PWM  620000
#define PAN_SERVO_NOM_PWM  1500000
#define PAN_SERVO_MAX_PWM  2380000
#define PAN_SERVO_MIN_DEG  0
#define PAN_SERVO_MAX_DEG  360
#define PAN_SERVO_REVERSED 1

#define TILT_SERVO_MIN_PWM  1496000
#define TILT_SERVO_NOM_PWM  1496000
#define TILT_SERVO_MAX_PWM  1705000
#define TILT_SERVO_MIN_DEG  0
#define TILT_SERVO_MAX_DEG  90
#define TILT_SERVO_REVERSED 0

int ob_verbose;

BlackPWM 	TILT_PWM_GPIO(TILT_PWMOUT_PIN); // initialization pwm with second led
BlackPWM	PAN_PWM_GPIO(PAN_PWMOUT_PIN); // initialization pwm with second led
//Read axis position

uint32_t calc_pan_duty_value(float ReqDegree) {
  uint32_t PanDutVal;

  if (PAN_SERVO_REVERSED) ReqDegree = ReqDegree * (-1);

  PanDutVal = PAN_SERVO_NOM_PWM + (((PAN_SERVO_MAX_PWM - PAN_SERVO_NOM_PWM)/(PAN_SERVO_MAX_DEG - PAN_SERVO_MIN_DEG )) * ReqDegree);
  //printf("Pan duty value= %d\n", PanDutVal);
  if (PanDutVal > PAN_SERVO_MAX_PWM ) return PAN_SERVO_MAX_PWM;
  if (PanDutVal < PAN_SERVO_MIN_PWM ) return PAN_SERVO_MIN_PWM;
  return PanDutVal;

}

uint32_t calc_tilt_duty_value(float ReqDegree) {
  uint32_t TiltDutVal;
  if (TILT_SERVO_REVERSED) {
    ReqDegree = ReqDegree * -1;
  }
  TiltDutVal = TILT_SERVO_NOM_PWM + (((TILT_SERVO_MAX_PWM - TILT_SERVO_NOM_PWM)/(TILT_SERVO_MAX_DEG - TILT_SERVO_MIN_DEG )) * ReqDegree);
  //printf("Tilt duty value= %d\n", TiltDutVal);
  if (TiltDutVal > TILT_SERVO_MAX_PWM) return TILT_SERVO_MAX_PWM;
  if (TiltDutVal < TILT_SERVO_MIN_PWM) return TILT_SERVO_MIN_PWM;
  return TiltDutVal;

}


int go_to_deg(int axis, float ReqAngle) {

  if ( axis == PAN ) {
    PAN_PWM_GPIO.setDutyValue(calc_pan_duty_value(ReqAngle));
    PanAxisReqAng = ReqAngle ;
    //printf("Pan duty appiled (deg): %f\n",ReqAngle );
    return 0;
  }

  if ( axis == TILT ) {
    TILT_PWM_GPIO.setDutyValue(calc_tilt_duty_value(ReqAngle));
    TiltAxisReqAng = ReqAngle;
    //printf("Tilt duty appiled (deg): %f\n", ReqAngle);
    return 0;
  }

  return -1;
}

//Set servo duty value (in anoseconds)
int go_to_int(int axis, uint32_t ReqDutyValue) {

  if ( axis == PAN ) {
    PAN_PWM_GPIO.setDutyValue(ReqDutyValue);
    //printf("Pan duty appiled %d.\n", ReqDutyValue);
    return 0;
  }

  if ( axis == TILT ) {
    TILT_PWM_GPIO.setDutyValue(ReqDutyValue);
    printf("Tilt duty appiled %d.\n", ReqDutyValue);
    return 0;
  }

  return -1;
}

void init_bbb_onboard_modules(int vb) { 
  ob_verbose = vb;

  if (ob_verbose) {
    printf("Initing bbb pwm modules.. \n");
    fflush(stdout);
  }

  //Initial setup of control pins
  if (TILT_PWM_GPIO.setPeriodTime(PWM_OUT_DCYCLE)) {
    printf("Tilt pwm setup completed.\n");
  } else printf("!! Tilt pwm setup error..\n");
  TILT_PWM_GPIO.setDutyValue(PAN_SERVO_NOM_PWM);


  if (PAN_PWM_GPIO.setPeriodTime(PWM_OUT_DCYCLE)) {
    printf("Pan pwm setup completed.\n");
  }else printf("!! Pan pwm setup error..\n");
  PAN_PWM_GPIO.setDutyValue(PAN_SERVO_NOM_PWM);

 TiltAxisReqAng = 0;
 PanAxisReqAng = 0; 
}


int go_to(int axis, float angle, float speed) {
 go_to_deg(axis,angle); 
 return 1;

}

void set_tracker_motors(float PanDegree, float PanSpeed, float TiltDegree, float TiltSpeed ) {

    //ignore the speeds with servos
    go_to_deg(PAN, PanDegree);
    go_to_deg(TILT, TiltDegree);
    

}


//Ivy bindings

void on_ant_tr_goto(IvyClientPtr app, void *user_data, int argc, char *argv[]) {
 //ANT_DRIVER_GOTO <axis> <angle> <speed>
  
  if (ob_verbose) {
  printf("Axis:\t%d\n",atoi(argv[1]));
  printf("Angle:\t%f\n",atof(argv[2]));
  printf("Speed:\t%f\n",atof(argv[3]));
  fflush(stdout);
  }

  go_to(atoi(argv[1]), atof(argv[2]), atof(argv[3]) ); 
  
}


void on_turn_tracker(IvyClientPtr app, void *user_data, int argc, char *argv[]) {
 //ANT_DRIVER_GOTO <axis> <angle> <speed>
  
  int mAxis;
  float mDir; 
  mAxis = atoi(argv[1]);
  mDir = atof(argv[2]);
  
  if (ob_verbose) {
    printf("\nAxis:\t%d\n",mAxis);
    printf("Direction:\t%f\n",mDir);
    fflush(stdout);
  }

  if ( mAxis == PAN){
    go_to (PAN, (PanAxisReqAng+mDir), 100);
  }

  if ( mAxis == TILT){
    go_to (TILT, (TiltAxisReqAng+mDir), 100);
  }
}



