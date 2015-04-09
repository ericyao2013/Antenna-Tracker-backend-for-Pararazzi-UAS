PPRZ BeagleBone Antenna Tracker Control Module Backend
===========================
This project is an intend to create an antenna tracker for [Paparazzi UAS][paparazzi_link] sytem with a BeagleBoneBlack (BBB) development card. 
It uses a pan tilt mechanism with DC motor-optical encoder, or servo motors to track desired AC.

## Using Build in Optical Encoders of BBB

BBB has build in eQep modules which can be used to read encoder values. No additional harware required.

More information for compiling the kernel with eQEP driver can be found at [Robert Nelson's github page.][RobertCNelson_kernel_link] (Kernel 3.8 is used for this project)

Please see [James Zapico's BBB-eQEP][BBB-eQEP] github page to see how eQEP modules are installed and loaded. 


In this case please remove `-DUSE_BBB_PWM` parameter from Makefile. 

Modify `bbb_ob_control.h` file to define encoder parameters - `PAN_ENC_COUNTS` and `TILT_ENC_COUNTS`. These values should be the number of encoder counts per one turn (360 degrees). 

###Control Outputs
- Tilt Motor:
  - Optical encoder input: `eQEP1` (A=P8_35 B=P8_33)
  - Motor pwm out: `P9_14`
  - Motor direction: `P9_15`
- Pan Motor:
  - Optical encoder input: `eQEP0` (A=P8_41  B=P8_42)
  - Motor pwm out: `P9_16`
  - Motor direction: `P9_12`

These values can be changed via `bbb_ob_control.h` file. 


## Using Servo motors

By default this module uses servo motors on a pan tilt mechanism. Code should be compiled with `-DUSE_BBB_PWM` parameter to use servo motors. BBB pwm output has a default 5ms duty cycle. 
To use servo motors we should use a slightly modified device tree for pwm outputs. 

##Controlling Over IvyBus

Antenna tracker can be controlled over ivybus with these messages;

To start tracking of an AC, simply pass the `<TRACKER_ID>` and `<AC_ID>` parameters thru ivy.. 

```
SET_ANT_TR <TRACKER_ID> <AC_ID> <TRACKER_LAT> <TRACKER_LON> <TRACKER_ALT>
```

`<TRACKER_LAT>`,`<TRACKER_LON>` and `<TRACKER_ALT>` can be ignored if an onboard gps is being used. 

```
STOP_ANT_TR <TRACKER_ID>
```
 obviously stops tracking. 



If DC Motor with encoder option is being used;
```
ANT_TRACKER_SET_ZERO <TRACKER_ID>
```
can be used to set zero positions for pan and tilt motor. 

#OnBoard GPS Module

This module uses a slightly modified version of `gpsd2ivy.c` file which can be found in `paparazzi/sw/ground_segment/tmtc/gpsd2ivy.c`. Please see [gpsd][gpsd_link] documentation for a proper o/b gps setup. 
Simply gpsd2ivy reads gps values from gpsd deamon and pushes them to ivybus and this module reads its own position from the messages that gpsd2ivy creates. 

#Using Multiple Trackers
If there will be more than one tracker in the system assign differant `TRACKER_ID` values for each trackers. `TRACKER_ID` value should be modified in  `gpsd2ivy.c` and `main.c` files.  



 [BBB-eQEP]: https://github.com/jadedanemone/BBB-eQEP
 [RobertCNelson_kernel_link]: https://github.com/RobertCNelson/linux-dev
 [gpsd_link]: http://www.catb.org/gpsd
 [paparazzi_link]: https://github.com/paparazzi/paparazzi