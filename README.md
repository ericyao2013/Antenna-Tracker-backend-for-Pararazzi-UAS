PPRZ BeagleBone Antenna Tracker Control Module Backend
===========================
This project is an intend to create an antenna tracker for [Paparazzi UAS][paparazzi_link] sytem with a BeagleBoneBlack (BBB) development card. 
It uses a pan tilt mechanism with DC motor-optical encoder, or servo motors to track desired AC.

##Install
Compiling the code is fairly straightforward. 

```shell
git clone https://github.com/savass/pprz_ant_tracker_backend.git
cd pprz_ant_tracker_backend
``` 
For servo controlled pan-tilt mechanisms;

```shell
make ant_tr_pwm
```
which will create `ant_tr_pwm` file. 
Also for dc-motor (with optic encoder) controlled pan-tilt mechenisms;

```shell
make ant_tr_dc
```
which will create `ant_tr_dc` file. 

If on board gps module will be used;
```shell
make gpsd2ivy
```
will create modified gpsd2ivy app. 


## Using Build in Optical Encoders of BBB

BBB has build in eQep modules which can be used to read encoder values. No additional harware required.

More information for compiling the kernel with eQEP driver can be found at [Robert Nelson's github page.][RobertCNelson_kernel_link] (Kernel 3.8 is used for this project)

Please see [James Zapico's BBB-eQEP][BBB-eQEP] github page to see how eQEP modules are installed and loaded. 

Create `ant_tr_dc` file for dc-motor controlled pan-tilt mechanisms.

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


##Using Servo motors

Create `ant_tr_pwm` file for servo controlled pan-tilt mechanisms.. BBB pwm output has a default 5ms duty cycle. 
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

##OnBoard GPS Module

This module uses a slightly modified version of `gpsd2ivy.c` file which can be found in `paparazzi/sw/ground_segment/tmtc/gpsd2ivy.c`. Please see [gpsd][gpsd_link] documentation for a proper o/b gps setup. 
Simply gpsd2ivy reads gps values from gpsd deamon and pushes them to ivybus and this module reads its own position from the messages that gpsd2ivy creates. 

##Using Multiple Trackers
If there will be more than one tracker in the system assign differant `TRACKER_ID` values for each trackers. `TRACKER_ID` value should be modified in  `gpsd2ivy.c` and `main.c` files.  

##Mechanical Installation
There is no attitute control in this module. So when the module ran it should be heading to north. Especially with servo mechanisms this is the 'zero point'. This point can be modified with `-o` argument. 
For instance, if the application is started with:

```shell
./ant_tr_pwm -o 90
```

command, calculated pan values will be offsetted 90 degrees. In this case 'zero point' will be west. 

##Mechanical Installation
There are a few command line arguments that can be passed to the app. 
`-o`: Adds offset to calculated pan value. (Ex: `-o 180`)

`-b`: Defines the ivybus (Ex: `-b 192.168.4.255`)

`-v`: Verbose mode

`-h` or `--help`: Shows the app usage. 



###TODO..
- Code can be simplified to hold only one AC parameter. 
- Attitide control can be added direction sensing.
- Multi tracker usage should be improved to get rid of TRACKER_ID modifications. 

 [BBB-eQEP]: https://github.com/jadedanemone/BBB-eQEP
 [RobertCNelson_kernel_link]: https://github.com/RobertCNelson/linux-dev
 [gpsd_link]: http://www.catb.org/gpsd
 [paparazzi_link]: https://github.com/paparazzi/paparazzi