// Arduino User Config

/*
  You can enter Pairing Mode by holding down the reset button until the LED will start pulsing quickly
  You may have to hold the button for up to 30sec.

  To Switch between AP and SMART pairing mode, release the pairing button and Press and hold again until the LED begins to flash at a different pattern

  LED Patterns:
    - Fast Pulsing:     SMART Pairing Mode
    - Slow Pulsing:     AP Pairing Mode
    - Steady Glow:      TUYA Paired and Connected
    - Triple Flashing:  TUYA Module ERROR
*/

/////////////////////////////////
/////////// TUYA Setup //////////
/////////////////////////////////

#define TUYA_PID "*****" // << This can be found in your TUYA account
#define TUYA_MCU_Version "1.0.0"    // << This version does not really matter. You can put anything you like. It can be a useful tool to keep track of revisions

#define TUYA_WiFi_ModeSetPin 3 // << configure pin number for the button you'll use to Pair your TUYA Device

#define TUYA_ModuleResetPin 7 // << configure pin number for Arduino to reset the TUYA module in the case of mis-communication or module error

#define StatusLedPin 6 // << pin Number of status LED. Make sure pin is a 'PWM' and NOT FROM TIMER 2 (5, 6, 9, or 10. NOT 3 or 11)

#define DPID_UpdateInterval 10000  // << set update DPID's interval in milliseconds

/////////////////////////////////
///////// Stepper Setup /////////
/////////////////////////////////

// Setup stepper stepping pin and direction control pin (these pins will be shared with all steppers):
#define StepperStepPin 8
#define StepperDirPin 9

// These are some defaults for each stepper:
#define StepperMaxSpeedDefault 1000 // << max speed in steps-per-second
// for acceleration and deceleration, larger number means faster accel/deccel, lower number means slower accel/deccel (in steps-per-second)
#define StepperAccelDefault 1500
#define StepperDeccelDefault 5000  //  this is the deceleration speed of when the user hits 'STOP' in the App. This value is usually Higher than normal acceleration

// this is were you will declare all of your steppers and define profile for each of them:
// The DPID's can be found in your TUYA Account

// {StepperPosition_DPID, StepperCommand_DPID, StepperEnablePin, StepperMaxStep, StepperSpeed, StepperAccel, StepperStopDeccel}:
#define StepperConfig {{101, 102, A0, 3000, StepperMaxSpeedDefault, StepperAccelDefault, StepperDeccelDefault}, /* << repeat for each individual stepper. Leave a backslash '\' at the end of each line except for the last line*/ \
                       {103, 104, A1, 3000, StepperMaxSpeedDefault, StepperAccelDefault, StepperDeccelDefault}, \
                       {105, 106, A2, 3000, StepperMaxSpeedDefault, StepperAccelDefault, StepperDeccelDefault}, \
                       {107, 108, A4, 3000, StepperMaxSpeedDefault, StepperAccelDefault, StepperDeccelDefault}, \
                       {109, 110, A5, 3000, StepperMaxSpeedDefault, StepperAccelDefault, StepperDeccelDefault}}

///////////////////////////////
///////// Servo Setup /////////
///////////////////////////////

#define Servo_DPID 115  // << Servo DPID can be found in your TUYA Account
#define ServoPin 4 
