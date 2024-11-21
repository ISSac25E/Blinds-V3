/*
Blind V3
Release Version: 1.0.1

Changes:
  - Update DPID's only when necessary.
    Updating all DPID's too often resulted in enormous app stutter and freezing
  - Status LED now runs on ISR to prevent stuttering when using Serial Communication
  - Changed PID. See "config.h"
  - Change Blinds Command Enum Order from (Open, Stop, Close) to (Close, Stop, Open)

ToDO:
  - clean-up timing with dpid send
  - implement feedback with servo e.g.('ready', 'running', 'run')
  - communication via RTX with Arduino Servo

Possible Future Features:
  - Advanced Accel Profiling with Steppers

////////////////////////////////////////////////////////////////////////////
// !!! Setup is done through "config.h" located in the same directory !!! //
////////////////////////////////////////////////////////////////////////////
*/

#include "Arduino.h"
#include "config.h"

// include all required core:
// Tuya MCU Serial SDK:
#include "Core/Tuya WiFi MCU Serial SDK/Tuya_WiFi_MCU_SDK_2.0.0/src/TuyaWifi.h"

// Stepper Driver and Stepper Queue:
#include "Core/StepperDriver/StepperDriver_2.1.3.h"
#include "Core/StepperQueue/StepperQueue_1.0.0.h"

// PinPort for fast digitalWrites and Reads:
#include "Core/PinPort/PinPort_1.0.0.h"

// LED Sequence builder and Macros:
#include "Core/SequenceBuild/SequenceBuild_1.0.2.h"
#include "Core/LedMacro/LedMacro_1.0.0.h"

// Button Input Driver and Macro:
#include "Core/InputMacro/InputMacro_1.0.1.h"
#include "Core/PinDriver/PinDriver_1.0.1.h"

// stepper configuration struct:
struct StepperConfigStruct
{
  uint8_t position_DPID;
  uint8_t command_DPID;
  PinPort EnPin;
  uint32_t stepperMaxStep;
  float stepperMaxSpeed;
  float stepperAccel;
  float StepperStopDeccel;
};

// generate prototypes for required functions:
unsigned char dp_process(unsigned char, const unsigned char[], unsigned short);
void dp_update_all(void);
void stepperCallBack(bool, bool);

/////////////////////////////////////
////////// Stepper Setup: //////////
/////////////////////////////////////
// Extract Stepper Setup:
StepperConfigStruct StepperConfigObj[] = StepperConfig;
const uint8_t stepperCount = (sizeof(StepperConfigObj) / sizeof(StepperConfigStruct)); // calculate number of steppers, this will be used through out the sketch:

// setup stepper queue:
StepperStruct _StepperStructObj[stepperCount]; // array of structs that 'StepperQueue' will use
StepperQueue StepperQueueObj(_StepperStructObj, stepperCount);

// setup 'StepperDriver':
StepperDriver stepper(stepperCallBack);
int8_t currentStepperProccessID = -1; // this will be used to identify which stepper is currently running. -1 = no stepper running

// setup pins for steppers:
PinPort stepPin(StepperStepPin);
PinPort dirPin(StepperDirPin);

// setup vars for steppers:
int32_t stepperCurrentPos[stepperCount];
int32_t stepperCurrentPos_dpid[stepperCount]; // keep track of all "sent" dpid values
uint8_t stepperCommand[stepperCount];
/*
  stepperCommand:
    0 = close
    1 = stop
    2 = open
*/

///////////////////////////////////
////////// Servo Setup: //////////
///////////////////////////////////
// setup Servo Pin:
PinPort servoPin(ServoPin);

// sequence build for the servo:
SequenceBuild servoBuild;

// setup Servo Vars:
uint8_t servoPinVal = 255; // idle high, trigger on LOW Pulse

/////////////////////////////////
////////// LED Setup: //////////
/////////////////////////////////
// setup sequence builder for LED:
SequenceBuild ledBuild;

// setup macro for led AND servo (total 2):
LedMacro _macro[2];
LedMacroManager macro(_macro, 2);

// PWM led value:
uint8_t ledVal = 0;

////////////////////////////////////
////////// Button Setup: //////////
////////////////////////////////////
// setup pinDriver for brownout handling and debouncing:
PinDriver pinInput(TUYA_WiFi_ModeSetPin);
InputMacro pinMacro(HIGH);

//////////////////////////////////
////////// TUYA Setup: //////////
//////////////////////////////////
// Tuya Module communication:
TuyaWifi tuya_module((HardwareSerial *)&Serial);

// setup DPID array to pass onto 'TuyaWifi':
unsigned char dpid_array[(stepperCount * 2) + 1][2]; // there are two DPID's per each stepper (position_DPID and command_DPID) + one DPID for the servo

// setup module reset pin:
PinPort moduleResetPin(TUYA_ModuleResetPin);

uint32_t update_dpid_timer = 0; // timer to handle periodic updating of DPID's

void setup()
{
  // initialize hardware Serial to 9600 for communication with TUYA Module:
  Serial.begin(9600);

  // Setup Timer Interrupt Interval for LED (100Hz):
  {
    cli(); // stop all interrupts

    // reset Timer 1 Registers:
    TCCR1A = 0;
    TCCR1B = 0;

    TCNT1 = 0; // initialize timer  counter value to 0

    // set compare match A register for 100 Hz increments:
    OCR1A = 19999; // = 16000000 / (8(pre-scaler) * 100) - 1 (must be <65536)

    TCCR1B |= (1 << WGM12); // turn on CTC(Clear-Timer(counter)-On-Compare) mode

    // Set CS12, CS11 and CS10 bits for 8 prescaler:
    TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);

    // enable timer compare interrupt on control mask register:
    TIMSK1 |= (1 << OCIE1A);
    sei(); // allow all interrupts
  }

  // setup pin module reset pin, stepper direction and step pin, led output pin, and servo pin:
  {
    // set module reset pin to Input so user can reset Module too without damaging Arduino:
    moduleResetPin.set(INPUT);
    moduleResetPin.write(LOW);

    // stepper pins setup:
    {
      stepPin.write(LOW);
      dirPin.write(LOW);

      stepPin.set(OUTPUT);
      dirPin.set(OUTPUT);
    }

    // LED pin setup:
    pinMode(StatusLedPin, OUTPUT);

    // Servo pin Setup:
    {
      servoPin.write(HIGH);
      servoPin.set(OUTPUT);
    }
  }

  // setup All stepper motors and corresponding values:
  for (uint8_t x = 0; x < stepperCount; x++)
  {
    // init position and command values:
    stepperCurrentPos[x] = 0;
    stepperCurrentPos_dpid[x] = 0;
    stepperCommand[x] = 1;
    /*
      stepperCommand:
        0 = close
        1 = stop
        2 = open
    */

    // configure stepper position DPID:
    dpid_array[(x * 2) + 0][0] = StepperConfigObj[x].position_DPID;
    dpid_array[(x * 2) + 0][1] = DP_TYPE_VALUE;
    // configure stepper command DPID:
    dpid_array[(x * 2) + 1][0] = StepperConfigObj[x].command_DPID;
    dpid_array[(x * 2) + 1][1] = DP_TYPE_ENUM;

    // setup stepper enable pins:
    StepperConfigObj[x].EnPin.write(HIGH); // stepper is enabled on logic LOW. Keep them disabled for now
    StepperConfigObj[x].EnPin.set(OUTPUT);
  }

  // setup servo DPID:
  {
    dpid_array[stepperCount * 2][0] = Servo_DPID;
    dpid_array[stepperCount * 2][1] = DP_TYPE_ENUM;
  }

  // set up TUYA Module:
  {
    unsigned char pid[] = {TUYA_PID};
    unsigned char mcu_ver[] = {TUYA_MCU_Version};

    // init TUYA PID and MCU Version:
    tuya_module.init(pid, mcu_ver);

    // Set all TUYA DPID's:
    tuya_module.set_dp_cmd_total(dpid_array, (stepperCount * 2) + 1);

    // set callbacks:
    tuya_module.dp_process_func_register(dp_process);       // function to proccess incomming commands
    tuya_module.dp_update_all_func_register(dp_update_all); // function to update all DPID states at once
  }

  // set initial LED sequence:
  ledBuild.setPrioritySequence(init_led, 0, true);
}

void loop()
{
  // run tuya uart service:
  tuya_module.uart_service();

  // run all handles:
  outputHandle();
  buttonHandle();
  moduleErrorHandle();
  stepperProccess();
  checkUpdate();

  // check update all timer:
  if (DPID_UpdateAllInterval && millis() - update_dpid_timer >= DPID_UpdateAllInterval)
    dp_update_all();
}

/*
  outputHandle():
    handles LED sequences and servo sequences
*/
inline void outputHandle()
{
  // determine led sequence:
  if (tuya_module.mcu_get_wifi_work_state() == AP_STATE)
    ledBuild.setSequence(ap_mode_led, 0, true);
  else if (tuya_module.mcu_get_wifi_work_state() == SMART_CONFIG_STATE)
    ledBuild.setSequence(smart_mode_led, 0, true);
  else if (tuya_module.mcu_get_wifi_work_state() == MODULE_UART_ERROR || tuya_module.mcu_get_wifi_work_state() == WIFI_STATE_UNKNOWN)
    ledBuild.setSequence(error_led, 0, true);
  else
    ledBuild.setSequence(idle_led, 0, true);
}

/*
  buttonHandle():
    handle WiFi Mode button, sets TUYA Module to Pairing mode
*/
inline void buttonHandle()
{
  /*
    WiFi_Config:
      0 = don't set WiFi
      1 = Smart_Mode
      2 = AP_Mode
  */
  static uint8_t WiFi_Config = 0;
  static uint32_t WiFi_setTimer = 0;
  const uint16_t WiFi_setInterval = 20000;

  // check for pin state change:
  if (pinMacro(pinInput))
  {
    if (pinMacro) // button released
    {
      WiFi_Config = 0; // stop setting up WiFi
    }
    else // button pressed
    {
      WiFi_setTimer = (millis() - WiFi_setInterval); // reset wifi send timer so it sends immediately next timer
    }
  }

  // check if button has been pushed down for more than 1000ms
  if (!pinMacro && !pinMacro.triggered() && pinMacro.interval() > 1000)
  {
    pinMacro.trigger();

    // check if the module is already in Smart Config State:
    if (tuya_module.mcu_get_wifi_work_state() == SMART_CONFIG_STATE)
      WiFi_Config = 2; // set to AP Mode
    else
      WiFi_Config = 1; // set to smart config
  }

  // set up wifi according to 'WiFi_Config'
  if (WiFi_Config)
  {
    // check if WiFi already done with setup:
    if (WiFi_Config == 1 && tuya_module.mcu_get_wifi_work_state() == SMART_CONFIG_STATE)
      WiFi_Config = 0; // no need to set up WiFi pair anymore
    else if (WiFi_Config == 2 && tuya_module.mcu_get_wifi_work_state() == AP_STATE)
      WiFi_Config = 0; // no need to set up WiFi pair anymore,
    else
    {
      if (millis() - WiFi_setTimer >= WiFi_setInterval)
      {
        WiFi_setTimer = millis(); // reset timer

        if (WiFi_Config == 1) // set to smart config mode
          tuya_module.mcu_set_wifi_mode(SMART_CONFIG);
        else if (WiFi_Config == 2) // set to smart config mode
          tuya_module.mcu_set_wifi_mode(AP_CONFIG);
      }
    }
  }
}

/*
  moduleErrorHandle():
    handle resetting module when error occurs
*/
inline void moduleErrorHandle()
{
  // keep track of the state for the reset pin. false/LOW = reset, true/HIGH = idle
  static bool resetPinState = true;
  const uint16_t moduleResetInterval = 10000;

  if (tuya_module.mcu_get_wifi_work_state() == MODULE_UART_ERROR)
  {
    static uint32_t moduleResetTimer = 0;
    if (resetPinState)
    {
      if (millis() - moduleResetTimer >= moduleResetInterval)
      {
        moduleResetTimer = millis(); // reset timer

        // set pin to LOW and OUTPUT:
        moduleResetPin.write(LOW);
        moduleResetPin.set(OUTPUT);

        // set pin state:
        resetPinState = false;
      }
    }
    else
    {
      if (millis() - moduleResetTimer >= 300)
      {
        moduleResetTimer = millis(); // reset timer

        // set pin to LOW and INPUT:
        moduleResetPin.set(INPUT);
        moduleResetPin.write(LOW);

        // set pin state:
        resetPinState = true;
      }
    }
  }
  else
  {
    // no module error, make sure pin is high:
    if (!resetPinState)
    {
      // set reset pin to input:
      moduleResetPin.set(INPUT);
      moduleResetPin.write(LOW);

      resetPinState = true; // reset pin state
    }
  }
}

/*
  stepperProccess():
    handle stepper moivment and stepper queue
*/
inline void stepperProccess()
{
  // only setup steppers if none are currently running:
  if (stepper.ready())
  {
    static bool resetSteppers = false; // used to prevent extra processing when no steppers are running. false = reset required, true = steppers are reset

    // check if a stepper is waiting in queue:
    if (StepperQueueObj.available())
    {
      resetSteppers = false; // set to "reset required"

      // setup return values from queue:
      uint8_t id;
      int32_t newPos;

      if (StepperQueueObj.read(id, newPos)) // only proceed on a successful read
      {
        // set proccess id so other parts of the sketch know which stepper is currently running:
        currentStepperProccessID = id;

        // reset all stepper enable pins:
        for (uint8_t x = 0; x < stepperCount; x++)
          StepperConfigObj[x].EnPin.write(HIGH); // stepper is enabled on logic LOW

        // set only the specified stepper enable pin:
        StepperConfigObj[id].EnPin.write(LOW); // enable the stepper

        // set current pos value reference so the stepper lib can modify the stepper position value:
        stepper.setStepper(stepperCurrentPos[id]);

        // setup the stepper profile for this specific stepper:
        stepper.setAccel(StepperConfigObj[id].stepperAccel);
        stepper.setMaxSpeed(StepperConfigObj[id].stepperMaxSpeed);

        // move the stepper to the desired position:
        stepper.moveTo(newPos);
      }
    }
    else if (!resetSteppers) // if no steppers are running, reset them. 'resetSteppers' is used so they dont have to be reset on each loop
    {
      currentStepperProccessID = -1; // reset current stepper ID

      // reset all stepper enable pins:
      for (uint8_t x = 0; x < stepperCount; x++)
        StepperConfigObj[x].EnPin.write(HIGH); // stepper is enabled on logic LOW

      resetSteppers = true; // set reset flag as "complete"
    }
  }
}

/*
  dp_proccess():
    all DPID commands pass through this function
*/
unsigned char dp_process(unsigned char dpid, const unsigned char value[], unsigned short length)
{
  // cycle through each stepper and find if the DPID's match:
  for (uint8_t x = 0; x < stepperCount; x++)
  {
    if (dpid == StepperConfigObj[x].position_DPID) // check position DPID
    {
      // record new, mapped, position:
      int32_t newPos = map(tuya_module.mcu_get_dp_download_data(dpid, value, length), 0, 100, 0, StepperConfigObj[x].stepperMaxStep);
      // update current position:
      stepperCurrentPos_dpid[x] = stepperCurrentPos[x];
      tuya_module.mcu_dp_update(dpid, map(stepperCurrentPos_dpid[x], 0, StepperConfigObj[x].stepperMaxStep, 0, 100), 1);
      update_dpid_timer = millis();

      // check if this stepper is currently moving:
      if (currentStepperProccessID == x)
      {
        // set profile just in case we are currently stopping:
        stepper.setAccel(StepperConfigObj[x].stepperAccel);
        stepper.moveTo(newPos); // move to new position
      }
      else
      {
        // add new target position to the stepper queue:
        StepperQueueObj.write(x, newPos);
      }
    }
    else if (dpid == StepperConfigObj[x].command_DPID) // check command DPID
    {
      // retrieve new stepper command:
      stepperCommand[x] = tuya_module.mcu_get_dp_download_data(dpid, value, length);
      // update new command:
      tuya_module.mcu_dp_update(dpid, stepperCommand[x], 1);

      // identify the given command:
      /*
        stepperCommand:
          0 = close
          1 = stop
          2 = open
      */
      switch (stepperCommand[x])
      {
      case 2: // open Stepper

        // check if this stepper is currently moving:
        if (currentStepperProccessID == x)
        {
          // set profile just in case we are currently stopping:
          stepper.setAccel(StepperConfigObj[x].stepperAccel);
          stepper.moveTo(StepperConfigObj[x].stepperMaxStep); // move to max/open position
        }
        else
        {
          // add new target position to the stepper queue:
          StepperQueueObj.write(x, StepperConfigObj[x].stepperMaxStep);
        }
        break;
      case 0: // close stepper

        // check if this stepper is currently moving:
        if (currentStepperProccessID == x)
        {
          // set profile just in case we are currently stopping:
          stepper.setAccel(StepperConfigObj[x].stepperAccel);
          stepper.moveTo(0); // move to 0/closed position
        }
        else
        {
          // add new target position to the stepper queue:
          StepperQueueObj.write(x, 0);
        }
        break;
      default: // stop stepper

        // check if this stepper is currently moving:
        if (currentStepperProccessID == x)
        {
          // set accel profile to the stop profile:
          stepper.setAccel(StepperConfigObj[x].StepperStopDeccel);
          stepper.stop(); // stop the stepper
        }
        else
        {
          // set target as current position:
          StepperQueueObj.write(x, stepperCurrentPos[x]);
        }
        break;
      }
    }
  }

  // check servo DPID:
  if (dpid == Servo_DPID)
  {
    // run servo sequence:
    servoBuild.setSequence(run_servo, 0, true);

    // update value:
    tuya_module.mcu_dp_update(dpid, 0, 1);
  }

  // return:
  return TY_SUCCESS;
}

/*
  checkUpdate():
    check all "report-able" DPID's and update if necessary. Resets Update timer interval if any update are run
*/
void checkUpdate()
{
  // only update dpid's if setup and if enough time passed:
  if (DPID_UpdateInterval && millis() - update_dpid_timer >= DPID_UpdateInterval)
  {
    // go through each stepper and update if necessary:
    for (uint8_t x = 0; x < stepperCount; x++)
    {
      if (stepperCurrentPos[x] != stepperCurrentPos_dpid[x])
      {
        // update stepper position dpid:
        stepperCurrentPos_dpid[x] = stepperCurrentPos[x];
        tuya_module.mcu_dp_update(StepperConfigObj[x].position_DPID, map(stepperCurrentPos_dpid[x], 0, StepperConfigObj[x].stepperMaxStep, 0, 100), 1);

        // reset dpid update interval timer only if a dpid has been updated:
        update_dpid_timer = millis();
      }
    }
  }
}

/*
  dp_update_all():
    updates all DPID values. Used by "Tuya_WiFi_MCU_SDK"
    also resets update timer
*/
void dp_update_all()
{
  // cycle through each stepper and update their positions and commands:
  for (uint8_t x = 0; x < stepperCount; x++)
  {
    // update position:
    stepperCurrentPos_dpid[x] = stepperCurrentPos[x];
    tuya_module.mcu_dp_update(StepperConfigObj[x].position_DPID, map(stepperCurrentPos_dpid[x], 0, StepperConfigObj[x].stepperMaxStep, 0, 100), 1);
    // update command:
    tuya_module.mcu_dp_update(StepperConfigObj[x].command_DPID, stepperCommand[x], 1);
  }

  // update servo DPID:
  tuya_module.mcu_dp_update(Servo_DPID, 0, 1);

  // reset dpid update interval timer:
  update_dpid_timer = millis();
}

/*
  stepperCallBack():
    callback function for moving stepper
    this is handled by the Timer 2 interrupt managed by 'StepperDriver'
*/
void stepperCallBack(bool par, bool dir)
{
  // write direction pin first before step pin, just in case direction changed:
  dirPin.write(dir);
  stepPin.write(par);
}

/*
  ISR(TIMER1_COMPA_vect):
    Timer 1 Interrupt Vector for LED and Servo Output
    Runs 100 timers-per-second

    Writes to LED and Servo Outputs also
*/
ISR(TIMER1_COMPA_vect)
{
  // run all necessary handles first:
  ledBuild.run();
  servoBuild.run();
  macro.run();

  // write to output pins:
  analogWrite(StatusLedPin, ledVal);
  servoPin.write((servoPinVal > 0 ? HIGH : LOW));
}

/////////////////////////////////////////////
////////// LED and Servo Sequences //////////
/////////////////////////////////////////////

// smart mode led sequence, fast pulse:
SB_FUNCT(smart_mode_led, macro.ready(ledVal))
SB_STEP(macro.quadEase(ledVal, 0, 10);)
SB_STEP(macro.quadEase(ledVal, 255, 10);)
SB_STEP(macro.delay(ledVal, 100);)
SB_STEP(macro.quadEase(ledVal, 0, 10);)
SB_STEP(macro.delay(ledVal, 100);)
SB_STEP(ledBuild.loop(1);)
SB_END

// ap mode led sequence, slow pulse:
SB_FUNCT(ap_mode_led, macro.ready(ledVal))
SB_STEP(macro.quadEase(ledVal, 0, 10);)
SB_STEP(macro.quadEase(ledVal, 255, 10);)
SB_STEP(macro.delay(ledVal, 1400);)
SB_STEP(macro.quadEase(ledVal, 0, 10);)
SB_STEP(macro.delay(ledVal, 1400);)
SB_STEP(ledBuild.loop(1);)
SB_END

// idle led, steady with very soft glow:
SB_FUNCT(idle_led, macro.ready(ledVal))
SB_STEP(macro.quadEase(ledVal, 255, 180);)
SB_STEP(macro.delay(ledVal, 400);)
SB_STEP(macro.quadEase(ledVal, 100, 180);)
SB_STEP(macro.delay(ledVal, 400);)
SB_STEP(ledBuild.loop(0);)
SB_END

// error led, rapid flashing:
SB_FUNCT(error_led, macro.ready(ledVal))
SB_STEP(macro.set(ledVal, 255, 50);)
SB_STEP(macro.set(ledVal, 0, 50);)
SB_STEP(macro.set(ledVal, 255, 50);)
SB_STEP(macro.set(ledVal, 0, 50);)
SB_STEP(macro.set(ledVal, 255, 50);)
SB_STEP(macro.set(ledVal, 0, 1000);)
SB_STEP(ledBuild.loop(0);)
SB_END

// init led, slow fade in and small delay to allow full initialization of Arduino:
SB_FUNCT(init_led, macro.ready(ledVal))
SB_STEP(macro.quadEase(ledVal, 255, 120);)
SB_STEP(macro.delay(ledVal, 1000);)
SB_END

// run servo, sends a small pulse and sets sequence to idle servo
SB_FUNCT(run_servo, macro.ready(servoPinVal))
SB_STEP(macro.set(servoPinVal, 0, 100);)
SB_STEP(macro.set(servoPinVal, 255, 100);)
SB_STEP(servoBuild.setSequence(idle_servo, 0, true);)
SB_END

// idle servo, mostly a place holder sequence. sets servo val to 255(idle)
SB_FUNCT(idle_servo, true)
SB_STEP(servoPinVal = 255;)
SB_END