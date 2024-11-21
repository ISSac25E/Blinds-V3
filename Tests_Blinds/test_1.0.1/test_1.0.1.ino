#include "Arduino.h"
#include "config.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Blinds V3\Core\Tuya WiFi MCU Serial SDK\Tuya_WiFi_MCU_SDK_2.0.0\Tuya_WiFi_MCU_SDK\src\TuyaWifi.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Blinds V3\Core\StepperDriver\StepperDriver_2.1.3.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Blinds V3\Core\StepperQueue\StepperQueue_1.0.0.h"

#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Blinds V3\Core\PinPort\PinPort_1.0.0.h"

#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\WordOfLife\core\LedMacros\SequenceBuild\SequenceBuild_1.0.2.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\WordOfLife\core\LedMacros\LedMacro\LedMacro_1.0.0.h"

#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\WordOfLife\core\InputMacro\InputMacro_1.0.1.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\WordOfLife\core\PinDriver\PinDriver_1.0.1.h"

struct StepperConfigStruct
{
  uint8_t position_DPID;
  uint8_t command_DPID;
  uint8_t EnPin;
  uint32_t stepperMaxStep;
  float stepperMaxSpeed;
  float stepperAccel;
  float stepperAccelStop;
};

unsigned char dp_process(unsigned char, const unsigned char[], unsigned short);
void dp_update_all(void);
void stepperCallBack(bool, bool);

// Stepper Config:
StepperConfigStruct StepperConfigStructObj[] = SteppersConfig;
const uint8_t stepperCount = (sizeof(StepperConfigStructObj) / sizeof(StepperConfigStruct));

StepperStruct StepperStructObj[stepperCount];
StepperQueue StepperQueueObj(StepperStructObj, stepperCount);

StepperDriver stepper(stepperCallBack);
int32_t stepperCurrentPos[stepperCount];
uint8_t stepperCommand[stepperCount];

uint8_t servoCommand = 0;
uint8_t servoPinVal = 255;

SequenceBuild servoBuild;

int8_t currentIdStepperProccess = -1;

unsigned char dpid_array[(stepperCount * 2) + 1][2];

PinPort stepPin(StepperStepPin);
PinPort dirPin(StepperDirPin);

// LED Config:
SequenceBuild build;

LedMacro _macro[4];
LedMacroManager macro(_macro, 4);

uint8_t ledVal = 0;

// Button Config:
PinDriver pinInput(WiFi_ModeSetPin);
InputMacro pinMacro(true);

// Tuya Serial Config:
TuyaWifi tuya_device(&Serial);
uint32_t updateAll_timer = 0;

void setup()
{
  Serial.begin(9600);

  pinMode(ModuleResetPin, INPUT);

  pinMode(LED_Pin, OUTPUT);

  digitalWrite(ServoPin, HIGH);
  pinMode(ServoPin, OUTPUT);

  {
    stepPin.write(LOW);
    dirPin.write(LOW);

    stepPin.set(HIGH);
    dirPin.set(HIGH);
  }

  for (uint8_t x = 0; x < stepperCount; x++)
  {
    stepperCurrentPos[x] = 0;
    stepperCommand[x] = 1;

    dpid_array[(x * 2) + 0][0] = StepperConfigStructObj[x].position_DPID;
    dpid_array[(x * 2) + 0][1] = DP_TYPE_VALUE;
    dpid_array[(x * 2) + 1][0] = StepperConfigStructObj[x].command_DPID;
    dpid_array[(x * 2) + 1][1] = DP_TYPE_ENUM;

    digitalWrite(StepperConfigStructObj[x].EnPin, HIGH); // stepper is enabled on logic LOW
    pinMode(StepperConfigStructObj[x].EnPin, OUTPUT);
  }

  {
    dpid_array[stepperCount * 2][0] = Servo_DPID;
    dpid_array[stepperCount * 2][1] = DP_TYPE_ENUM;
  }

  {
    unsigned char pid[] = {TUYA_PID};
    unsigned char mcu_ver[] = {TUYA_MCU_Version};
    tuya_device.init(pid, mcu_ver);
    tuya_device.set_dp_cmd_total(dpid_array, (stepperCount * 2) + 1);
    tuya_device.dp_process_func_register(dp_process);
    tuya_device.dp_update_all_func_register(dp_update_all);
  }

  build.setPrioritySequence(start_led, 0, true);
}

void loop()
{
  outputHandle();
  buttonHandle();
  stepperProccess();
  tuya_device.uart_service();

  {
    static bool pinState = false;

    if (tuya_device.mcu_get_wifi_work_state() == MODULE_UART_ERROR)
    {
      static uint32_t moduleRstTimer = 0;
      if (pinState)
      {
        if (millis() - moduleRstTimer >= 200)
        {
          moduleRstTimer = millis();
          pinState = false;
          pinMode(ModuleResetPin, INPUT);
        }
      }
      else
      {
        if (millis() - moduleRstTimer >= 9800)
        {
          moduleRstTimer = millis();
          pinState = true;
          digitalWrite(ModuleResetPin, LOW);
          pinMode(ModuleResetPin, OUTPUT);
        }
      }
    }
    else
    {
      pinState = false;
    }
  }

  if (millis() - updateAll_timer >= 3000)
    dp_update_all();
}

inline void buttonHandle()
{
  /*
    0 = don't set WiFi
    1 = AP_Mode
    2 = SmartMode
  */
  static uint8_t setWiFi = 0;
  if (pinMacro(pinInput))
  {
    if (pinMacro)
    {
      setWiFi = 0;
    }
  }

  if (!pinMacro && !pinMacro.triggered() && pinMacro.interval() > 1000)
  {
    if (tuya_device.mcu_get_wifi_work_state() == SMART_CONFIG_STATE)
    {
      setWiFi = 1;
    }
    else
    {
      setWiFi = 2;
    }
    pinMacro.trigger();
  }

  if (setWiFi)
  {
    bool WiFi_set = false;
    if (setWiFi == 1 && tuya_device.mcu_get_wifi_work_state() == AP_STATE)
      WiFi_set = true;
    else if (setWiFi != 1 && tuya_device.mcu_get_wifi_work_state() == SMART_CONFIG_STATE)
      WiFi_set = true;

    if (!WiFi_set)
    {
      static uint32_t setTimer = millis();
      if (millis() - setTimer >= 18000)
      {
        setTimer = millis();
        if (setWiFi == 1)
        {
          tuya_device.mcu_set_wifi_mode(AP_CONFIG);
        }
        else
        {
          tuya_device.mcu_set_wifi_mode(SMART_CONFIG);
        }
      }
    }
  }
}

inline void outputHandle()
{
  build.run();
  servoBuild.run();
  macro.run();
  analogWrite(LED_Pin, ledVal);
  digitalWrite(ServoPin, (servoPinVal == 0 ? LOW : HIGH));

  if (tuya_device.mcu_get_wifi_work_state() == AP_STATE)
  {
    build.setSequence(ap_mode_led, 0, true);
  }
  else if (tuya_device.mcu_get_wifi_work_state() == SMART_CONFIG_STATE)
  {
    build.setSequence(smart_mode_led, 0, true);
  }
  else if (tuya_device.mcu_get_wifi_work_state() == MODULE_UART_ERROR || tuya_device.mcu_get_wifi_work_state() == WIFI_STATE_UNKNOWN)
  {
    build.setSequence(error_led, 0, true);
  }
  else
  {
    build.setSequence(idle_led, 0, true);
  }
}

inline void stepperProccess()
{
  if (stepper.ready())
  {
    static bool reset = false;
    if (StepperQueueObj.available())
    {
      reset = false;
      uint8_t id;
      int32_t val;
      if (StepperQueueObj.read(id, val))
      {
        if (id < stepperCount)
        {
          currentIdStepperProccess = id;

          for (uint8_t x = 0; x < stepperCount; x++)
            digitalWrite(StepperConfigStructObj[x].EnPin, HIGH);
          digitalWrite(StepperConfigStructObj[id].EnPin, LOW);

          stepper.setStepper(stepperCurrentPos[id]);
          stepper.setAccel(StepperConfigStructObj[id].stepperAccel);
          stepper.setMaxSpeed(StepperConfigStructObj[id].stepperMaxSpeed);

          stepper.moveTo(val);
        }
      }
    }
    else
    {
      if (!reset)
      {
        currentIdStepperProccess = -1;
        for (uint8_t x = 0; x < stepperCount; x++)
          digitalWrite(StepperConfigStructObj[x].EnPin, HIGH);
        reset = true;
      }
    }
  }
}

unsigned char dp_process(unsigned char dpid, const unsigned char value[], unsigned short length)
{
  for (uint8_t x = 0; x < stepperCount; x++)
  {
    if (dpid == StepperConfigStructObj[x].position_DPID)
    {
      int32_t newPos = map(tuya_device.mcu_get_dp_download_data(dpid, value, length), 0, 100, 0, StepperConfigStructObj[x].stepperMaxStep);
      tuya_device.mcu_dp_update(dpid, map(stepperCurrentPos[x], 0, StepperConfigStructObj[x].stepperMaxStep, 0, 100), 1);

      if (currentIdStepperProccess == x)
      {
        stepper.setAccel(StepperConfigStructObj[x].stepperAccel);
        stepper.moveTo(newPos);
      }
      else
      {
        StepperQueueObj.write(x, newPos);
      }
    }
    else if (dpid == StepperConfigStructObj[x].command_DPID)
    {
      stepperCommand[x] = tuya_device.mcu_get_dp_download_data(dpid, value, length);
      tuya_device.mcu_dp_update(dpid, stepperCommand[x], 1);

      switch (stepperCommand[x])
      {
      case 0: // Open
        if (currentIdStepperProccess == x)
        {
          stepper.setAccel(StepperConfigStructObj[x].stepperAccel);
          stepper.moveTo(StepperConfigStructObj[x].stepperMaxStep);
        }
        else
        {
          StepperQueueObj.write(x, StepperConfigStructObj[x].stepperMaxStep);
        }
        break;
      case 2: // Close
        if (currentIdStepperProccess == x)
        {
          stepper.setAccel(StepperConfigStructObj[x].stepperAccel);
          stepper.moveTo(0);
        }
        else
        {
          StepperQueueObj.write(x, 0);
        }
        break;
      default: // Stop
        if (currentIdStepperProccess == x)
        {
          stepper.setAccel(StepperConfigStructObj[x].stepperAccelStop);
          stepper.stop();
        }
        else
        {
          StepperQueueObj.write(x, stepperCurrentPos[x]);
        }
        break;
      }
    }
  }
  if (dpid == Servo_DPID)
  {
    {
      // run servo
      servoBuild.setSequence(servoRun, 0, true);
    }
    tuya_device.mcu_dp_update(Servo_DPID, 0, 1);
  }
  return TY_SUCCESS;
}

void dp_update_all()
{
  updateAll_timer = millis();
  for (uint8_t x = 0; x < stepperCount; x++)
  {
    tuya_device.mcu_dp_update(StepperConfigStructObj[x].position_DPID, map(stepperCurrentPos[x], 0, StepperConfigStructObj[x].stepperMaxStep, 0, 100), 1);
    tuya_device.mcu_dp_update(StepperConfigStructObj[x].command_DPID, stepperCommand[x], 1);
  }
  tuya_device.mcu_dp_update(Servo_DPID, 0, 1);
}

void stepperCallBack(bool par, bool dir)
{
  dirPin.write(dir);
  stepPin.write(par);
}

SB_FUNCT(ap_mode_led, macro.ready(ledVal))
SB_STEP(macro.quadEase(ledVal, 0, 10);)
SB_STEP(macro.quadEase(ledVal, 255, 10);)
SB_STEP(macro.delay(ledVal, 1000);)
SB_STEP(macro.quadEase(ledVal, 0, 10);)
SB_STEP(macro.delay(ledVal, 1000);)
SB_STEP(build.loop(1);)
SB_END

SB_FUNCT(smart_mode_led, macro.ready(ledVal))
SB_STEP(macro.quadEase(ledVal, 0, 10);)
SB_STEP(macro.quadEase(ledVal, 255, 10);)
SB_STEP(macro.delay(ledVal, 100);)
SB_STEP(macro.quadEase(ledVal, 0, 10);)
SB_STEP(macro.delay(ledVal, 100);)
SB_STEP(build.loop(1);)
SB_END

SB_FUNCT(idle_led, macro.ready(ledVal))
SB_STEP(macro.quadEase(ledVal, 255, 120);)
SB_STEP(macro.delay(ledVal, 300);)
SB_STEP(macro.quadEase(ledVal, 100, 120);)
SB_STEP(macro.delay(ledVal, 300);)
SB_STEP(build.loop(0);)
SB_END

SB_FUNCT(error_led, macro.ready(ledVal))
SB_STEP(macro.set(ledVal, 255, 50);)
SB_STEP(macro.set(ledVal, 0, 50);)
SB_STEP(macro.set(ledVal, 255, 50);)
SB_STEP(macro.set(ledVal, 0, 50);)
SB_STEP(macro.set(ledVal, 255, 50);)
SB_STEP(macro.set(ledVal, 0, 1000);)
SB_STEP(build.loop(0);)
SB_END

SB_FUNCT(start_led, macro.ready(ledVal))
SB_STEP(macro.quadEase(ledVal, 255, 120);)
SB_STEP(macro.delay(ledVal, 1000);)
SB_END

SB_FUNCT(servoRun, macro.ready(servoPinVal))
SB_STEP(macro.set(servoPinVal, 0, 1000);)
SB_STEP(macro.set(servoPinVal, 255, 1000);)
SB_STEP(servoBuild.setSequence(servoIdle, 0, true);)
SB_END

SB_FUNCT(servoIdle, macro.ready(servoPinVal))
SB_STEP(servoPinVal = 155;)
SB_END