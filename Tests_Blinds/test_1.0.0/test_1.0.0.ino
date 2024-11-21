#include "Arduino.h"
#include "config.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Blinds V3\Core\Tuya WiFi MCU Serial SDK\Tuya_WiFi_MCU_SDK_2.0.0\Tuya_WiFi_MCU_SDK\src\TuyaWifi.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Blinds V3\Core\StepperDriver\StepperDriver_2.1.3.h"

#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\WordOfLife\core\LedMacros\SequenceBuild\SequenceBuild_1.0.2.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\WordOfLife\core\LedMacros\LedMacro\LedMacro_1.0.0.h"

#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\WordOfLife\core\InputMacro\InputMacro_1.0.1.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\WordOfLife\core\PinDriver\PinDriver_1.0.1.h"

unsigned char dp_process(unsigned char, const unsigned char[], unsigned short);
void dp_update_all(void);
void stepperCallBack(bool, bool);

SequenceBuild build;

LedMacro _macro[3];
LedMacroManager macro(_macro, 3);

uint8_t ledVal = 0;

PinDriver pinInput(WiFi_ModeSetPin);
InputMacro pinMacro(true);

TuyaWifi tuya_device(&Serial);

StepperDriver stepper(stepperCallBack);

uint8_t stepper_control = 1;

uint32_t updateAll_timer = 0;

unsigned char dpid_array[][2] =
    {{TUYA_DPID_BlindPosition, DP_TYPE_VALUE},
     {TUYA_DPID_BlindControl, DP_TYPE_ENUM}};

void setup()
{
  Serial.begin(9600);

  pinMode(ModuleResetPin, INPUT);

  pinMode(LED_Pin, OUTPUT);

  pinMode(StepperStepPin, OUTPUT);
  pinMode(StepperDirPin, OUTPUT);

  unsigned char pid[] = {TUYA_PID};
  unsigned char mcu_ver[] = {TUYA_Version};
  tuya_device.init(pid, mcu_ver);
  tuya_device.set_dp_cmd_total(dpid_array, 2);
  tuya_device.dp_process_func_register(dp_process);
  tuya_device.dp_update_all_func_register(dp_update_all);

  build.setPrioritySequence(start_led, 0, true);
  stepper.setAccel(StepperAccel);
  stepper.setMaxSpeed(StepperSpeed);
}

void loop()
{
  ledHandle();
  buttonHandle();
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

  if (millis() - updateAll_timer >= 5000)
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
      if (millis() - setTimer >= 15000)
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

inline void ledHandle()
{
  build.run();
  macro.run();
  analogWrite(LED_Pin, ledVal);

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

unsigned char dp_process(unsigned char dpid, const unsigned char value[], unsigned short length)
{
  if (dpid == TUYA_DPID_BlindPosition)
  {
    int32_t newPos = tuya_device.mcu_get_dp_download_data(dpid, value, length);
    tuya_device.mcu_dp_update(TUYA_DPID_BlindPosition, map(stepper.step(), 0, BlindsMaxStep, 0, 100), 1);

    stepper.setAccel(StepperAccel);
    stepper.moveTo(map(newPos, 0, 100, 0, BlindsMaxStep));
  }
  else if (dpid == TUYA_DPID_BlindControl)
  {
    stepper_control = tuya_device.mcu_get_dp_download_data(dpid, value, length);
    tuya_device.mcu_dp_update(TUYA_DPID_BlindControl, stepper_control, 1);
    switch (stepper_control)
    {
    case 0:
      stepper.setAccel(StepperAccel);
      stepper.moveTo(BlindsMaxStep);
      break;
    case 2:
      stepper.setAccel(StepperAccel);
      stepper.moveTo(0);
      break;
    default:
      stepper.setAccel(StepperAccelStop);
      stepper.stop();
      break;
    }
  }
  return TY_SUCCESS;
}

void dp_update_all()
{
  updateAll_timer = millis();
  tuya_device.mcu_dp_update(TUYA_DPID_BlindPosition, map(stepper.step(), 0, BlindsMaxStep, 0, 100), 1);
  tuya_device.mcu_dp_update(TUYA_DPID_BlindControl, stepper_control, 1);
}

void stepperCallBack(bool par, bool dir)
{
  digitalWrite(StepperDirPin, dir);
  digitalWrite(StepperStepPin, par);
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
