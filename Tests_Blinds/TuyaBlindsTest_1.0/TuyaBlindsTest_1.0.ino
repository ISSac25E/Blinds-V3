#include <AccelStepper.h>
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\Serge\Blinds V3\Core\Tuya WiFi MCU Serial SDK\Tuya_WiFi_MCU_SDK_2.0.0\Tuya_WiFi_MCU_SDK\src\TuyaWifi.h"

// #include <SoftwareSerial.h>
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\WordOfLife\core\LedMacros\SequenceBuild\SequenceBuild_1.0.2.h"
#include "C:\Users\AVG\Documents\Electrical Main (Amir)\Arduino\Projects\WordOfLife\core\LedMacros\LedMacro\LedMacro_1.0.0.h"

SequenceBuild build;

void forwardstep(void);
void backwardstep(void);

AccelStepper stepper(forwardstep, backwardstep);

#define BlindsMaxStep 5000
#define BlindsPin 8
#define DirPin 9

#define BlindsAccell 500
#define BlindsAccellStop 1500

LedMacro _macros[3];
LedMacroManager macro(_macros, 3);

uint8_t builtin_led_val = 0;

// SoftwareSerial softSerial(2, 3);
// TuyaWifi my_device(&softSerial);
TuyaWifi my_device(&Serial);

long percentControl = 0;

/* Current LED status */
unsigned char led_state = 0;
/* Connect network button pin */
#define WIFI_RESET_PIN 7

#define MODULE_RESET_PIN 11

/* Data point define */
#define DPID_BlindPosition 101
#define DPID_BlindControl 105

/* Stores all DPs and their types. PS: array[][0]:dpid, array[][1]:dp type.
 *                                     dp type(TuyaDefs.h) : DP_TYPE_RAW, DP_TYPE_BOOL, DP_TYPE_VALUE, DP_TYPE_STRING, DP_TYPE_ENUM, DP_TYPE_BITMAP
 */
unsigned char dp_array[][2] =
    {{DPID_BlindPosition, DP_TYPE_VALUE},
     {DPID_BlindControl, DP_TYPE_ENUM}};

uint16_t BlindPos = 0;
uint8_t BlindCont = 1;

unsigned char pid[] = {"ui1sml81jtapfrm5"};
unsigned char mcu_ver[] = {"1.0.0"};

/* last time */
unsigned long last_time = 0;

void setup()
{
  // Serial.begin(9600);
  Serial.begin(9600);

#ifdef DEBUG
  Serial.println("init");
#endif

  // softSerial.begin(9600);

  // Initialize led port, turn off led.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(MODULE_RESET_PIN, INPUT);

  pinMode(BlindsPin, OUTPUT);
  pinMode(DirPin, OUTPUT);

  digitalWrite(LED_BUILTIN, LOW);

  // Initialize networking keys.
  pinMode(WIFI_RESET_PIN, INPUT_PULLUP);

  // Enter the PID and MCU software version
  my_device.init(pid, mcu_ver);
  // incoming all DPs and their types array, DP numbers
  my_device.set_dp_cmd_total(dp_array, 2);
#ifdef DEBUG
  Serial.println((sizeof(dp_array) / sizeof(dp_array[0][0])) / 2);
#endif
  // register DP download processing callback function
  my_device.dp_process_func_register(dp_process);
  // register upload all DP callback function
  my_device.dp_update_all_func_register(dp_update_all);

  // my_device.mcu_reset_wifi();

  last_time = millis();

  // TIMER 2 for interrupt frequency 50000 Hz:
  cli();      // stop interrupts
  TCCR2A = 0; // set entire TCCR2A register to 0
  TCCR2B = 0; // same for TCCR2B
  TCNT2 = 0;  // initialize counter value to 0
  // set compare match register for 50000 Hz increments
  OCR2A = 39; // = 16000000 / (8 * 50000) - 1 (must be <256)
  // turn on CTC mode
  TCCR2B |= (1 << WGM21);
  // Set CS22, CS21 and CS20 bits for 8 prescaler
  TCCR2B |= (0 << CS22) | (1 << CS21) | (0 << CS20);
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
  sei(); // allow interrupts

  stepper.setMaxSpeed(500);
#ifdef DEBUG
  Serial.println("setup complete");
#endif
}

enum wifiMode
{
  wifi,
  wifi_reset
};

void loop()
{
  build.run();
  macro.run();
  digitalWrite(LED_BUILTIN, builtin_led_val);

  my_device.uart_service();

  static wifiMode _wifiMode = wifi;
  // Enter the connection network mode when Pin7 is pressed.
  if (!digitalRead(WIFI_RESET_PIN) && _wifiMode != wifi_reset)
  {
    _wifiMode = wifi_reset;
    my_device.mcu_reset_wifi();
#ifdef DEBUG
    Serial.println("WiFi mode: reset");
#endif
  }
  else
  {
    if (digitalRead(WIFI_RESET_PIN))
      _wifiMode = wifi;
  }

  static uint8_t modualeWorkState = !my_device.mcu_get_wifi_work_state();

#ifdef DEBUG
  if (modualeWorkState != my_device.mcu_get_wifi_work_state())
  {
    modualeWorkState = my_device.mcu_get_wifi_work_state();
    switch (modualeWorkState)
    {
    case SMART_CONFIG_STATE:
      Serial.println("Work State: SMART_CONFIG_STATE");
      break;
    case AP_STATE:
      Serial.println("Work State: AP_STATE");
      break;
    case WIFI_NOT_CONNECTED:
      Serial.println("Work State: WIFI_NOT_CONNECTED");
      break;
    case WIFI_CONNECTED:
      Serial.println("Work State: WIFI_CONNECTED");
      break;
    case WIFI_CONN_CLOUD:
      Serial.println("Work State: WIFI_CONN_CLOUD");
      break;
    case WIFI_LOW_POWER:
      Serial.println("Work State: WIFI_LOW_POWER");
      break;
    case SMART_AND_AP_STATE:
      Serial.println("Work State: SMART_AND_AP_STATE");
      break;
    case MODULE_UART_ERROR:
      Serial.println("Work State: MODULE_UART_ERROR");
      break;
    case WIFI_STATE_UNKNOWN:
      Serial.println("Work State: WIFI_STATE_UNKNOWN");
      break;
    default:
      Serial.println("Work State: STATE_ERROR");
      break;
    }
  }
#endif

  {
    static bool pinState = false;

    if (my_device.mcu_get_wifi_work_state() == MODULE_UART_ERROR)
    {
      static uint32_t moduleRstTimer = 0;
      if (pinState)
      {
        if (millis() - moduleRstTimer >= 200)
        {
          moduleRstTimer = millis();
          pinState = false;
          pinMode(MODULE_RESET_PIN, INPUT);
        }
      }
      else
      {
        if (millis() - moduleRstTimer >= 9800)
        {
          moduleRstTimer = millis();
          pinState = true;
          digitalWrite(MODULE_RESET_PIN, LOW);
          pinMode(MODULE_RESET_PIN, OUTPUT);
        }
      }
    }
    else
    {
      pinState = false;
    }
  }

  /* LED blinks when network is being connected */
  if ((my_device.mcu_get_wifi_work_state() != WIFI_LOW_POWER) && (my_device.mcu_get_wifi_work_state() != WIFI_CONN_CLOUD) && (my_device.mcu_get_wifi_work_state() != WIFI_STATE_UNKNOWN))
  {
    if (my_device.mcu_get_wifi_work_state() == AP_STATE)
    {
      build.setSequence(ap_mode_led, 0, true);
    }
    else if (my_device.mcu_get_wifi_work_state() == SMART_CONFIG_STATE)
    {
      build.setSequence(smart_mode_led, 0, true);
    }
    else
    {
      build.setSequence(stop_blink, 0, true);
    }
  }
  else
  {
    build.setSequence(stop_blink, 0, true);
  }

  static uint32_t updateAllTimer = millis();
  if (millis() - updateAllTimer >= 1000)
  {
    updateAllTimer = millis();
    dp_update_all();
  }
}

/**
 * @description: DP download callback function.
 * @param {unsigned char} dpid
 * @param {const unsigned char} value
 * @param {unsigned short} length
 * @return {unsigned char}
 */
unsigned char dp_process(unsigned char dpid, const unsigned char value[], unsigned short length)
{
#ifdef DEBUG

  Serial.print("dp process: ");
  Serial.print(dpid);
  Serial.print(", [");
  for (uint8_t x = 0; x < length; x++)
  {
    Serial.print((uint8_t)value[x]);
    if (x < length - 1)
      Serial.print(", ");
  }
  Serial.println("]");
#endif
  if (dpid == DPID_BlindPosition)
  {
    BlindPos = my_device.mcu_get_dp_download_data(dpid, value, length);
    my_device.mcu_dp_update(DPID_BlindPosition, BlindPos, 1);
    stepper.setAcceleration(BlindsAccell);
    stepper.moveTo(map(BlindPos, 0, 100, 0, BlindsMaxStep));
  }
  else if (dpid == DPID_BlindControl)
  {
    BlindCont = my_device.mcu_get_dp_download_data(dpid, value, length);
    my_device.mcu_dp_update(DPID_BlindControl, BlindCont, 1);
    if (BlindCont == 0)
    {
      stepper.setAcceleration(BlindsAccell);
      stepper.moveTo(BlindsMaxStep);
    }
    else if (BlindCont == 1)
    {
      stepper.setAcceleration(BlindsAccellStop);
      stepper.stop();
    }
    else if (BlindCont == 2)
    {
      stepper.setAcceleration(BlindsAccell);
      stepper.moveTo(0);
    }
  }
  return TY_SUCCESS;
}

/**
 * @description: Upload all DP status of the current device.
 * @param {*}
 * @return {*}
 */
void dp_update_all(void)
{
#ifdef DEBUG
  Serial.println("dp update all");
#endif
  BlindPos = map(stepper.currentPosition(), 0, BlindsMaxStep, 0, 100);
  my_device.mcu_dp_update(DPID_BlindPosition, BlindPos, 1);
  my_device.mcu_dp_update(DPID_BlindControl, BlindCont, 1);
}

void forwardstep()
{
  digitalWrite(DirPin, HIGH);
  digitalWrite(BlindsPin, !digitalRead(BlindsPin));
}

void backwardstep()
{
  digitalWrite(DirPin, LOW);
  digitalWrite(BlindsPin, !digitalRead(BlindsPin));
}

ISR(TIMER2_COMPA_vect)
{
  stepper.run();
}

SB_FUNCT(ap_mode_led, macro.ready(builtin_led_val))
SB_STEP(macro.set(builtin_led_val, 255, 1200);)
SB_STEP(macro.set(builtin_led_val, 0, 1200);)
SB_STEP(build.loop(0);)
SB_END

SB_FUNCT(smart_mode_led, macro.ready(builtin_led_val))
SB_STEP(macro.set(builtin_led_val, 255, 500);)
SB_STEP(macro.set(builtin_led_val, 0, 500);)
SB_STEP(build.loop(0);)
SB_END

SB_FUNCT(stop_blink, macro.ready(builtin_led_val))
SB_STEP(macro.set(builtin_led_val, 0, 0);)
SB_END