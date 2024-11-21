//ESP_BUTTON_INTERFACE REV 1.0.0
//.h
#ifndef ESP_BUTTON_INTERFACE_h
#define ESP_BUTTON_INTERFACE_h

#include "Arduino.h"

class ESP_PIN_DRIVER {
  public:

    ESP_PIN_DRIVER(uint8_t Pin);                //default PullUp, GPIO 16 PullDown
    ESP_PIN_DRIVER(uint8_t Pin, bool PullMode); //Mode = High, PullUp Enabled Exp. GPIO 16 PullDown

    //Returns Pin State and runs Button:
    bool Run();
    //Returns button State, does not run Button:
    bool ButtonState() {
      return _ButtonState;
    }
    void ButtonDebounce(uint16_t Debounce_us) {
      _ButtonDeBounceDelay = Debounce_us;
    }
    
  private:

    uint8_t _Pin;

    bool _ButtonState = false;
    bool _ButtonTest = false;
    uint32_t _ButtonTestTimer = 0;
    uint16_t _ButtonDeBounceDelay = 5000; //Default Value 5ms, 5000us
    //Function for Pin reading
    bool _PinRead() {
      return digitalRead(_Pin);
    }
};

class PIN_MACRO {
  public:
    //Returns true if StateChange
    bool Run(bool PinState);
    bool State() {
      return _MacroState;
    }
    uint32_t PrevInterval() {
      return _MacroPrevInterval;
    }
    uint32_t Interval() {
      return millis() - _MacroIntervalTimer;
    }
    void TimerReset() {
      _MacroIntervalTimer = millis();
    }
    void TimerSet(uint32_t TimerSet) {
      _MacroIntervalTimer = (millis() + TimerSet);
    }
  private:
    //Variables for Running Macro:
    bool _MacroState = false;
    uint32_t _MacroIntervalTimer = 0;
    uint32_t _MacroPrevInterval = 0;
};

//.cpp
//#include "ESP_BUTTON_INTERFACE.h"
//#include "Arduino.h"

ESP_PIN_DRIVER::ESP_PIN_DRIVER(uint8_t Pin) {
  //Defualt Do PullUp, if GPIO 16 do PullDown:
  _Pin = Pin;
  if (_Pin == 16) {
    pinMode(_Pin, INPUT_PULLDOWN_16);
  }
  else {
    pinMode(_Pin, INPUT_PULLUP);
  }
}

ESP_PIN_DRIVER::ESP_PIN_DRIVER(uint8_t Pin, bool PullMode) {
  //Mode High: PullUp, if GPIO 16 PullDown
  _Pin = Pin;
  if (PullMode) {
    if (_Pin == 16) {
      pinMode(_Pin, INPUT_PULLDOWN_16);
    }
    else {
      pinMode(_Pin, INPUT_PULLUP);
    }
  }
  else {
    pinMode(_Pin, INPUT);
  }
}

bool ESP_PIN_DRIVER::Run() {
  if (_ButtonTest) {
    if (micros() - _ButtonTestTimer >= _ButtonDeBounceDelay) {
      if (_PinRead() != _ButtonState) {
        _ButtonState = !_ButtonState;
        _ButtonTest = false;
      }
      else {
        _ButtonTest = false;
      }
    }
  }
  else {
    if (_PinRead() != _ButtonState) {
      _ButtonTest = true;
      _ButtonTestTimer = micros();
    }
  }
  return _ButtonState;
}

bool PIN_MACRO::Run(bool PinState) {
  if (PinState != _MacroState) {
    //State Change:
    _MacroState = PinState;
    _MacroPrevInterval = millis() - _MacroIntervalTimer;
    _MacroIntervalTimer = millis();
    return true;
  }
  else return false;
}

#endif
