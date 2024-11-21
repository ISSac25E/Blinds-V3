StepperDriver is meant to handle smooth acceleration and deceleration of a Stepper motor

It uses timer interrupt from the "TimerInterval" lib to handle stepper motor seamlessly in the background.

Version 1.0.0:
  - layout potential structure for motor driver
  Version abandoned, not enough research colleted prior to beginning library

Version 2.x.x:
  - rebuild and modify "accelStepper" library in the simplest way possible

  Version 2.1.x & 2.1.0
    - Minimal Modification to "accelStepper"

    Version 2.1.1
      - runs stepper completely off of an interrupt timer using the "TimerInterval" lib

    Version 2.1.2
      - step polarity functionality added
  
  Version 2.2.x
    - Attempt at multi-profile acceleration using "accelStepper" core
      This could be used to have different acceleration profiles for stopping and changing directions quickly and effectively
      results successful but deemed excessive and unnecessary for current project