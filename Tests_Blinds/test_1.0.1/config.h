#define StepperSpeed 1000

#define StepperAccel 1500
#define StepperAccelStop 4000

#define TUYA_PID "*****"
#define TUYA_MCU_Version "1.0.0"

#define WiFi_ModeSetPin 3

#define ModuleResetPin 7

#define StepperStepPin 8
#define StepperDirPin 9

#define Servo_DPID 115
#define ServoPin 4

#define LED_Pin 6

// {DPID_Position, DPID_Commmand, EnPin, MaxStep, StepperSpeed, StepperAccel, StepperAccelStop}
#define SteppersConfig {{101, 102, A0, 3000, StepperSpeed, StepperAccel, StepperAccelStop},\
                        {103, 104, A1, 3000, StepperSpeed, StepperAccel, StepperAccelStop},\
                        {105, 106, A2, 3000, StepperSpeed, StepperAccel, StepperAccelStop}, \
                        {107, 108, A4, 3000, StepperSpeed, StepperAccel, StepperAccelStop},\
                        {109, 110, A5, 3000, StepperSpeed, StepperAccel, StepperAccelStop}}