#include <Arduino.h>
#include "Thermocouple/Thermocouple.h"
#include "AwfulPID.h"

#define PIN_IN_THERMOCOUPLE A0
#define PIN_OUT_HEATER 0

Thermocouple tc;
//Debounce period;

// Configure PID for an output in seconds correlating to the duty cycle of the heater.
// 100 second duty cycle period, duty cycle of 0-100%
// 10 second PID period
AwfulPID pid;
const PIDConfiguration pidCfg {10, 0, 100, false, 3, 50};
const PIDParameters pidParam {9.0, 0.0, 0.0};
int temperatureSetpoint;

void setup() {
    tc.setup(0,1);
    //period.setup(0);
    pid.setConfig(pidCfg);
    pid.setParam(pidParam);
}

void loop() {
    int uVolt;
    uVolt = map(analogRead(PIN_IN_THERMOCOUPLE), 0, 1023, 0, 5000000);

    int temp;    
    temp = tc.update(uVolt);

    int dutyCycle = pid.update(PID_ENABLE, temp, temperatureSetpoint);

    //period.update(!period.getState, )
    // Need to create a new function in timer just for duty cycle
    
}

