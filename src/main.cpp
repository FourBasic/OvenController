#include <Arduino.h>
#include "Thermocouple/Thermocouple.h"
#include "AwfulPID.h"
#include "DutyCycle.h"

#define PIN_IN_THERMOCOUPLE A0
#define PIN_OUT_HEATER 0

Thermocouple tc;
DutyCycle dc;

// Configure PID for an output in seconds correlating to the duty cycle of the heater.
// duty cycle of 0-100% * 10
// 10 second PID period
AwfulPID pid;
const PIDConfiguration pidCfg {10000, 0, 1000, false, 30, 5};
const PIDParameters pidParam {9.0, 0.0, 0.0};
int temperatureSetpoint;

void setup() {
    tc.setup(TC_TYPE_K, 1, 10, 200);
    pid.setConfig(pidCfg);
    pid.setParam(pidParam);
}

void loop() {
    int uVolt;
    uVolt = map(analogRead(PIN_IN_THERMOCOUPLE), 0, 1023, 0, 5000000);

    int temp;    
    temp = tc.update(uVolt);

    float dutyCycle;
    dutyCycle = float(pid.update(PID_ENABLE, temp, temperatureSetpoint) / 10);
    bool heatCycle;
    heatCycle = dc.update(true, 60000, dutyCycle);

    digitalWrite(PIN_OUT_HEATER, heatCycle);
}

