#include <Arduino.h>
#include "Thermocouple/Thermocouple.h"
#include "AwfulPID.h"
#include "DutyCycle.h"
#include "Adafruit_GFX.h" 
#include "Adafruit_TFTLCD.h"
#include "MCUFRIEND_kbv.h"

#define PIN_IN_THERMOCOUPLE A0
#define PIN_OUT_HEATER 0

#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0
#define LCD_RESET A4

#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

struct FieldPosition {
  int x,y;
};

struct FieldProperties {
  FieldPosition pos;
  uint16_t color, bg, font;
  String label;
  uint16_t labelColor;
  uint16_t labelOffs;
  String unit;
  uint16_t padding;
};

struct TextField {    
  FieldProperties prop;
  String text;
};

struct IntegerField {    
  FieldProperties prop;
  int val;
};

Thermocouple tc;

TimerOnDelay screenRefresh;
MCUFRIEND_kbv tft;
IntegerField field_sp {.prop={.pos={.x=0, .y=0}, .color=CYAN, .bg=BLACK, .font=3, .label="SP:", .labelColor=WHITE, .labelOffs=0, .unit="C", .padding = 5}, .val=0};

// Configure PID for an output in seconds correlating to the duty cycle of the heater.
// duty cycle of 0-100% * 10
// 10 second PID period
AwfulPID pid;
const PIDConfiguration pidCfg {2000, 0, 1000, false, 30, 5};
const PIDParameters pidParam {1.0, 1.0, 0.0};
DutyCycle dc;
int temperatureSetpoint;

void initializeFieldProperties (struct FieldProperties* prop) {
  FieldPosition* pos = &prop->pos;
  tft.setCursor(pos->x, pos->y); 
  tft.setTextSize(prop->font); 
  tft.setTextColor(prop->labelColor);  
  tft.print(prop->label);
  prop->labelOffs = tft.getCursorX();
}

void updateIntegerField(struct IntegerField* field, int val) {
  FieldProperties* prop = &field->prop;
  FieldPosition* pos = &prop->pos;
  tft.setTextSize(prop->font);
  int pX_val = pos->x + prop->labelOffs;
  // Backgound previous value
  tft.setCursor(pX_val, pos->y);  
  tft.setTextColor(prop->bg);  
  tft.print(field->val);
  tft.setCursor(tft.getCursorX() + prop->padding, pos->y);
  tft.print(prop->unit);
  // Forground new value
  field->val = val;
  tft.setCursor(pX_val, pos->y);  
  tft.setTextColor(prop->color);
  tft.print(val);
  tft.setTextColor(prop->labelColor);
  tft.setCursor(tft.getCursorX() + prop->padding, pos->y);
  tft.print(prop->unit);
}

void setup() {
  //Serial.begin(9600);

  tc.setup(TC_TYPE_K, 1, 10, 200);
  
  pid.setConfig(pidCfg);
  pid.setParam(pidParam);

  tft.reset();
  tft.begin(tft.readID());
  tft.fillScreen(BLACK);
  initializeFieldProperties(&field_sp.prop);
  delay(3000);
}

void loop() {
  temperatureSetpoint++; 

  int uVolt;
  //uVolt = map(analogRead(PIN_IN_THERMOCOUPLE), 0, 1023, 0, 5000000);

  int temp;    
  temp = tc.update(uVolt);

  float dutyCycle;
  dutyCycle = float(pid.update(PID_ENABLE, temp, temperatureSetpoint) / 10);
  bool heatCycle;
  heatCycle = dc.update(true, 60000, dutyCycle);

  //digitalWrite(PIN_OUT_HEATER, heatCycle);

  //text_sp.text = String(temperatureSetpoint);
  //tft.println(temperatureSetpoint);

  //delay(200);
  if (screenRefresh.update(!screenRefresh.getTimerDone(), 500)) {
    updateIntegerField(&field_sp, temperatureSetpoint);
  }


}



