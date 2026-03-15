#include <Arduino.h>
#include "Sensor555/Sensor555.h"
#include "AwfulPID.h"
#include "DutyCycle.h"
#include "GeneralFunctions.h"
#include "Adafruit_GFX.h" 
#include "Adafruit_TFTLCD.h"
#include "MCUFRIEND_kbv.h"

#define PIN_IN_RTD A5
#define PIN_OUT_HEATER 13

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
  int base;
  int val;
};

Sensor555 rtd;
TableDataPoint rtdLookupTable[8] 
{{241,110},
{253,105},
{278,96},
{330,84},
{424,70},
{520,62},
{730,50},
{2015,24}};

TimerOnDelay screenRefresh;
MCUFRIEND_kbv tft;
IntegerField field_sp {.prop={.pos={.x=5, .y=5}, .color=CYAN, .bg=BLACK, .font=3, .label="SP:", .labelColor=WHITE, .labelOffs=0, .unit="C", .padding = 5}, .base=1, .val=0};
IntegerField field_temp {.prop={.pos={.x=5, .y=35}, .color=CYAN, .bg=BLACK, .font=3, .label="PV:", .labelColor=WHITE, .labelOffs=0, .unit="C", .padding = 5}, .base=1, .val=0};
IntegerField field_heat {.prop={.pos={.x=5, .y=65}, .color=CYAN, .bg=BLACK, .font=3, .label="OUT:", .labelColor=WHITE, .labelOffs=0, .unit="%", .padding = 5}, .base=1, .val=0};

// Configure PID for an output in seconds correlating to the duty cycle of the heater.
// duty cycle of 0-100% * 10
// 10 second PID period
AwfulPID pid;
const PIDConfiguration pidCfg {2000, 0, 1000, true, 30, 5};
const PIDParameters pidParam {1.0, 0.0, 0.0};

DutyCycle dc;

int SP_temp;

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
  field->val = val / field->base;
  tft.setCursor(pX_val, pos->y);  
  tft.setTextColor(prop->color);
  tft.print(field->val);
  tft.setTextColor(prop->labelColor);
  tft.setCursor(tft.getCursorX() + prop->padding, pos->y);
  tft.print(prop->unit);
}

void setup() {
  Serial.begin(9600);

  digitalWrite(PIN_OUT_HEATER, HIGH);
  pinMode(PIN_OUT_HEATER, OUTPUT);

  pinMode(PIN_IN_RTD, INPUT);
  rtd.setup(5000, -0.070528, 145.43, 9999);
  
  pid.setConfig(pidCfg);
  pid.setParam(pidParam);

  tft.reset();
  tft.begin(tft.readID());
  tft.fillScreen(BLACK);
  initializeFieldProperties(&field_sp.prop);
  initializeFieldProperties(&field_temp.prop);
  initializeFieldProperties(&field_heat.prop);
  
  digitalWrite(PIN_OUT_HEATER, HIGH);
  pinMode(PIN_OUT_HEATER, OUTPUT);

  delay(1000);
}
    
void loop() {
  SP_temp = 1900;

  //long tcScaled;
  // Vin = (Vref(5V)/1024)*ADC(1023) = 4.995117V
  // Vin = (Vref(1.1V)/1024)*ADC(1023) = 1.098926V

  float temp;    
  temp = rtd.update(digitalRead(PIN_IN_RTD));
  temp = scaleFloatByTable(rtd.getPulseLength(), rtdLookupTable, 8);

  float dutyCycle = pid.update(PID_ENABLE, temp, SP_temp);
  dutyCycle = 16.0;
  
  bool heatCycle = dc.update(true, 30000, dutyCycle);
  //digitalWrite(PIN_OUT_HEATER, !heatCycle);


  if (screenRefresh.update(!screenRefresh.getTimerDone(), 500)) {
    updateIntegerField(&field_sp, SP_temp);
    updateIntegerField(&field_temp, temp);
    updateIntegerField(&field_heat, dutyCycle);
  }

}



