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

#define SCREEN_BOOT 1
#define SCREEN_MAIN 2
#define SCREEN_SETTINGS 3
#define SCREEN_SETTINGS_PID 4

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

struct FloatField {    
  FieldProperties prop;
  float val;
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
// HMI
int screen, screenLast;
// Page 1 - Main
IntegerField field_sp {.prop={.pos={.x=5, .y=5}, .color=CYAN, .bg=BLACK, .font=3, .label="SP:", .labelColor=WHITE, .labelOffs=0, .unit="C", .padding = 5}, .base=1, .val=0};
IntegerField field_temp {.prop={.pos={.x=5, .y=35}, .color=CYAN, .bg=BLACK, .font=3, .label="PV:", .labelColor=WHITE, .labelOffs=0, .unit="C", .padding = 5}, .base=1, .val=0};
FloatField field_heat {.prop={.pos={.x=5, .y=65}, .color=CYAN, .bg=BLACK, .font=3, .label="OUT:", .labelColor=WHITE, .labelOffs=0, .unit="%", .padding = 5}, .val=0};
FloatField field_error {.prop={.pos={.x=5, .y=95}, .color=CYAN, .bg=BLACK, .font=3, .label="ERR:", .labelColor=WHITE, .labelOffs=0, .unit="C", .padding = 5}, .val=0};
FloatField field_integral {.prop={.pos={.x=5, .y=125}, .color=CYAN, .bg=BLACK, .font=3, .label="ACC:", .labelColor=WHITE, .labelOffs=0, .unit="%", .padding = 5}, .val=0};

// Configure PID for an output in seconds correlating to the duty cycle of the heater.
// duty cycle of 0-100% * 10
// 10 second PID period
AwfulPID pid;
const PIDConfiguration pidCfg {10000, -16, 16, false, 2, 5, 5.0};
const PIDParameters pidParam {0.5, 0.01, 0.0};
TableDataPoint pidBaseResponseTable[8] 
{{24,0},
{50,3},
{62,5},
{70,7},
{84,10},
{96,12},
{105,14},
{110,16}};

DutyCycle dc;

int SP_temp, temp, baseResponse, bias, pulseLength, dutyCycle;

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

void updateFloatField(struct FloatField* field, float val) {
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
  tft.print(field->val);
  tft.setTextColor(prop->labelColor);
  tft.setCursor(tft.getCursorX() + prop->padding, pos->y);
  tft.print(prop->unit);
}

void drawScreen(int screen) {
  switch (screen) {
    case SCREEN_BOOT:
      tft.setTextColor(RED);
      tft.setTextSize(4);
      tft.println("");
      tft.println("");
      tft.println("    O O ");
      tft.println("     |  ");
      tft.println("     <  ");
      tft.println("  *    *");
      tft.println("  *    *");
      tft.println("    **");
      break;

    case SCREEN_MAIN:
      initializeFieldProperties(&field_sp.prop);
      initializeFieldProperties(&field_temp.prop);
      initializeFieldProperties(&field_heat.prop);
      initializeFieldProperties(&field_error.prop);
      initializeFieldProperties(&field_integral.prop);
      break;
    
    case SCREEN_SETTINGS:
      break;

    case SCREEN_SETTINGS_PID:
      break;
  }
}

void updateField(int screen) { 
  switch (screen) {
        case SCREEN_BOOT:
        break;

        case SCREEN_MAIN:
          updateIntegerField(&field_sp, SP_temp);
          updateIntegerField(&field_temp, temp);
          updateFloatField(&field_heat, dutyCycle);
          updateFloatField(&field_error, pid.getError());
          updateFloatField(&field_integral, pid.getIntegral());
          break;
        
        case SCREEN_SETTINGS:
          break;

        case SCREEN_SETTINGS_PID:
          break;
    }
}

void refreshScreen() {
  if (screenRefresh.update(!screenRefresh.getTimerDone(), 500)) {
    if (screen != screenLast) {
      screenLast = screen;
      tft.fillScreen(BLACK);
      drawScreen(screen);
    } else {
      updateField(screen);
    }
  }
  
}

void waitForBootComplete() {
  TimerOnDelay bootTimer;
  //bootTimer.update(false, 2000);
  delay(1000);
  bool bootComplete = false;
  while (!bootComplete) {    
    rtd.update(digitalRead(PIN_IN_RTD));
    refreshScreen();
    bootComplete = bootTimer.update(true, 4000);
  }
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
  
  digitalWrite(PIN_OUT_HEATER, HIGH);
  pinMode(PIN_OUT_HEATER, OUTPUT);

  screen = SCREEN_BOOT;
  waitForBootComplete();
  screen = SCREEN_MAIN;
}
    
void loop() {
  SP_temp = 70;

  //long tcScaled;
  // Vin = (Vref(5V)/1024)*ADC(1023) = 4.995117V
  // Vin = (Vref(1.1V)/1024)*ADC(1023) = 1.098926V
  
  rtd.update(digitalRead(PIN_IN_RTD));
  pulseLength = rtd.getPulseLength();
  temp = scaleFloatByTable(pulseLength, rtdLookupTable, 8);
  baseResponse = scaleFloatByTable(SP_temp, pidBaseResponseTable, 8);
  if (pid.getError() > 25) {
    pid.setManual(50);
    bias = pid.update(PID_MANUAL, temp, SP_temp);
  } else {
    bias = pid.update(PID_ENABLE, temp, SP_temp);
  }
  dutyCycle = limitFloat(0, (baseResponse + bias), 50);
  /*
  Serial.print("base:");
  Serial.print(baseResponse);
  Serial.print(" bias:");
  Serial.print(bias);
  Serial.print(" dutyCycle::");
  Serial.print(dutyCycle);
  Serial.print(" error::");
  Serial.print(pid.getError());
  Serial.println("");
  */
  bool heatCycle = dc.update(true, 30000, dutyCycle);
  digitalWrite(PIN_OUT_HEATER, !heatCycle);

  refreshScreen();
}



