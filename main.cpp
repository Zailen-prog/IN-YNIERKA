////////////// LIBRARIES ///////////////////////
#include <Arduino.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <PinChangeInterrupt.h>
#include <Rotary.h>
#include <U8g2lib.h>
#include <ArduinoJson.h>

#define SIZEOF(a) sizeof(a) / sizeof(a[0])

////////////// DEFINE PINS /////////////////////

#define SENSORS_BUS 12
#define FAN_HOT_SIDE 9
#define FAN_COLD_SIDE 10

#define PELTIER_PIN 3
#define ENCODER_A 4
#define ENCODER_B 5
#define ENCODER_C 6

///////////////////////////////////////////////////////////////////

#define MAX_SET_TEMP 80
#define MIN_SET_TEMP 10

//////////////////// TEMPERATURE SENSOR ///////////////////////////

OneWire oneWire(SENSORS_BUS); // create oneWire object
DallasTemperature sensors(&oneWire);  // Pass our oneWire reference to Dallas Temperature.
#define BIT_RESOLUTION 12 // sensors resolution
unsigned long MILLIS_TO_WAIT_FOR_CONVERSION = 750;
unsigned long last_time_requested = 0;  // stores time when temperature was last time requested

// sensors adresses

DeviceAddress addr_hot = { 0x28, 0xA7, 0x56, 0x8B, 0x0D, 0x00, 0x00, 0x93 };
DeviceAddress addr_outside = { 0x28, 0xBE, 0xA8, 0x75, 0xD0, 0x01, 0x3C, 0xEE };
DeviceAddress addr_cold = { 0x28, 0x0D, 0xFE, 0x69, 0x0D, 0x00, 0x00, 0x43 };
DeviceAddress addr_inside = { 0x28, 0x2E, 0xD3, 0x8A, 0x0D, 0x00, 0x00, 0x79 };

/////////////////////////////////////////////////////////////////////

////////////// CONTROL //////////////////////

float SET_TEMP = 25;

///////////////////////////////////////////////

/////////////// OLED ////////////////////////

U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

enum OLED_STATES{
  _MAIN,
  _MENU,
  _EDIT_TSET,
  _CHANGE_TSET_SOURCE,
  _CHANGE_FAN_POWER
};
enum OLED_STATES OLED_STATE = _MAIN;

enum TSET_SOURCE_OPTIONS{
  _USER,
  _USB
};
enum TSET_SOURCE_OPTIONS SELECTED_TSET_OPTION = _USER;

const char *const MENU_OPTIONS[] = {"Edit Tset","Tset Source", "Edit Fan", "Exit"};
const char *const TSET_SOURCES[] = {"User", "USB"};

byte HOVERED_MENU_OPTION = 0;
byte HOVERED_TSET_OPTION = SELECTED_TSET_OPTION;
float CURRENT_EDIT_SET_TEMP = SET_TEMP;

/////////////////////////////////////////////

Rotary rotary = Rotary(ENCODER_B, ENCODER_A);
unsigned long LAST_TIME_PRESSED = 0;

void analogWrite25k(byte pin, int value){
  switch (pin) {
      case 9:
          OCR1A = value;
          break;
      case 10:
          OCR1B = value;
          break;
      default:
          break;
  }
}

////////////////////////////////////////////

enum control_states {
  _HEATING,
  _COOLING
};

struct PID_controller {
  float p;
  float i;
  float kp;
  float ki;
  float output;
  float error;
  byte limMax = 1;
  byte limMin = 0;
};

PID_controller PID;

control_states control_state;

float fan_power = 90;
bool flag_change;
byte CURRENT_EDIT_FAN_POWER;

unsigned long last_time_fan_change = millis();
unsigned long timer = millis();
float diff = 0;

void PIDcontroller(const float& inside, const float& hot, const float& cold, const float& outside){
  float Ku = 0.6;
  float Tu = 108;
  PID.kp = 0.45*Ku;
  PID.ki = PID.kp/(0.8*Tu);

  PID.error = SET_TEMP - inside;

  PID.p = PID.kp * PID.error;

  PID.i = PID.i + PID.ki * PID.error*((millis()-timer)/1000);

 // anti-windup
  if (PID.i > PID.limMax)
    PID.i = PID.limMax;
  else if(PID.i < PID.limMin)
    PID.i = PID.limMin;

  PID.output = PID.p + PID.i;

// output clamping
  if(PID.output < PID.limMin)
    PID.output = PID.limMin;
  else if(PID.output > PID.limMax)  
    PID.output = PID.limMax;

  if(control_state == _COOLING){
    PID.output = 1 - PID.output;
    if (PID.output <  0.05)
      PID.output = 0.05;
    else if(PID.output >  0.6)
      PID.output = 0.6;
  }

  analogWrite(PELTIER_PIN,PID.output*255);
  timer = millis();
}


void RenderMainPage(const float& inside, const float& hot, const float& cold, const float& outside){
  u8g2.setFontPosTop();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(0,0);
  u8g2.print(F("Tset: "));

  u8g2.setCursor(30, 0);
  u8g2.print(SET_TEMP, 1);

  u8g2.setFontPosBottom();
  u8g2.setCursor(0, 64);
  u8g2.print(F("H: "));

  u8g2.setCursor(15, 64);
  u8g2.print(hot, 1);

  u8g2.setCursor(45,64);
  u8g2.print(F("C: "));

  u8g2.setCursor(60, 64);
  u8g2.print(cold, 1);

  u8g2.setCursor(90,64);
  u8g2.print(F("O: "));

  u8g2.setCursor(105, 64);
  u8g2.print(outside, 1);

  u8g2.setFontPosTop();
  u8g2.setCursor(80, 0);
  u8g2.print(TSET_SOURCES[SELECTED_TSET_OPTION]);

  u8g2.setFont(u8g2_font_u8glib_4_tf);
  u8g2.setCursor(55, 0);
  u8g2.print(char(176));

  u8g2.setCursor(60, 0);
  u8g2.print(F("C"));

  u8g2.setFont(u8g2_font_7x14B_tf);
  u8g2.setCursor(93, 20);
  u8g2.print(char(176));

  u8g2.setCursor(100, 20);
  u8g2.print(F("C"));

  u8g2.setFont(u8g2_font_logisoso24_tn);
  u8g2.setFontPosCenter();
  u8g2.setCursor(33, 35);
  u8g2.print(inside, 1);

}

void RenderMenu(void){
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setFontPosTop();
  u8g2.setFontMode(1);
  for (unsigned int i = 0; i < SIZEOF(MENU_OPTIONS); i++) {
    if (i == HOVERED_MENU_OPTION){
      u8g2.setDrawColor(1);
      const char* s = MENU_OPTIONS[i];
      byte w = u8g2.getStrWidth(s);      // returns w = 24
      byte h = u8g2.getAscent()-u8g2.getDescent();
      u8g2.drawBox(10-1,(i * 15), w+2, h+2);
      u8g2.setDrawColor(2);
    }
    u8g2.drawStr(10, 1 + i * 15, MENU_OPTIONS[i]);
  }
}

void RenderEditTset(){
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(40, 0);
  u8g2.print(F("Edit Tset"));

  u8g2.setFont(u8g2_font_logisoso38_tn);
  u8g2.setDrawColor(1);
  u8g2.setCursor(22, 20);
  u8g2.print(CURRENT_EDIT_SET_TEMP, 1);
  u8g2.setFont(u8g2_font_7x14B_tf);
  u8g2.setCursor(110, 22);
  u8g2.print(char(176));
  u8g2.setCursor(117, 22);
  u8g2.print(F("C"));
}

void RenderEditTsetSource(void){
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setFontPosTop();
  u8g2.setFontMode(1);
  for (unsigned int i = 0; i < SIZEOF(TSET_SOURCES); i++) {
    if (i == HOVERED_TSET_OPTION){ 
      u8g2.setDrawColor(1);
      const char* s = TSET_SOURCES[i];
      byte w = u8g2.getStrWidth(s);      // returns w = 24
      byte h = u8g2.getAscent()-u8g2.getDescent();
      u8g2.drawBox(10-1,(i * 15), w+2, h+2);
      u8g2.setDrawColor(2);
    }
    u8g2.drawStr(10, 1 + i * 15, TSET_SOURCES[i]);
  }
}

void RenderEditFanPower(){
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(25, 0);
  u8g2.print(F("Edit Fan Power"));

  u8g2.setFont(u8g2_font_logisoso38_tn);
  u8g2.setDrawColor(1);
  u8g2.setCursor(40, 20);
  u8g2.print(CURRENT_EDIT_FAN_POWER, 1);
  u8g2.setCursor(110, 22);
  u8g2.print(F("%"));
}

void draw(const float& inside, const float& hot, const float& cold, const float& outside) {
  u8g2.firstPage();
  do {
    switch (OLED_STATE) {
    case _MAIN:
      RenderMainPage(inside, hot, cold, outside);
      break;
    case _MENU:
      RenderMenu();
      break;
    case _EDIT_TSET:
      RenderEditTset();
      break;
    case _CHANGE_TSET_SOURCE:
      RenderEditTsetSource();
      break;
    case _CHANGE_FAN_POWER:
      RenderEditFanPower();
      break;
    default:
      break;
    } 
  } while ( u8g2.nextPage() );

}

void enRotate() {
  unsigned char result = rotary.process();

  switch (OLED_STATE) {
    case _MENU:
      if (result == DIR_CW) {
        if (HOVERED_MENU_OPTION == SIZEOF(MENU_OPTIONS) - 1) {
          HOVERED_MENU_OPTION = 0;
        } else {
          HOVERED_MENU_OPTION++;
        }
      } else if (result == DIR_CCW) {
        if (HOVERED_MENU_OPTION == 0) {
          HOVERED_MENU_OPTION = SIZEOF(MENU_OPTIONS) - 1;
        } else {
          HOVERED_MENU_OPTION--;
        }
      }
      
      break;

    case _EDIT_TSET:
      if (result == DIR_CW) {
        CURRENT_EDIT_SET_TEMP = CURRENT_EDIT_SET_TEMP < MAX_SET_TEMP ? CURRENT_EDIT_SET_TEMP + 0.5 : MAX_SET_TEMP;
      } else if (result == DIR_CCW) {
        CURRENT_EDIT_SET_TEMP = CURRENT_EDIT_SET_TEMP > MIN_SET_TEMP ? CURRENT_EDIT_SET_TEMP - 0.5 : MIN_SET_TEMP;
      }
      break;

    case _CHANGE_TSET_SOURCE:
      if (result == DIR_CW) {
        if (HOVERED_TSET_OPTION == SIZEOF(TSET_SOURCES) - 1) {
          HOVERED_TSET_OPTION = 0;
        } else {
          HOVERED_TSET_OPTION++;
        }
      } else if (result == DIR_CCW) {
        if (HOVERED_TSET_OPTION == 0) {
          HOVERED_TSET_OPTION = SIZEOF(TSET_SOURCES) - 1;
        } else {
          HOVERED_TSET_OPTION--;
        }
      }
      break;

      case _CHANGE_FAN_POWER:
      if (result == DIR_CW) {
        CURRENT_EDIT_FAN_POWER = CURRENT_EDIT_FAN_POWER < 100 ? CURRENT_EDIT_FAN_POWER + 2 : 100;
      } else if (result == DIR_CCW) {
        CURRENT_EDIT_FAN_POWER = CURRENT_EDIT_FAN_POWER > 50 ? CURRENT_EDIT_FAN_POWER - 2 : 50;
      }
      break;

    default:
      break;
  }
}

void enButton() {
  if ((millis() - LAST_TIME_PRESSED) >= 1000) {
    switch (OLED_STATE) {
      case _MAIN:
        OLED_STATE = _MENU;
        HOVERED_MENU_OPTION = 0;
        break;

      case _MENU:
        if (HOVERED_MENU_OPTION == 0 && SELECTED_TSET_OPTION == _USER) {
          OLED_STATE = _EDIT_TSET;
          CURRENT_EDIT_SET_TEMP = SET_TEMP;
        } else if (HOVERED_MENU_OPTION == 1) {
          OLED_STATE = _CHANGE_TSET_SOURCE;
          HOVERED_TSET_OPTION = SELECTED_TSET_OPTION;
        } else if (HOVERED_MENU_OPTION == 2) {
          OLED_STATE = _CHANGE_FAN_POWER;
        } else if (HOVERED_MENU_OPTION == 3) {
          OLED_STATE = _MAIN;
        }
        break;

      case _EDIT_TSET:
        SET_TEMP = CURRENT_EDIT_SET_TEMP;
        EEPROM.write(0,SET_TEMP);
        OLED_STATE = _MAIN;
        break;

      case _CHANGE_TSET_SOURCE:
        if (HOVERED_TSET_OPTION == 0){
          SELECTED_TSET_OPTION = _USER;
          EEPROM.write(1,0);
        }
        else {
          SELECTED_TSET_OPTION = _USB;
          EEPROM.write(1,1);
        }
        OLED_STATE = _MENU;
        break;

      case _CHANGE_FAN_POWER:
        fan_power = CURRENT_EDIT_FAN_POWER;
        EEPROM.write(2,fan_power);
        flag_change = true;
        OLED_STATE = _MENU;
        break;

      default:
        break;
    }
    LAST_TIME_PRESSED = millis();
  }
}

void setup() {

  // Configure Timer 1 for PWM @ 25 kHz.
  TCCR1A = 0;           // undo the configuration done
  TCCR1B = 0;           // vy the Arduino core library
  TCNT1  = 0;           // reset timer
  TCCR1A = _BV(COM1A1)  // non-inverted PWM on ch. A
          | _BV(COM1B1)  // same on ch; B
          | _BV(WGM11);  // mode 10:ph. correct PWM, TOP=ICR1
  TCCR1B = _BV(WGM13)   // ditto
          | _BV(CS10);   // prescaler = 1
  ICR1   = 320;         // TOP = 320

  TCCR2B = TCCR2B & B11111000 | B00000001;

  // Set the PWM pins as output.
  pinMode(FAN_HOT_SIDE, OUTPUT);
  pinMode(FAN_COLD_SIDE, OUTPUT);
  pinMode(PELTIER_PIN,OUTPUT);

  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);

  // set PWM values to 0
  analogWrite25k(FAN_HOT_SIDE,0);
  analogWrite25k(FAN_COLD_SIDE,0);
  analogWrite(PELTIER_PIN,0);

  pinMode(ENCODER_C, INPUT_PULLUP);

  attachPCINT(digitalPinToPCINT(ENCODER_A), enRotate, CHANGE);
  attachPCINT(digitalPinToPCINT(ENCODER_B), enRotate, CHANGE);
  attachPCINT(digitalPinToPCINT(ENCODER_C), enButton, FALLING);

  // rotary.begin(false);


  SET_TEMP = EEPROM.read(0);
  fan_power = EEPROM.read(2);
  CURRENT_EDIT_FAN_POWER = fan_power;
  if (EEPROM.read(1) == 0)
    SELECTED_TSET_OPTION = _USER;
  else
    SELECTED_TSET_OPTION = _USB;
  
  HOVERED_TSET_OPTION = SELECTED_TSET_OPTION;

  sensors.begin();  // initialize sensors
  sensors.setResolution(BIT_RESOLUTION);  // set resolution
  sensors.setWaitForConversion(0);  // dont wait for conversion 
  sensors.requestTemperatures();  // request temperatures

  last_time_requested = millis();
  

  // init oled
  u8g2.begin();

  // begin serial comunitation
  Serial.begin(9600);

  last_time_fan_change = millis();
}

bool flag_first_run = true;
float hot;
float cold;
float inside;
float outside;
StaticJsonDocument<64> doc;


void loop() {
  if(SELECTED_TSET_OPTION == _USB){
    if (Serial.available()) {
      DeserializationError JSONerror = deserializeJson(doc, Serial);
      if (!JSONerror) {
        if (doc["set"]) {
          SET_TEMP = doc["set"];
          EEPROM.write(0,SET_TEMP);
        }
      }
    }
  }

  if (millis() - last_time_requested >= MILLIS_TO_WAIT_FOR_CONVERSION && millis() - last_time_requested >= 1000){
    Serial.println(millis() - last_time_requested);
    
    inside = sensors.getTempC(addr_inside);
    cold = sensors.getTempC(addr_cold);
    hot = sensors.getTempC(addr_hot);
    outside = sensors.getTempC(addr_outside);
    sensors.requestTemperatures();
    last_time_requested = millis();
    
    if((SET_TEMP > outside && control_state == _COOLING) || flag_first_run){
      control_state = _HEATING;
      analogWrite25k(FAN_HOT_SIDE,320);
      analogWrite25k(FAN_COLD_SIDE,0);
      last_time_fan_change = millis();
      flag_first_run = false;
      flag_change = true;
    }
    else if((SET_TEMP < outside && control_state == _HEATING) || flag_first_run){
      control_state = _COOLING;
      analogWrite25k(FAN_COLD_SIDE,320);
      analogWrite25k(FAN_HOT_SIDE,0);
      last_time_fan_change = millis();
      flag_first_run = false;
      flag_change = true;
    }

    if(millis() - last_time_fan_change >= 5000 && control_state == _HEATING && flag_change){
      analogWrite25k(FAN_HOT_SIDE,(fan_power/100)*320);
      analogWrite25k(FAN_COLD_SIDE,0);
      flag_change = false;
    } else if (millis() - last_time_fan_change >= 5000 && control_state == _COOLING && flag_change){
      analogWrite25k(FAN_HOT_SIDE,0);
      analogWrite25k(FAN_COLD_SIDE,(fan_power/100)*320);
      flag_change = false;
    }
    
    PIDcontroller(inside, cold, hot, outside);
    
    doc["set"] = SET_TEMP;
    doc["inside"] = inside;
    doc["hot"] = hot;
    doc["cold"] = cold;
    doc["outside"] = outside;
    doc["fan-power"] = fan_power;
    doc["peltier-power"] = PID.output;

    serializeJson(doc, Serial);
    Serial.println();
    
  }
  draw(inside, hot, cold, outside);
}