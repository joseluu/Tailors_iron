#include <SSD1306Wire.h>
#include <math.h> 
#include "esp_adc_cal.h"
#include "ArduPID.h"

ArduPID pidController;

// HelTec ESP32 DevKit parameters
#define D3 4
#define D5 15
SSD1306Wire  oled(0x3c, D3, D5);

static esp_adc_cal_characteristics_t adc1_chars;
esp_adc_cal_value_t val_type;

double pidInput,pidOutput,pidSetpoint;
const float p = 20;
const float i = 0.5;
const float d = 3;

#define LED_PIN 2
#define POWER_PIN 17

const int WindowSize = 255;
// setting PWM properties
const int freq = 100;
const int PWM_CHANNEL = 1;
const int resolution = 8;

#define SET_POINT 50

unsigned long windowStartTime;

  
void setupPins(){
   // configure LED PWM functionalitites
  ledcSetup(PWM_CHANNEL, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(POWER_PIN, PWM_CHANNEL);
}

void setupPID(){
  pidController.begin(&pidInput, &pidOutput, &pidSetpoint, p, i, d);
  pidController.stop();
  pidController.setOutputLimits(0, WindowSize);
  pidController.setBias(0);
  pidController.setSampleTime(500);
  pidController.setWindUpLimits(-100, 100); // Growth bounds for the integral term to prevent integral wind-up
  windowStartTime = millis();
  pidController.start();
  pidSetpoint=SET_POINT;
}
void setupADC(){
      //Characterize ADC at particular atten
    val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 0, &adc1_chars);
    //Check type of calibration value used to characterize ADC
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        Serial.println("eFuse Vref");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        Serial.println("Two Point");
    } else {
        Serial.println("Default");
    }
}

void setup() {
    oledSetup();
    setupPins();
    setupPID();
    Serial.begin(115200);//initialize the serial monitor
    setupADC();
}




void loop() {
  unsigned short raw;
  raw=readRaw();
  displayRaw(raw);
  unsigned short mV = rawToVoltagemV(raw);
  displayVoltage(mV);
  short temp = voltageToTemperature(mV);
  displayTemperature(temp);

  delay (100);

  pidInput=temp;
  pidController.compute();
  displayOutput(pidOutput);

  ledcWrite(PWM_CHANNEL, pidOutput);
}

#define TEMP_INPUT_PIN 36
#define NO_OF_SAMPLES   64          //Multisampling

unsigned short readRaw() {
  unsigned long rawReading=0;
  for (int i=0;i<NO_OF_SAMPLES;i++){
    rawReading += analogRead(TEMP_INPUT_PIN);
  }
  return rawReading/NO_OF_SAMPLES;
}

const float Vdd=3.306f;
const float R0= 3240.0f;
const float Tz= 273.15f;
const float Tc1= 25.0f +Tz; // thermistor calibration point
const float R1=100000.0f;
const float beta=4300.0f;  // from thermistor data or measurements

const float mVIncrement = 0.952f;  // ESP32 ADV using default -11db setting (4095 is 1.1V without attenuation)


void displayVoltage(short voltage) {
  char voltageStr[30];

  sprintf(voltageStr,"%4hd mV",voltage);
  displayAt(45,1,40,voltageStr);
}

void displayRaw(unsigned short raw) {
  char rawStr[30];

  sprintf(rawStr,"0x%04hx",raw);
  displayAt(2,1,40,rawStr);
}

void displayTemperature(short temp) {
  char tempStr[30];

  sprintf(tempStr,"%3hd C",temp);
  displayAt(100,1,40,tempStr);
}

void displayOutput(short value) {
  char valueStr[30];

  sprintf(valueStr,"%4hd",value);
  displayAt(10,2,40,valueStr);
}

unsigned short rawToVoltagemV(unsigned short rawValue){
  #if 0
  float value;
  value= rawValue * mVIncrement;
  return value;
  #else
  uint32_t voltage;
  voltage = esp_adc_cal_raw_to_voltage(rawValue, &adc1_chars);
  return voltage;
  #endif
}

short voltageToTemperature(unsigned short voltagemV){
  float V = voltagemV / 1000.0f;
  float temp;
  
  temp= 1.0f/(1/Tc1 - log(R1/((Vdd-V)*R0/V))/beta);
  temp= temp-Tz;
  return temp;
}

void displayAt(unsigned short x, unsigned short lineNo, unsigned short xSpan, char * text){
  oled.setFont(ArialMT_Plain_10);
  oled.setTextAlignment(TEXT_ALIGN_LEFT);

  oled.setColor(BLACK);
  oled.fillRect(x,10*lineNo,xSpan,10);
  oled.setColor(WHITE);
  oled.drawString(x,10*lineNo,text);
  oled.display();
}


void oledSetup(void) {
  // reset OLED
  pinMode(16,OUTPUT); 
  digitalWrite(16,LOW); 
  delay(50); 
  digitalWrite(16,HIGH); 
  
  oled.init();
  oled.clear();
  oled.flipScreenVertically();
  oled.setFont(ArialMT_Plain_10);
  oled.setTextAlignment(TEXT_ALIGN_LEFT);
  oled.drawString(0 , 0, "START" );
  oled.display();
}
