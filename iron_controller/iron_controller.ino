#include <SSD1306Wire.h>
#include <math.h> 
#include "esp_adc_cal.h"
#include "ArduPID.h"
#include "EEPROM.h"

#include <BluetoothSerial.h>
#include <JsonParser.h>

ArduPID pidController_1;
ArduPID pidController_2;

// HelTec ESP32 DevKit parameters
#define D3 4
#define D5 15
SSD1306Wire  oled(0x3c, D3, D5);
BluetoothSerial serialBT;
EEPROMClass  parametersStorage("eeprom", 0x500);


static esp_adc_cal_characteristics_t adc1_chars;
esp_adc_cal_value_t val_type;

#define LED_PIN 2
#define POWER_PIN_1 17
#define POWER_PIN_2 5

#define TEMP_INPUT_PIN_1 36
#define TEMP_INPUT_PIN_2 37
#define NO_OF_SAMPLES   64          //Multisampling


// setting PWM properties
const int freq = 100;
const int resolution = 8;
const int PWM_CHANNEL_1 = 1;
const int PWM_CHANNEL_2 = 1;

const float p = 20;
const float i = 0.5;
const float d = 3;

#define SET_POINT_1 70
#define SET_POINT_2 80

class CIronParameters {
  public:
  char name[64];
  unsigned short temp1;
  unsigned short temp2;
};

CIronParameters g_ironParameters;

void initializeIronParameters()
{
  strcpy(g_ironParameters.name,"TailorsIron");
  g_ironParameters.temp1 = 150;
  g_ironParameters.temp2 = 150;
}


void setupStorage(){
    if (!parametersStorage.begin(parametersStorage.length())) {
    Serial.println("Failed to initialise parametersStorage");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }

  parametersStorage.get(0,g_ironParameters);
  if (g_ironParameters.temp1 < 50 || g_ironParameters.temp1 > 250) {
    Serial.println("Forcing uninitialized parameters");
    initializeIronParameters(); // restore defaults
    parametersStorage.put(0,g_ironParameters);
    parametersStorage.commit();
  } else {
    Serial.println("Stored parameters are OK");
    Serial.println(g_ironParameters.name);
  }
}
void setupPins(){
   // configure LED PWM functionalitites
  ledcSetup(PWM_CHANNEL_1, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(POWER_PIN_1, PWM_CHANNEL_1);
  
  ledcSetup(PWM_CHANNEL_2, freq, resolution);
  ledcAttachPin(POWER_PIN_2, PWM_CHANNEL_2);
}


double pidInput_1,pidOutput_1,pidSetpoint_1;
double pidInput_2,pidOutput_2,pidSetpoint_2;

void setupPIDs(){
  pidSetpoint_1 = SET_POINT_1;
  setupPID(pidController_1, pidInput_1, pidOutput_1, pidSetpoint_1);
  pidSetpoint_2 = SET_POINT_2;
  setupPID(pidController_2, pidInput_2, pidOutput_2, pidSetpoint_2);
}

void setupPID(ArduPID &pidController, double &pidInput, double &pidOutput, double &pidSetpoint){
  const int WindowSize = 255;
  pidController.begin(&pidInput, &pidOutput, &pidSetpoint, p, i, d);
  pidController.stop();
  pidController.setOutputLimits(0, WindowSize);
  pidController.setBias(0);
  pidController.setWindUpLimits(-100, 100); // Growth bounds for the integral term to prevent integral wind-up
  pidController.setSampleTime(500); // starts timer
  pidController.start();
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
    Serial.begin(115200);//initialize the serial monitor
    oledSetup();
    setupStorage();
    setupPins();
    setupPIDs();
    serialBT.begin("TailorsIron"); //Bluetooth device name
    setupADC();
}




void loop() {
  unsigned short raw;
  raw=readRaw(TEMP_INPUT_PIN_1);
  displayRaw(raw,1);
  unsigned short mV = rawToVoltagemV(raw);
  displayVoltage(mV,1);
  short temp1 = voltageToTemperature(mV);
  displayTemperature(temp1,1);

  raw=readRaw(TEMP_INPUT_PIN_2);
  displayRaw(raw, 2);
  mV = rawToVoltagemV(raw);
  displayVoltage(mV, 2);
  short temp2 = voltageToTemperature(mV);
  displayTemperature(temp2, 2);

  delay (100);

  pidInput_1=temp1;
  pidController_1.compute();
  displayOutput(pidOutput_1,1);
  ledcWrite(PWM_CHANNEL_1, pidOutput_1);

  pidInput_2=temp2;
  pidController_2.compute();
  displayOutput(pidOutput_2,2);
  ledcWrite(PWM_CHANNEL_2, pidOutput_2);
  processBT();
}


JsonParser<100> parser;

void processBT(){
  if (serialBT.available()) {
    char * command=read_from_BT();
    Serial.println("Received from bluetooth");
    Serial.println(command);
    //char * nl=strchr(command,'\n');
    //if (NULL != nl && nl < command+BT_BUFFER_SIZE){
    //  *nl='\0';
    //}
    JsonHashTable parametersHash = parser.parseHashTable(command);
    if (!parametersHash.success())
    {
        Serial.println("parsing failed");
    } else {
        Serial.println("command executed");
    }
  }
}
  
char * read_from_BT() {
  #define BT_BUFFER_SIZE 200
  static char buffer[BT_BUFFER_SIZE];
  bool go=true;
  int i=0;
  char c;
  while (go){
    if (serialBT.available()){
      c=serialBT.read();
      buffer[i++]=c;
    }
    if (c==0) {
      return buffer;
    }
    if (c=='\n' && i>0) {
      buffer[i-1]='\0';
      return buffer;
    }
    delay(2);
  }
}


unsigned short readRaw(int input) {
  unsigned long rawReading=0;
  for (int i=0;i<NO_OF_SAMPLES;i++){
    rawReading += analogRead(input);
  }
  return rawReading/NO_OF_SAMPLES;
}

const float Vdd=3.306f;
const float R0= 1200.0f; // measured
const float Tz= 273.15f;
const float Tc1= 25.0f +Tz; // thermistor calibration point
const float R1=100000.0f;   // thermistor type
const float beta=4300.0f;   // from thermistor data or measurements

const float mVIncrement = 0.953f;  // ESP32 ADV using default -11db setting (4095 is 1.1V without attenuation)


void displayVoltage(short voltage, int lineNo) {
  char voltageStr[30];

  sprintf(voltageStr,"%4hd mV",voltage);
  displayAt(45,lineNo,40,voltageStr);
}

void displayRaw(unsigned short raw, int lineNo) {
  char rawStr[30];

  sprintf(rawStr,"0x%04hx",raw);
  displayAt(2,lineNo,40,rawStr);
}

void displayTemperature(short temp, int lineNo) {
  char tempStr[30];

  sprintf(tempStr,"%3hd C",temp);
  displayAt(100,lineNo,40,tempStr);
}

void displayOutput(short value, int lineNo) {
  char valueStr[30];

  oled.setColor(BLACK);
  oled.fillRect(40,20+10* lineNo,80,9);
  oled.setColor(WHITE);
  oled.drawProgressBar(40,22+10* lineNo,80,8,value*100/255);
  sprintf(valueStr,"%3hd",value);
  displayAt(1, lineNo + 2, 39, valueStr);
}

unsigned short rawToVoltagemV(unsigned short rawValue){
  uint32_t voltage;
  voltage = esp_adc_cal_raw_to_voltage(rawValue, &adc1_chars);
  return voltage;
}

short voltageToTemperature(unsigned short voltagemV){
  float V = voltagemV / 1000.0f;
  float temp;
  
  //temp= 1.0f/(1/Tc1 - log(R1/((Vdd-V)*R0/V))/beta); // resistor on lower leg
  temp = 1.0f/(1/Tc1 - log(R1/(R0*V/(Vdd-V)))/beta); // resistor on upper leg
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
  oled.drawString(0 , 50, &g_ironParameters.name[0]);
  oled.display();
}
