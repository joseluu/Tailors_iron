#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"
#include <SSD1306Wire.h>
#include <TimeLib.h>
#include <math.h>
#include "esp_adc_cal.h"
#include "ArduPID.h"
#include "EEPROM.h"

#include "rom/ets_sys.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"


#include <BluetoothSerial.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#include "Arduino_JSON.h"

ArduPID pidController_1;
ArduPID pidController_2;

// HelTec ESP32 DevKit parameters
#define D3 4
#define D5 15
SSD1306Wire  oled(0x3c, D3, D5);
BluetoothSerial serialBT;
EEPROMClass  parametersStorage("eeprom", 0x500);
#define FORCE_FLASH_INIT false


static esp_adc_cal_characteristics_t adc1_chars;
esp_adc_cal_value_t val_type;

#define LED_PIN 2
#define POWER_PIN_1 17
#define POWER_PIN_2 5

#define TEMP_INPUT_PIN_1 36
#define TEMP_INPUT_PIN_2 37
#define NO_OF_SAMPLES   64          //Multisampling

time_t endOfSessionTime = 1800; // default is 30mn

// setting PWM properties
const int freq = 1;
const int resolution = 8;
const int PWM_CHANNEL_1 = 1;
const int PWM_CHANNEL_2 = 2;

bool bDoRegulation = true;
bool bDebugDisplay = false;
bool bDebugForcePowerOff1 = false;
bool bDebugForcePowerOff2 = false;
bool bDebugForcePowerOn1 = false;
bool bDebugForcePowerOn2 = false;


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
  strcpy(g_ironParameters.name, "Fer de Lisa");
  g_ironParameters.temp1 = 150;
  g_ironParameters.temp2 = 150;
}


void setupStorage() {
  if (!parametersStorage.begin(parametersStorage.length())) {
    Serial.println("Failed to initialise parametersStorage");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }

  parametersStorage.get(0, g_ironParameters);
  if (FORCE_FLASH_INIT || g_ironParameters.temp1 < 50 || g_ironParameters.temp1 > 250) {
    Serial.println("Forcing uninitialized parameters");
    initializeIronParameters(); // restore defaults
    parametersStorage.put(0, g_ironParameters);
    parametersStorage.commit();
  } else {
    Serial.println("Stored parameters are OK");
    Serial.println(g_ironParameters.name);
  }
}
void setupPins() {
  // configure LED PWM functionalitites
  ledcSetup(PWM_CHANNEL_1, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(POWER_PIN_1, PWM_CHANNEL_1);

  ledcSetup(PWM_CHANNEL_2, freq, resolution);
  ledcAttachPin(POWER_PIN_2, PWM_CHANNEL_2);
}


double pidInput_1, pidOutput_1, pidSetpoint_1;
double pidInput_2, pidOutput_2, pidSetpoint_2;

void setupPIDs() {
  pidSetpoint_1 = SET_POINT_1;
  setupPID(pidController_1, pidInput_1, pidOutput_1, pidSetpoint_1);
  pidSetpoint_2 = SET_POINT_2;
  setupPID(pidController_2, pidInput_2, pidOutput_2, pidSetpoint_2);
}

void setupPID(ArduPID &pidController, double &pidInput, double &pidOutput, double &pidSetpoint) {
  const int WindowSize = 255;
  pidController.begin(&pidInput, &pidOutput, &pidSetpoint, p, i, d);
  pidController.stop();
  pidController.setOutputLimits(0, WindowSize);
  pidController.setBias(0);
  pidController.setWindUpLimits(-100, 100); // Growth bounds for the integral term to prevent integral wind-up
  pidController.setSampleTime(500); // starts timer
  pidController.start();
}

void setupADC() {
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

void serialBT_SPP_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    Serial.println("Client Connected has address:");
    for (int i = 0; i < 6; i++) {
      Serial.printf("%02X", param->srv_open.rem_bda[i]);
      if (i < 5) {
        Serial.print(":");
      }
    }
    Serial.println("");
  } else if (event == ESP_SPP_CLOSE_EVT) {
    Serial.println("Client disconnected");
  }
}

void setupBT() {
  bool bStatus = serialBT.begin(g_ironParameters.name); //Bluetooth device name
  if (! bStatus ) {
    Serial.println("Serial BT initialization failure");
  }
  serialBT.register_callback(serialBT_SPP_callback);
  serialBT.onData(serialBT_receive_callback);
}

void setupTime() {
  setTime(0);
}

bool bForcePowerState = true;
bool bTimedPowerState;

bool bSafetyActivated = false;
bool bStartedSafetyCount = false;
int safetyCount = 0;

int power1;
int power2;
#define POWERDIPLAYSIZE 15
char powerDisplay[POWERDIPLAYSIZE];

int previousProgress;

void setup() {
  Serial.begin(115200);//initialize the serial monitor
  oledSetup();
  setupStorage();
  setupPins();
  setupPIDs();
  setupBT();
  setupADC();
  setupTime();
}
    
void doPowerControl() {

    bTimedPowerState = (now() < endOfSessionTime);

    if (bTimedPowerState && bForcePowerState && ! bSafetyActivated) { // all conditions needed for power On
      power1 = pidOutput_1;
      power2 = pidOutput_2;
    } else {
      power1 = 0;
      power2 = 0;
    }
    ledcWrite(PWM_CHANNEL_1, power1);
    ledcWrite(PWM_CHANNEL_2, power2);
}


void doSafetyCount(short temp1, short temp2){
  if (temp1 > 40 && temp2 > 40) { // temperature measurement is working
    bSafetyActivated = false;
    bStartedSafetyCount = false;
  }
  if (safetyCount > 6000) { 
    bSafetyActivated = true;
    return;
  }
  if (temp1 < 40 || temp2 < 40) {
    if (bStartedSafetyCount) {
      safetyCount++;
    } else {
      bStartedSafetyCount = true;
      safetyCount = 0;
    }
  }
}
void loop() {
  unsigned short raw;
  raw = readRaw(TEMP_INPUT_PIN_1);
  displayRaw(raw, 1);
  unsigned short mV = rawToVoltagemV(raw);
  displayVoltage(mV, 1);
  short temp1 = voltageToTemperature(mV);
  displayTemperature(temp1, 1);

  raw = readRaw(TEMP_INPUT_PIN_2);
  displayRaw(raw, 2);
  mV = rawToVoltagemV(raw);
  displayVoltage(mV, 2);
  short temp2 = voltageToTemperature(mV);
  displayTemperature(temp2, 2);

  doSafetyCount(temp1, temp2);
  short tempInternal = temperatureInternal();

  delay (100);

  pidInput_1 = temp1;
  pidController_1.compute();
  pidInput_2 = temp2;
  pidController_2.compute();

  if (! bDoRegulation) { // debug mode
    pidOutput_1 = (bDebugForcePowerOff1 ? 0 : (bDebugForcePowerOn1 ? 255 : (pidSetpoint_1-50)*255/200 ));
    pidOutput_2 = (bDebugForcePowerOff2 ? 0 : (bDebugForcePowerOn2 ? 255 : (pidSetpoint_2-50)*255/200 ));
  }
  displayOutput(pidOutput_1, 1);
  displayOutput(pidOutput_2, 2);

  doPowerControl();
  standardDisplay(temp1,temp2,pidSetpoint_1, pidSetpoint_2, bTimedPowerState, endOfSessionTime - now());
  doPowerDisplay();
  
  processBT(temp1, temp2, tempInternal);
}

void doPowerDisplay(){
  int c=0;

  if (bSafetyActivated) { // Malfunction
    powerDisplay[c++]='M';
    powerDisplay[c++]='a';
    powerDisplay[c++]='l';
  } else {
    powerDisplay[c++]='O';
    if (bTimedPowerState && bForcePowerState) {
      powerDisplay[c++]='n';
      powerDisplay[c++]=' ';
    } else {
      powerDisplay[c++]='f';
      powerDisplay[c++]='f';
    }
  }
  powerDisplay[c++]=' ';
  powerDisplay[c++]=' ';
  powerDisplay[c++]=' ';
  powerDisplay[c++]=' ';
  powerDisplay[c++]=' ';
  if (power1 < 128)  powerDisplay[c++]='-';
  if (power1 >= 128)  powerDisplay[c++]='+';
  powerDisplay[c++]=' ';
  powerDisplay[c++]=' ';
  if (power2 < 128)  powerDisplay[c++]='-';
  if (power2 >= 128)  powerDisplay[c++]='+';
  powerDisplay[c++]=0;
  standardDisplayText2(powerDisplay);
}

char json_test[] =
  "{\"commands\": [ {\"setPoint1\": 150 }, { \"setPoint2:\": 150 }, { \"setTime\": \"15:45:41\""
  "}, { \"setEndOfSession\": \"16:45:34\" } ]                                               }";

int bDataRequested = 0;

void processBT(short temp1, short temp2, short tempInternal) {
  char * command;
  if ((command = read_from_BT()) != NULL) {
    Serial.println("Received from bluetooth");
    Serial.println(command);

    JSONVar receivedJson = JSON.parse(command);
    if (JSON.typeof(receivedJson) == "undefined") {
      Serial.println("json parsing failed");
      return;
    } else {
      Serial.println("command parsed");
    }
    bDataRequested = receivedJson["commands"]["getData"];
    if (bDataRequested) {
      Serial.println("Client needs data");
      return;
    } 
    bDebugDisplay = receivedJson["commands"]["debug"];

    bDebugForcePowerOn1 = receivedJson["commands"]["forceOn1"];
    bDebugForcePowerOn2 = receivedJson["commands"]["forceOn2"];
    bDebugForcePowerOff1 = receivedJson["commands"]["forceOff1"];
    bDebugForcePowerOff2 = receivedJson["commands"]["forceOff2"];
    bDoRegulation = ! (bDebugForcePowerOn1 ||
              bDebugForcePowerOn2 ||
              bDebugForcePowerOff1 ||
              bDebugForcePowerOff2);
              
    int newSetPoint1 = receivedJson["commands"]["setPoint1"];
    int newSetPoint2 = receivedJson["commands"]["setPoint2"];
      Serial.print("setPoint1 received: ");   
      Serial.println(newSetPoint1);
    if (newSetPoint1 != 0 && newSetPoint2 != 0) {
      if ((newSetPoint1 != g_ironParameters.temp1) ||
          (newSetPoint2 != g_ironParameters.temp2)) {
        g_ironParameters.temp1 = newSetPoint1;
        g_ironParameters.temp2 = newSetPoint2;
        parametersStorage.put(0, g_ironParameters);
      }
      pidSetpoint_1 = newSetPoint1;
      pidSetpoint_2 = newSetPoint2;
    }
    int newSetEndOfSession = receivedJson["commands"]["setEndOfSession"];
    if (newSetEndOfSession != 0) {
      Serial.print("newSetEndOfSession: ");
      Serial.println(newSetEndOfSession);
      endOfSessionTime = now() + newSetEndOfSession;
      bTimedPowerState = true;
      Serial.print("endOfSessionTime: ");
      Serial.println(endOfSessionTime);
    }
    int bNewDebugDisplay = receivedJson["commands"]["debugDisplay"];
    if (bDebugDisplay != bNewDebugDisplay) {
       displayFullErase();
       previousProgress = 0; // reset time progress bar
    }
    if (bNewDebugDisplay) {
      bDebugDisplay = bNewDebugDisplay;
    }
    bool bNewForcePowerState = receivedJson["commands"]["forcePowerState"];
    bForcePowerState = bNewForcePowerState;
  } else {
    static unsigned int previousSendTime;
    unsigned int nowSendTime = millis();
    if (nowSendTime < previousSendTime) { // wrap around
      previousSendTime = 0;
    }
    if (bDataRequested && (nowSendTime - previousSendTime) > 1000) { // we send status every second
      JSONVar sendJson;
      JSONVar statusJson;
      sendJson["status"] = statusJson;
      sendJson["status"]["setPoint1"] = (int) pidSetpoint_1;
      sendJson["status"]["temperature1"] = temp1;
      sendJson["status"]["power1"] = power1;
      sendJson["status"]["setPoint2"] = (int) pidSetpoint_2;
      sendJson["status"]["temperature2"] = temp2;
      sendJson["status"]["power2"] = power2;
      sendJson["status"]["time"] = now();
      sendJson["status"]["endOfSession"] = endOfSessionTime - now();
      sendJson["status"]["temperatureProcessor"] = tempInternal;
      sendJson["status"]["internalFan"] = 20;
      sendJson["status"]["debugDisplay"] = bDebugDisplay;
      sendJson["status"]["forcePowerState"] = bForcePowerState;
      sendJson["status"]["timedPowerState"] = bTimedPowerState;
      sendJson["status"]["debug"] = ! bDoRegulation;
      sendJson["status"]["forceOn1"] = bDebugForcePowerOn1;
      sendJson["status"]["forceOn2"] = bDebugForcePowerOn2;
      sendJson["status"]["forceOff1"] = bDebugForcePowerOff1;
      sendJson["status"]["forceOff2"] = bDebugForcePowerOff2;
      
      String jsonString = JSON.stringify(sendJson);

      serialBT.write((uint8_t*)jsonString.c_str(), jsonString.length());
      Serial.println(jsonString.c_str());
      previousSendTime = nowSendTime;
      bDataRequested = false;
    }
  }
}

#define BT_BUFFER_SIZE 500U
static char bufferBT[BT_BUFFER_SIZE];
static size_t lenBT = 0;

void serialBT_receive_callback(const uint8_t *data , size_t len) {
  // doc here: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/esp_spp.html#_CPPv4N18esp_spp_cb_param_t8data_indE
  Serial.print("in Receive callback: ");
  Serial.print(len);
  Serial.println(" bytes available");
  lenBT = len;
  strncpy(bufferBT, (const char *)data, min(len, BT_BUFFER_SIZE - 1));
  bufferBT[len] = 0;
}


//TODO use BluetoothSerial::onData to register a callback
char * read_from_BT() {
  if (lenBT == 0) {
    return NULL;
  }
  lenBT = 0;
  return bufferBT;
}


unsigned short readRaw(int input) {
  unsigned long rawReading = 0;
  for (int i = 0; i < NO_OF_SAMPLES; i++) {
    rawReading += analogRead(input);
  }
  return rawReading / NO_OF_SAMPLES;
}

const float Vdd = 3.333f;
const float R0 = 3900.0f; // measured
const float Tz = 273.15f;
const float Tc1 = 25.0f + Tz; // thermistor calibration point
const float R1 = 96000.0f; // thermistor type
const float beta = 3950.0f; // from thermistor data or measurements

const float mVIncrement = 0.953f;  // ESP32 ADV using default -11db setting (4095 is 1.1V without attenuation)

void displayVoltage(short voltage, int lineNo) {
  char voltageStr[30];
  if (!bDebugDisplay)
    return;
  sprintf(voltageStr, "%4hd mV", voltage);
  displaySmall(45, lineNo, 40, voltageStr);
}

void displayRaw(unsigned short raw, int lineNo) {
  char rawStr[30];
  if (!bDebugDisplay)
    return;
  if (lineNo == 1) {
      displayBlankRectangle(0,0,128,10);
  }
  sprintf(rawStr, "0x%04hx", raw);
  displaySmall(2, lineNo, 40, rawStr);
}

void displayTemperature(short temp, int lineNo) {
  char tempStr[30];
  if (!bDebugDisplay)
    return;
  sprintf(tempStr, "%3hd C", temp);
  displaySmall(100, lineNo, 40, tempStr);
}

void displayOutput(short value, int lineNo) {
  char valueStr[30];
  if (!bDebugDisplay)
    return;
  oled.setFont(ArialMT_Plain_10);
  oled.setTextAlignment(TEXT_ALIGN_LEFT);
  
  oled.setColor(BLACK);
  oled.fillRect(40, 20 + 10 * lineNo, 80, 9);
  oled.setColor(WHITE);
  oled.drawProgressBar(40, 22 + 10 * lineNo, 80, 8, value * 100 / 255);
  sprintf(valueStr, "%3hd", value);
  displaySmall(1, lineNo + 2, 39, valueStr);
}

void displaySmall(unsigned short x, unsigned short lineNo, unsigned short xSpan, char * text) {
  oled.setFont(ArialMT_Plain_10);
  oled.setTextAlignment(TEXT_ALIGN_LEFT);

  oled.setColor(BLACK);
  oled.fillRect(x, 10 * lineNo, xSpan, 10);
  oled.setColor(WHITE);
  oled.drawString(x, 10 * lineNo, text);
  oled.display();
}

void standardDisplay(unsigned int temp1, int temp2, unsigned int setPoint1, int setPoint2, bool bValidate, double timeout) {
  char valueStr[20];  
  if (bDebugDisplay)
    return;
  unsigned int temp = (temp1 + temp2)/2;
  unsigned int setPoint = (setPoint1 + setPoint2)/2;
  if (bValidate) {
      sprintf(valueStr, "%3 d/%3d", temp,setPoint);
  } else {
      sprintf(valueStr, "%3 d", temp);
  }
  displayBig(0,0,128,valueStr);
  if (timeout < 0.0) {
      timeout = 0.0;
  }
  int progress = sqrt(timeout/60.0)*5;
  if (progress != previousProgress) {
    displayBlankRectangle(0,44,128,20);
    oled.drawProgressBar(5, 55, 110, 8, progress);
    previousProgress = progress;
  }
}

void standardDisplayText2(const char * message) {
  if (bDebugDisplay)
    return;
  displayBig(0,1,128,message);
}

void displayFullErase() {
  displayBlankRectangle(0,0,128,64);
}
void displayBlankRectangle(unsigned short x, unsigned short y, unsigned short xSpan, unsigned short ySpan) {
  oled.setColor(BLACK);
  oled.fillRect(x, y, xSpan, ySpan);
  oled.setColor(WHITE);
  oled.display();
}


void displayBig(unsigned short x, unsigned short lineNo, unsigned short xSpan, const char * text) {
  oled.setFont(ArialMT_Plain_24);
  oled.setTextAlignment(TEXT_ALIGN_LEFT);

  oled.setColor(BLACK);
  oled.fillRect(x, 24 * lineNo, xSpan, 24);
  oled.setColor(WHITE);
  oled.drawString(x, 24 * lineNo, text);
  oled.display();
}

unsigned short rawToVoltagemV(unsigned short rawValue) {
  uint32_t voltage;
  voltage = esp_adc_cal_raw_to_voltage(rawValue, &adc1_chars);
  return voltage;
}

short voltageToTemperature(unsigned short voltagemV) {
  float V = voltagemV / 1000.0f;
  float temp;

  //temp= 1.0f/(1/Tc1 - log(R1/((Vdd-V)*R0/V))/beta); // resistor on lower leg
  temp = 1.0f / (1 / Tc1 - log(R1 / (R0 * V / (Vdd - V))) / beta); // resistor on upper leg
  temp = temp - Tz;
  return temp;
}


void oledSetup(void) {
  // reset OLED
  pinMode(16, OUTPUT);
  digitalWrite(16, LOW);
  delay(50);
  digitalWrite(16, HIGH);

  oled.init();
  oled.clear();
  oled.flipScreenVertically();
  oled.setFont(ArialMT_Plain_10);
  oled.setTextAlignment(TEXT_ALIGN_LEFT);
  oled.drawString(0 , 0, "START" );
  oled.drawString(0 , 50, &g_ironParameters.name[0]);
  oled.display();
}



float temperatureInternal() {
  SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_SAR, 3, SENS_FORCE_XPD_SAR_S);
  SET_PERI_REG_BITS(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_CLK_DIV, 10, SENS_TSENS_CLK_DIV_S);
  CLEAR_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_POWER_UP);
  CLEAR_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_DUMP_OUT);
  SET_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_POWER_UP_FORCE);
  SET_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_POWER_UP);
  ets_delay_us(100);
  SET_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_DUMP_OUT);
  ets_delay_us(5);
  float temp_f = (float)GET_PERI_REG_BITS2(SENS_SAR_SLAVE_ADDR3_REG, SENS_TSENS_OUT, SENS_TSENS_OUT_S);
  float temp_c = (temp_f - 32) / 1.8;
  return temp_c;
}
