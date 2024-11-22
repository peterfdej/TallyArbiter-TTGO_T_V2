/*
#########################################################################################
TTGO-T-display 135x240 1.14"
Board: ESP32 Dev Module
Partition Sceme: Huge AP

Modify User_Setup_Select.h in libraryY TFT_eSPI
  //#include <User_Setup.h>
  #include <User_Setups/Setup25_TTGO_T_Display.h>

include LANC interface.
zoom and record control with PTZOptics APP 
(record on/off = Preset 7 and 8) 
In messaging TallyArbiter #recon or #recoff for all listenerdevices
For specific listenerdevice:
#xxz-t, xxz+t zoom command (xx = firt 2 characters devicername, t = zoomtime 2 ,3 ,4, 5, 6, 7, 8 or 9)
t = 1, start zoom, t = 0, stop zoom
#xxr+ record command  
#########################################################################################
*/
#include <Arduino.h>
#include <AsyncTCP.h>  //TCP server
#include <SocketIOclient.h>
#include <WiFi.h>
#include <TFT_eSPI.h>
#include <Preferences.h>
#include <WiFiManager.h>
#include <PinButton.h>
#include <Arduino_JSON.h>
#include "esp_adc_cal.h"

#define TCP_SERVER_PORT 5678 //PTZOptics-VISCA
#define ADC_EN  14  //ADC_EN is the ADC detection enable port
#define ADC_PIN 34
#define TFT_DISPOFF 0x28
#define TFT_SLPIN   0x10
#define TFT_BL      4        // Display backlight control pin

#define BUTTON_PIN_BITMASK 0x800000001  //gpio 0 + gpio 35 for awake from deep sleep.
                                        //Bottom button long press is deep sleep mode.

#define LANC_INTERFACE true

float battery_voltage;
String voltage;

int vref = 1100;
int batteryLevel = 100;
int barLevel = 0;
int LevelColor = TFT_WHITE;
bool backlight = true;
PinButton btntop(35); //top button, switch screen.
PinButton btnbottom(0); //bottom button, screen on/off
Preferences preferences;

long previousMillis = 0;
long interval = 5000; //interval check LANC connection and recording state

//Tally Arbiter Server
char tallyarbiter_host[40]; //IP address of the Tally Arbiter Server
char tallyarbiter_port[6] = "4455";

TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setups/Setup25_TTGO_T_Display.h

#if LANC_INTERFACE
const int led_program = 17; //connected with 270ohm resistor
const int led_preview = 2;  //connected with 270ohm resistor
const int led_blue = 15;    //connected with 270ohm resistor
#else
const int led_program = 25; //connected with 270ohm resistor
const int led_preview = 27;  //connected with 270ohm resistor
const int led_blue = 26;    //connected with 270ohm resistor
#endif

WiFiManager wm; // global wm instance
WiFiManagerParameter custom_field; // global param ( for non blocking w params )
bool portalRunning = false;

#if LANC_INTERFACE
//LANC varables
int PinLANC = 13;                // 5V limited input signal from LANC data
int PinCMD = 12;                 // Command to send to LANC
byte typeCam = B00101000;
byte typeVTR = B00011000;
int bitDuration = 104;
byte byte_0;
byte byte_1;
byte byte_2;
byte byte_3;
byte byte_4;
byte byte_5;
byte byte_6;
byte byte_7;
byte statusbyte_1;
byte statusbyte_2;

int zoomt = 0;
int zoomw = 0;
bool recordon = 0;
bool recordoff = 0;
bool recording = 0;
bool lancconnect = 0;
int zoomtime;
TaskHandle_t zoomhandle;
#endif

//Tally Arbiter variables
bool CUT_BUS = true; // true = Programm + Preview = Red Tally; false = Programm + Preview = Yellow Tally screen
bool LAST_MSG = true; // true = show messages on tally screen
SocketIOclient socket;
JSONVar BusOptions;
JSONVar Devices;
JSONVar DeviceStates;
String DeviceId = "unassigned";
String DeviceName = "Unassigned";
String LastMessage = "";
String listenerDeviceName = "TTGO_T-1";
String boxnaam = "";

String prevType = ""; // reduce display flicker by storing previous state
String actualType = "";
String actualColor = "";
int actualPriority = 0;
bool mode_preview = false;  
bool mode_program = false;
bool tallyconnected = 0;

//General Variables
bool networkConnected = false;
int currentScreen = 0;        //0 = Tally Screen, 1 = Settings Screen

void setup(void) {
  Serial.begin(115200);
  while (!Serial);
/*
  ADC_EN is the ADC detection enable port
  If the USB port is used for power supply, it is turned on by default.
  If it is powered by battery, it needs to be set to high level
  */
  pinMode(ADC_EN, OUTPUT);
  digitalWrite(ADC_EN, HIGH);
  //check if top button is pressed at boot for wifimanager setup mode
  if (digitalRead(35) == 0) {
    wm.resetSettings();
  }

  #if LANC_INTERFACE
  pinMode(PinLANC, INPUT_PULLUP); //listens to the LANC line
  pinMode(PinCMD, OUTPUT); //writes to the LANC line
  digitalWrite(PinCMD, LOW); //set LANC line to +5V
  #else
  setCpuFrequencyMhz(80);    //Save battery by turning down the CPU clock
  #endif

  pinMode(led_blue, OUTPUT);
  digitalWrite(led_blue, HIGH);
  Serial.println("Blue LED ON.");
  pinMode(led_program, OUTPUT);
  pinMode(led_preview, OUTPUT);

  logger("Initializing TTGO.", "info-quiet");
 
  btStop();                  //Save battery by turning off BlueTooth
  
  #if LANC_INTERFACE
  xTaskCreatePinnedToCore(  //Task on core 0 to make zoom smooth.
    zoom,
    "Zoom Tele or Wide",
    800,
    NULL,
    1,
    &zoomhandle,
    0
  );
  #endif

  //uint64_t chipid = ESP.getEfuseMac();
  //listenerDeviceName = "TTGO_T-" + String((uint16_t)(chipid>>32)) + String((uint32_t)chipid);
  uint8_t baseMac[6];
  char macchar[6];
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  sprintf(macchar, "%02X%02X%02X", baseMac[3], baseMac[4], baseMac[5]);
  listenerDeviceName = "T-" + String(macchar);


  tft.init();
  tft.setRotation(1);
  cleartftscreen(TFT_BLACK, 2);
  tft.setTextColor(TFT_WHITE);
  tft.setSwapBytes(true);

  logger("Tally Arbiter TTGO Listener Client booting.", "info");
  logger("Listener device name: " + listenerDeviceName, "info");
  preferences.begin("tally-arbiter", false);

  // added to clear out corrupt prefs
  //preferences.clear();
  logger("Reading preferences", "info-quiet");
  if(preferences.getString("deviceid").length() > 0){
    DeviceId = preferences.getString("deviceid");
  }
  if(preferences.getString("devicename").length() > 0){
    DeviceName = preferences.getString("devicename");
  }
  if(preferences.getString("taHost").length() > 0){
    String newHost = preferences.getString("taHost");
    logger("Setting TallyArbiter host as" + newHost, "info-quiet");
    newHost.toCharArray(tallyarbiter_host, 40);
  }
  if(preferences.getString("taPort").length() > 0){
    String newPort = preferences.getString("taPort");
    logger("Setting TallyArbiter port as" + newPort, "info-quiet");
    newPort.toCharArray(tallyarbiter_port, 6);
  }
  preferences.end();

  delay(1000); //wait 100ms before moving on
  cleartftscreen(TFT_BLACK, 2);
  connectToNetwork(); //starts Wifi connection
  //while (!networkConnected) {
  //  delay(200);
  //}

  #if LANC_INTERFACE
  //connectToServer(); // connect TallyArbiter server
  AsyncServer *server = new AsyncServer(TCP_SERVER_PORT); // start listening on tcp port 5678 for PTZ commands
	server->onClient(&handleNewClient, server);
	server->begin();
  #endif
}

void loop() {
  if (!networkConnected && !portalRunning)  {
    cleartftscreen(TFT_BLACK, 2);
    digitalWrite(led_blue, LOW);
    logger("Connection failed","info");
    logger("Longpress topbutton", "info");
    logger("for config WiFi", "info");
    logger("or reset device.","info");
    delay(1000);
    digitalWrite(led_blue, HIGH);
    //connectToNetwork();
  }
  if(portalRunning){
    wm.process();
  }
  socket.loop();
  #if LANC_INTERFACE
  unsigned long currentMillis = millis();
  if (tallyconnected){
    if(currentMillis - previousMillis > (interval)) {
      previousMillis = currentMillis;
      lanccheck();
      recordcheck();
    }
  }
  #endif
  btntop.update();
  btnbottom.update();
  showVoltage();
  #if LANC_INTERFACE
  if (recordon && lancconnect) {
    recordstart();
    recordon = 0;
  }
  if (recordoff && lancconnect) {
    recordstop();
    recordoff = 0;
  }
  #endif
  if (btntop.isSingleClick()) {
    switch (currentScreen) {
      case 0:
        showSettings();
        currentScreen = 1;
        break;
      case 1:
        showDeviceInfo();
        currentScreen = 0;
        break;
    }
  }
  if (btntop.isLongClick()) { //wipe WiFi credentials
    wm.resetSettings();
    connectToNetwork();
    //ESP.restart();
  }
  if (btnbottom.isSingleClick()) {
    if (backlight == true) {
      digitalWrite(TFT_BL, LOW);
      backlight = false;
    } else {
      digitalWrite(TFT_BL, HIGH);
      backlight = true;
    }
  }
  if (btnbottom.isLongClick()){ // MutiButton.h static const int LONGCLICK_DELAY   = 1000; // ms
    intosleepmode(); 
  }
  //Serial.println(uxTaskGetStackHighWaterMark(zoomhandle));
  delay(100);
}

#if LANC_INTERFACE
// xTask
void zoom(void * parameter) {
  for(;;){
    if (lancconnect){ 
      while (zoomt > 0) {
        ZoomTele(zoomt);
        delay(20); //a lanc telegram = 16.6 - 20ms
      }
      while (zoomw > 0){
        ZoomWide(zoomw);
        delay(20);
      }
    }
    delay(100);
  }
}
#endif

void cleartftscreen(int background, int textsize) {
  tft.setCursor(0, 0);
  tft.fillScreen(background);
  tft.setTextSize(textsize);
}

void intosleepmode() {
  tft.println("Going into sleepmode");
  delay(3000);
  tft.writecommand(TFT_DISPOFF);
  tft.writecommand(TFT_SLPIN);
  //After using light sleep, you need to disable timer wake, because here use external IO port to wake up
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
  // esp_sleep_enable_ext1_wakeup(GPIO_SEL_35, ESP_EXT1_WAKEUP_ALL_LOW);
  esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK, ESP_EXT1_WAKEUP_ALL_LOW); 
  //esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);
  delay(200);
  esp_deep_sleep_start();
}
//**********************************************
// WiFi Manager
//
void connectToNetwork() {
  WiFi.mode(WIFI_STA); 
  logger("Connecting to SSID: " + (String)wm.getWiFiSSID(), "info");
  //reset settings - wipe credentials for testing
  //wm.resetSettings();
  WiFiManagerParameter custom_taServer("taHostIP", "Tally Arbiter Server", tallyarbiter_host, 40);
  WiFiManagerParameter custom_taPort("taHostPort", "Port", tallyarbiter_port, 6);
  wm.addParameter(&custom_taServer);
  wm.addParameter(&custom_taPort);
  wm.setSaveParamsCallback(saveParamCallback);

  // If no saved WiFi we assume that configuration is needed via the captive portal
  if (wm.getWiFiIsSaved()) {
    delay(100);
    wm.setEnableConfigPortal(false);
    portalRunning = false;
  } else {
    cleartftscreen(TFT_BLACK, 2);
    logger("Setup mode", "info");
    logger("Connect to:", "info");
    logger(String(listenerDeviceName), "info");
    logger("http://192.168.4.1", "info");
    //wm.setConfigPortalTimeout(120);
    wm.setEnableConfigPortal(true);
    portalRunning = true;
  }

  std::vector<const char *> menu = {"wifi","param","info","sep","restart","exit"};
  wm.setMenu(menu);
  wm.setClass("invert");
  //wm.setConfigPortalTimeout(120);
  wm.setConnectTimeout(10);
  wm.setConnectRetries(2);

  bool res;
  res = wm.autoConnect(listenerDeviceName.c_str()); // AP name for setup
  if (!res) {
    logger("Failed to connect", "info");
    // ESP.restart();
    delay(2000);
  } else {
    logger("connected)", "info");
    networkConnected = true;
    connectToServer(); // connect to TallyArbiter server
  }
}

String getParam(String name) {
  //read parameter from server, for customhmtl input
  String value;
  if (wm.server->hasArg(name)) {
    value = wm.server->arg(name);
  }
  return value;
}

void saveParamCallback() {
  logger("[CALLBACK] saveParamCallback fired", "info-quiet");
  logger("PARAM tally Arbiter Server = " + getParam("taHostIP"), "info-quiet");
  String str_taHost = getParam("taHostIP");
  String str_taPort = getParam("taHostPort");

  logger("Saving new TallyArbiter host", "info-quiet");
  logger(str_taHost, "info-quiet");
  preferences.begin("tally-arbiter", false);
  preferences.putString("taHost", str_taHost);
  preferences.putString("taPort", str_taPort);
  preferences.end();
}

void showVoltage() {
    static uint64_t timeStamp = 0;
    if (millis() - timeStamp > 5000) {
        timeStamp = millis();
        uint16_t v = analogRead(ADC_PIN);
        battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
        voltage = "Voltage :" + String(battery_voltage) + "V";
        batteryLevel = floor(100.0 * (((battery_voltage * 1.1) - 3.0) / (4.07 - 3.0))); //100%=3.7V, Vmin = 2.8V
        batteryLevel = batteryLevel > 100 ? 100 : batteryLevel;
        barLevel = 133 - (batteryLevel * 133/100);
        if (battery_voltage >= 3){
          LevelColor = TFT_WHITE;
        }
        else {
          LevelColor = TFT_RED;
        }
        if (currentScreen == 0){
          tft.fillRect(232, 0, 8, 135, LevelColor);
          tft.fillRect(233, 1, 6, barLevel, TFT_BLACK);
        }
        if (battery_voltage < 2.8){ //go into sleep,awake with top button
          tft.setRotation(1);
          cleartftscreen(TFT_BLACK, 2);
          tft.setTextColor(TFT_WHITE);
          tft.println("Battery level low");
          tft.println(v); //for debug
          tft.println(battery_voltage);
          delay(5000);
          intosleepmode();
        }
    }
}

#if LANC_INTERFACE
//****************************************
// LANC part
void lanccheck() {
  while (pulseIn(PinLANC, HIGH) < 5000) {
    if (pulseIn(PinLANC, HIGH) == 0) {  //timeout 1 sec, nothing connected
      lancconnect = 0;
      return;
    }
    else {
      lancconnect = 1;
      return;
    }
  }
}

void recordcheck() {
  if (lancconnect){
    if (statuscode() == 0xFB && recording == 0) { //local recording on
      recording = 1;
      SendMessage("Recording on");
    }
    else if (statuscode() == 0xEB && recording == 1) { //local recording off
      recording = 0;
      SendMessage("Recording off");
    }
  }
}

void printbytes(){ //test function
  //Serial.println("******");
  Serial.print(byte_0, HEX);Serial.print(" ");
  Serial.print(byte_1, HEX);Serial.print(" ");
  Serial.print(byte_2, HEX);Serial.print(" ");
  Serial.print(byte_3, HEX);Serial.print(" ");
  Serial.print(byte_4, HEX);Serial.print(" ");
  Serial.print(byte_5, HEX);Serial.print(" ");
  Serial.print(byte_6, HEX);Serial.print(" ");
  Serial.print(byte_7, HEX);Serial.print(" ");
  Serial.println("");
  if (byte_4 == 0xEB){
    Serial.println("Record OFF");
  }
  else if (byte_4 == 0xFB){
    Serial.println("Record ON");
  }
  else {
    Serial.print("Unknown");
  }
}

byte statuscode(){
  readbytes();
  statusbyte_1 = byte_4;
  readbytes();
  if (statusbyte_1 == byte_4){ //value valid
    return statusbyte_1;
  }
  else {
    statusbyte_2 = byte_4;
    readbytes();
    if (statusbyte_1 == byte_4){
      return statusbyte_1;
    }
    else {
      if (statusbyte_2 == byte_4){
        return statusbyte_2;
      }
      else {
        return 0x00;
      }
    } //the odds are nill to get 3 times different status bytes
  }
}

void recordstart() {
  if (statuscode() == 0xEB){ //status is non recording
    sendCMD(typeVTR, B00110011); // start/stop rec. 0x33
    Serial.print("Start recording");
    tft.fillRect(0, 30, 232, 23, TFT_BLACK); // x y width hight
    tft.setCursor(0, 30);
    tft.println("Recording on");
    SendMessage("Recording on");
    recording = 1;
    delay(1000);
    if (statuscode() == 0xEB) {
      sendCMD(typeVTR, B00110011); //send command a second time
    }
  }
  else if (statuscode() == 0xFB){
    Serial.print("Camera already recording");
    SendMessage("Camera already recording");
  }
  else {
    Serial.print("Error in status");
  }
}

void recordstop() {
  if (statuscode() == 0xFB){ //status is recording
    sendCMD(typeVTR, B00110011); // start/stop rec. 0x33
    Serial.print("Stop recording");
    tft.fillRect(0, 30, 232, 23, TFT_BLACK); // x y width hight
    tft.setCursor(0, 30);
    tft.println("Recording off");
    SendMessage("Recording off");
    recording = 0;
    delay(1000);
    if (statuscode() == 0xFB){
      sendCMD(typeVTR, B00110011); //send command a second time
    }
  }
  else if (statuscode() == 0xEB){
    Serial.print("Camera was not recording");
    SendMessage("Camera was not recording");
  }
  else {
    Serial.print("Error is status");
  }
}

void ZoomWide(int Wide) {
  switch(Wide) {
    case 1: //speed 1
    sendCMD(typeCam, B00010000);
    break;
    case 2: //speed 2
    sendCMD(typeCam, B00010010);
    break;
    case 3: //speed 3
    sendCMD(typeCam, B00010100);
    break;
    case 4: //speed 4
    sendCMD(typeCam, B00010110);
    break;
    case 5: //speed 5
    sendCMD(typeCam, B00011000);
    break;
    case 6: //speed 6
    sendCMD(typeCam, B00011010);
    break;
    case 7: //speed 7
    sendCMD(typeCam, B00011100);
    break;
    case 8: //speed 8
    sendCMD(typeCam, B00011110);
    break;
  }
}

void ZoomTele(int Tele) {
  switch(Tele) {
    case 1: //speed 1
    sendCMD(typeCam, B00000000);
    break;
    case 2: //speed 2
    sendCMD(typeCam, B00000010);
    break;
    case 3: //speed 3
    sendCMD(typeCam, B00000100);
    break;
    case 4: //speed 4
    sendCMD(typeCam, B00000110);
    break;
    case 5: //speed 5
    sendCMD(typeCam, B00001000);
    break;
    case 6: //speed 6
    sendCMD(typeCam, B00001010);
    break;
    case 7: //speed 7
    sendCMD(typeCam, B00001100);
    break;
    case 8: //speed 8
    sendCMD(typeCam, B00001110);
    break;
  }
}

void sendCMD(unsigned char cmd1, unsigned char cmd2) {
  for (int     cmdRepeatCount = 0; cmdRepeatCount < 4; cmdRepeatCount++) {
    while (pulseIn(PinLANC, HIGH) < 5000) {
      //if (pulseIn(PinLANC, HIGH) == 0) {  //timeout 1 sec, nothing connected
      //  return;
      //}
    }
    delayMicroseconds(bitDuration); //startbit for byte 0
    for( int i=0; i<8; i++) {
      digitalWrite(PinCMD, (cmd1 & (1<<i) ) ? HIGH : LOW);  //Write bit 0.
      delayMicroseconds(bitDuration - 8); //write takes 8 microseconbds
    }
    digitalWrite(PinCMD, LOW);
    delayMicroseconds(10);
    while (digitalRead(PinLANC)) {
      //Loop as long as the LANC line is +5V during the stop bit
    }
    delayMicroseconds(bitDuration); //startbit for byte 1
    for( int i=0; i<8; i++) {
      digitalWrite(PinCMD, (cmd2 & (1<<i) ) ? HIGH : LOW);  //Write bit 1
      delayMicroseconds(bitDuration - 8);
    }
    digitalWrite(PinCMD, LOW);
  }
}


void readbytes() {
  for (int     cmdRepeatCount = 0; cmdRepeatCount < 4; cmdRepeatCount++) {
    while (pulseIn(PinLANC, HIGH) < 5000) {
    }
    delayMicroseconds(bitDuration + (bitDuration/2)); //startbit for byte 0 + reading point at half bit
    byte_0 = 0;
    for( int i=0; i<8; i++) {
      if (digitalRead(PinLANC)){bitSet(byte_0,i);} //read bit
      delayMicroseconds(bitDuration);
    }
    digitalWrite(PinCMD, LOW);
    delayMicroseconds(10);
    while (digitalRead(PinLANC)) {
      //Loop as long as the LANC line is +5V during the stop bit
    }
    delayMicroseconds(bitDuration + (bitDuration/2)); //startbit for byte 1
    byte_1 = 0;
    for( int i=0; i<8; i++) {
      if (digitalRead(PinLANC)){bitSet(byte_1,i);}
      delayMicroseconds(bitDuration);
    }
    digitalWrite(PinCMD, LOW);
    //next byte 2
    delayMicroseconds(10);
    while (digitalRead(PinLANC)) {
      //Loop as long as the LANC line is +5V during the stop bit
    }
    delayMicroseconds(bitDuration + (bitDuration/2)); //startbit for byte 2
    byte_2 = 0;
    for( int i=0; i<8; i++) {
      if (digitalRead(PinLANC)){bitSet(byte_2,i);}
      delayMicroseconds(bitDuration);
    }
    digitalWrite(PinCMD, LOW);
    //next byte 3
    delayMicroseconds(10);
    while (digitalRead(PinLANC)) {
      //Loop as long as the LANC line is +5V during the stop bit
    }
    delayMicroseconds(bitDuration + (bitDuration/2)); //startbit for byte 3
    byte_3 = 0;
    for( int i=0; i<8; i++) {
      if (digitalRead(PinLANC)){bitSet(byte_3,i);}
      delayMicroseconds(bitDuration);
    }
    digitalWrite(PinCMD, LOW);
    //next byte 4
    delayMicroseconds(10);
    while (digitalRead(PinLANC)) {
      //Loop as long as the LANC line is +5V during the stop bit
    }
    delayMicroseconds(bitDuration + (bitDuration/2)); //startbit for byte 4
    byte_4 = 0;
    for( int i=0; i<8; i++) {
      if (digitalRead(PinLANC)){bitSet(byte_4,i);}
      delayMicroseconds(bitDuration);
    }
    digitalWrite(PinCMD, LOW);
    //nextbyte 5
    delayMicroseconds(10);
    while (digitalRead(PinLANC)) {
      //Loop as long as the LANC line is +5V during the stop bit
    }
    delayMicroseconds(bitDuration + (bitDuration/2)); //startbit for byte 5
    byte_5 = 0;
    for( int i=0; i<8; i++) {
      if (digitalRead(PinLANC)){bitSet(byte_5,i);}
      delayMicroseconds(bitDuration);
    }
    digitalWrite(PinCMD, LOW);
    //nextbyte 6
    delayMicroseconds(10);
    while (digitalRead(PinLANC)) {
      //Loop as long as the LANC line is +5V during the stop bit
    }
    delayMicroseconds(bitDuration + (bitDuration/2)); //startbit for byte 6
    byte_6 = 0;
    for( int i=0; i<8; i++) {
      if (digitalRead(PinLANC)){bitSet(byte_6,i);}
      delayMicroseconds(bitDuration);
    }
    digitalWrite(PinCMD, LOW);
    //nextbyte 7
    delayMicroseconds(10);
    while (digitalRead(PinLANC)) {
      //Loop as long as the LANC line is +5V during the stop bit
    }
    delayMicroseconds(bitDuration + (bitDuration/2)); //startbit for byte 7
    byte_7 = 0;
    for( int i=0; i<8; i++) {
      if (digitalRead(PinLANC)){bitSet(byte_7,i);}
      delayMicroseconds(bitDuration);
    }
    digitalWrite(PinCMD, LOW);
  }
}

//****************************************
// TCP server part, listening to PTZOptics App
void handle_visca(uint8_t *buf, size_t len) {
  for (int i = 0; (i < len && i < 16); i++) {
    Serial.print(" ");
    Serial.print(buf[i], HEX);
  }
  Serial.println("");
  if (buf[1] == 0x01 && buf[2] == 0x06) {
    if (buf[3] == 0x04) {
      Serial.println("PTZ HOME DETECTED..."); //81 01 06 04 FF
    }
    else if (buf[6] == 0x03 && buf[7] == 0x03) {
      Serial.println("PTZ STOP");  //81 01 06 01 00 00 03 03 FF
    }
    else {
      Serial.println("PTZ CONTROL DETECTED..."); //81 01 06 01  2 messages , release is stop 81 01 06 01 00 00 03 03 FF
      //buf[4] = pan speed
      //buf[5] = tilt speed
      //buf[6] buf[7] = 0301 up, 0302 down, 0103 left, 0203 right, 0101 upleft, 0201 upright, 0102 downleft, 0202 downright, 0303 stop.
    }
  }
  if (buf[1] == 0x01 && buf[2] == 0x04 && buf[3] == 0x07) { //81 01 04 07 xx FF  00=stop, 02=tele, 03=wide, 2p=tele var, 3p=wide var; p=0-7
                                                          //2 messages , release is stop zoom  81 01 04 07 00 FF
    if (buf[4] == 0x00) {
      Serial.println("ZOOM STOP");
      zoomt = 0;
      zoomw = 0;
      Serial.println(zoomt);
      Serial.println(zoomw);
    }
    else {
      Serial.println("ZOOM CONTROL DETECTED");
      if ((buf[4] >> 4) == 0x02) {
        zoomt = (buf[4] & 0xDF);
      }
      if (buf[4] == 0x02) { //Bitfocus Companion command (no speed)
        zoomt = (5);
      }
      if ((buf[4] >> 4) == 0x03) {
        zoomw = (buf[4] & 0xCF);
      }
      if (buf[4] == 0x03) {
        zoomw = (5);
      }
    }
  }
  if (buf[1] == 0x01 && buf[2] == 0x04 && buf[3] == 0x3F && buf[4] == 0x02) { //81 01 04 3F 02 pp FF  pp=0-127
    if (buf[5] == 0x07)
    {
      //Serial.println("Start recording");
      recordon = 1;
    }
    else if (buf[5] == 0x08) {
      //Serial.println("Stop recording");
      recordoff = 1;
    }
    else {
      Serial.println("PTZ PRESET DETECTED");
    }
  }
}

static void handleData(void *arg, AsyncClient *client, void *data, size_t len)  {
	Serial.printf("\n data received from client %s \n", client->remoteIP().toString().c_str());
	//Serial.write((uint8_t *)data, len);
  handle_visca((uint8_t *)data, len);
}

static void handleError(void *arg, AsyncClient *client, int8_t error) {
	Serial.printf("\n connection error %s from client %s \n", client->errorToString(error), client->remoteIP().toString().c_str());
}

static void handleDisconnect(void *arg, AsyncClient *client)  {
	Serial.printf("\n client %s disconnected \n", client->remoteIP().toString().c_str());
  tft.fillRect(0, 30, 232, 23, TFT_BLACK); // x y width hight 
  tft.setCursor(0, 30);
  tft.println("Remote disconnected");
  SendMessage("Remote disconnected");
}

static void handleTimeOut(void *arg, AsyncClient *client, uint32_t time)  {
	Serial.printf("\n client ACK timeout ip: %s \n", client->remoteIP().toString().c_str());
}

static void handleNewClient(void *arg, AsyncClient *client) {
	Serial.printf("\n new client has been connected to server, ip: %s", client->remoteIP().toString().c_str());
  tft.fillRect(0, 30, 232, 23, TFT_BLACK); // x y width hight
  tft.setCursor(0, 30);
  tft.println("Remote connected");
  SendMessage("Remote connected");
	// register events
	client->onData(&handleData, NULL);
	client->onError(&handleError, NULL);
	client->onDisconnect(&handleDisconnect, NULL);
	client->onTimeout(&handleTimeOut, NULL);
} 
#endif

// *******************
// TallyArbiter
//

void showSettings() {
  wm.startWebPortal();
  portalRunning = true;
  //displays the current network connection and Tally Arbiter server data
  tft.setCursor(0, 0);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.println("SSID: " + String(WiFi.SSID()));
  tft.println(WiFi.localIP());
  tft.println();
  tft.println("TallyArbiter Server:");
  tft.println(String(tallyarbiter_host) + ":" + String(tallyarbiter_port));
  tft.println();
  Serial.println(voltage);
  if(battery_voltage >= 4.2){
    tft.println("Battery charging...");   // show when TTGO is plugged in
  } else if (battery_voltage < 3) {
    tft.println("Battery empty. Recharge!!");
  } else {
    tft.println("Battery:" + String(batteryLevel) + "%");
  }
}

void showDeviceInfo() {
  if(portalRunning) {
    wm.stopWebPortal();
    portalRunning = false;
  }
  cleartftscreen(TFT_BLACK, 2);
  evaluateMode();
}

void logger(String strLog, String strType) {
  if (strType == "info") {
    Serial.println(strLog);
    int x = strLog.length();
    for (int i=0; i < x; i=i+19) {
      tft.println(strLog.substring(0,19));
      strLog = strLog.substring(19);
    }
  }
  else {
    Serial.println(strLog);
  }
}

void ws_emit(String event, const char *payload = NULL) {
  if (payload) {
    String msg = "[\"" + event + "\"," + payload + "]";
    socket.sendEVENT(msg);
  } else {
    String msg = "[\"" + event + "\"]";
    socket.sendEVENT(msg);
  }
}

void connectToServer() {
  logger("Connecting to Tally Arbiter host: " + String(tallyarbiter_host) + " " + tallyarbiter_port, "info");
  socket.onEvent(socket_event);
  socket.begin(tallyarbiter_host, atol(tallyarbiter_port));
}

void socket_event(socketIOmessageType_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case sIOtype_CONNECT:
      socket_Connected((char*)payload, length);
      break;
    case sIOtype_DISCONNECT:
      socket_Disconnected();
    case sIOtype_ACK:
    case sIOtype_ERROR:
    case sIOtype_BINARY_EVENT:
    case sIOtype_BINARY_ACK:
      // Not handled
      break;

    case sIOtype_EVENT:
      String msg = (char*)payload;
      String type = msg.substring(2, msg.indexOf("\"",2));
      String content = msg.substring(type.length() + 4);
      content.remove(content.length() - 1);
      logger("Got event '" + type + "', data: " + content, "info-quiet");

      if (type == "bus_options") BusOptions = JSON.parse(content);
      if (type == "reassign") socket_Reassign(content);
      if (type == "flash") socket_Flash();
      if (type == "messaging") socket_Messaging(content);
      if (type == "deviceId") {
        DeviceId = content.substring(1, content.length()-1);
        SetDeviceName();
        showDeviceInfo();
        currentScreen = 0;
      }
      if (type == "devices") {
        Devices = JSON.parse(content);
        SetDeviceName();
      }
      if (type == "device_states") {
        DeviceStates = JSON.parse(content);
        processTallyData();
      }
      break;
  }
}

void socket_Disconnected() {
  digitalWrite(led_blue, LOW);
  cleartftscreen(TFT_BLACK, 2);
  logger("Disconnected from   TallyArbiter!", "info");
  tallyconnected = 0;
  delay(2000);
  digitalWrite(led_blue, HIGH);
}

void socket_Connected(const char * payload, size_t length) {
  logger("Connected to Tally Arbiter server.", "info");
  logger("DeviceId: " + DeviceId, "info-quiet");
  tallyconnected = 1;
  digitalWrite(led_blue, LOW);
  tft.fillScreen(TFT_BLACK);
  String deviceObj = "{\"deviceId\": \"" + DeviceId + "\", \"listenerType\": \"" + listenerDeviceName.c_str() + "\", \"canBeReassigned\": true, \"canBeFlashed\": true, \"supportsChat\": true }";
  char charDeviceObj[1024];
  strcpy(charDeviceObj, deviceObj.c_str());
  ws_emit("listenerclient_connect", charDeviceObj);
}

void socket_Flash() {
  //flash the screen white 3 times
  tft.fillScreen(TFT_WHITE);
  digitalWrite(led_blue, HIGH);
  delay(500);
  tft.fillScreen(TFT_BLACK);
  digitalWrite(led_blue, LOW);
  delay(500);
  tft.fillScreen(TFT_WHITE);
  digitalWrite(led_blue, HIGH);
  delay(500);
  tft.fillScreen(TFT_BLACK);
  digitalWrite(led_blue, LOW);
  delay(500);
  tft.fillScreen(TFT_WHITE);
  digitalWrite(led_blue, HIGH);
  delay(500);
  tft.fillScreen(TFT_BLACK);
  digitalWrite(led_blue, LOW);
  
  //then resume normal operation
  switch (currentScreen) {
    case 0:
      showDeviceInfo();
      break;
    case 1:
      showSettings();
      break;
  }
}

String strip_quot(String str) {
  if (str[0] == '"') {
    str.remove(0, 1);
  }
  if (str.endsWith("\"")) {
    str.remove(str.length()-1, 1);
  }
  return str;
}

void socket_Reassign(String payload) {
  String oldDeviceId = payload.substring(0, payload.indexOf(','));
  String newDeviceId = payload.substring(oldDeviceId.length()+1);
  newDeviceId = newDeviceId.substring(0, newDeviceId.indexOf(','));
  oldDeviceId = strip_quot(oldDeviceId);
  newDeviceId = strip_quot(newDeviceId);
  
  String reassignObj = "{\"oldDeviceId\": \"" + oldDeviceId + "\", \"newDeviceId\": \"" + newDeviceId + "\"}";
  char charReassignObj[1024];
  strcpy(charReassignObj, reassignObj.c_str());
  //socket.emit("listener_reassign_object", charReassignObj);
  ws_emit("listener_reassign_object", charReassignObj);
  ws_emit("devices");
  tft.fillScreen(TFT_WHITE);
  digitalWrite(led_blue, HIGH);
  delay(200);
  tft.fillScreen(TFT_BLACK);
  digitalWrite(led_blue, LOW);
  delay(200);
  tft.fillScreen(TFT_WHITE);
  digitalWrite(led_blue, HIGH);
  delay(200);
  tft.fillScreen(TFT_BLACK);
  digitalWrite(led_blue, LOW);
  DeviceId = newDeviceId;
  preferences.begin("tally-arbiter", false);
  preferences.putString("deviceid", newDeviceId);
  preferences.end();
  SetDeviceName();
}

void socket_Messaging(String payload) {
  String strPayload = String(payload);
  int typeQuoteIndex = strPayload.indexOf(',');
  String messageType = strPayload.substring(0, typeQuoteIndex);
  messageType.replace("\"", "");
  //Only messages from producer and clients.
  if (messageType != "server") {
    int messageQuoteIndex = strPayload.lastIndexOf(',');
    String message = strPayload.substring(messageQuoteIndex + 1);
    Serial.println(message);
    message.replace("\"", "");
    if (message.substring(0,1) == "#") {
      #if LANC_INTERFACE
      if (message == "#recon") {
        recordstart();
      } else if (message == "#recoff"){
        recordstop();
      }
      //Serial.println(listenerDeviceName.substring(7,9));
      else if (message.length() <= 6 && message.length() >=5) { // #xxz-7 zoom command  #xxr+ record command
        if (message.substring(1,3) ==  DeviceName.substring(0,2)){ //xx = first 2 characters devicename
          if (message.substring(3,5) == "z-") {
            zoomtime = (message.substring(5, 6)).toInt();
            if (zoomtime == 0){
              zoomw = 0;
            }
            else if (zoomtime == 1){
              zoomw = 5;
            }
            else {
              for (int i = 0; i <= zoomtime; i++) {
                zoomw = 5;
                delay(200);
              }
              zoomw = 0;
            }
            
          }
          else if (message.substring(3,5) == "z+") {
            zoomtime = (message.substring(5, 6)).toInt();
            if (zoomtime == 0){
              zoomt = 0;
            }
            else if (zoomtime == 1){
              zoomt = 5;
            }
            else {
              for (int i = 0; i <= zoomtime; i++) {
                zoomt = 5;
                delay(200);
              }
              zoomt = 0;;
            }
          }
          else if (message.substring(3,5) == "r+") {
            recordstart();
          }
          else if (message.substring(3,5) == "r-") {
            recordstop();
          }
        } 
      }
      #endif
    }
    else {
      LastMessage = messageType + ": " + message;
      evaluateMode();
    }
  }
}
void SendMessage(String message) {
  String deviceObj = "\""+DeviceName+"\",\""+message+"\"";
  char charDeviceObj[1024];
  strcpy(charDeviceObj, deviceObj.c_str());
  ws_emit("messaging", charDeviceObj);
}

void processTallyData() {
  for (int i = 0; i < DeviceStates.length(); i++) {
    if (getBusTypeById(JSON.stringify(DeviceStates[i]["busId"])) == "\"preview\"") {
      if (DeviceStates[i]["sources"].length() > 0) {
        mode_preview = true;
      }
      else {
        mode_preview = false;
      }
    }
    if (getBusTypeById(JSON.stringify(DeviceStates[i]["busId"])) == "\"program\"") {
      if (DeviceStates[i]["sources"].length() > 0) {
        mode_program = true;
      }
      else {
        mode_program = false;
      }
    }
  }
  currentScreen = 0;
  evaluateMode();
}

String getBusTypeById(String busId) {
  for (int i = 0; i < BusOptions.length(); i++) {
    if (JSON.stringify(BusOptions[i]["id"]) == busId) {
      return JSON.stringify(BusOptions[i]["type"]);
    }
  }
  return "invalid";
}

void SetDeviceName() {
  for (int i = 0; i < Devices.length(); i++) {
    if (JSON.stringify(Devices[i]["id"]) == "\"" + DeviceId + "\"") {
      String strDevice = JSON.stringify(Devices[i]["name"]);
      DeviceName = strDevice.substring(1, strDevice.length() - 1);
      break;
    }
  }
  preferences.begin("tally-arbiter", false);
  preferences.putString("devicename", DeviceName);
  preferences.end();
  evaluateMode();
}
void evaluateMode() {
  tft.setCursor(0, 0);
  tft.setTextSize(2);
  
  if (mode_preview && !mode_program) {
    logger("The device is in preview.", "info-quiet");
    digitalWrite(led_program, LOW);
    digitalWrite(led_preview, HIGH);
    tft.setTextColor(TFT_BLACK);
    tft.fillScreen(TFT_GREEN);
  }
  else if (!mode_preview && mode_program) {
    logger("The device is in program.", "info-quiet");
    digitalWrite(led_program, HIGH);
    digitalWrite(led_preview, LOW);
    tft.setTextColor(TFT_BLACK);
    tft.fillScreen(TFT_RED);
  }
  else if (mode_preview && mode_program) {
    logger("The device is in preview+program.", "info-quiet");
    tft.setTextColor(TFT_BLACK);
    if (CUT_BUS == true) {
      tft.fillScreen(TFT_RED);
    }
    else {
      tft.fillScreen(TFT_YELLOW);
    }
    digitalWrite(led_program, HIGH);
    digitalWrite(led_preview, LOW);
  }
  else {
    digitalWrite(led_program, LOW);
    digitalWrite(led_preview, LOW);
    tft.setTextColor(TFT_WHITE);
    tft.fillScreen(TFT_BLACK);
  }
  tft.println(listenerDeviceName);
  tft.println(DeviceName);
  tft.println("-------------------");
  if (LAST_MSG == true) {
    //tft.println(LastMessage);
    logger(LastMessage, "info");
  }
  tft.fillRect(232, 0, 8, 135, LevelColor);
  tft.fillRect(233, 1, 6, barLevel, TFT_BLACK);
}
