#include <OneWire.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <WiFi.h>
#include <EEPROM.h>
#include <Preferences.h>
#include <PubSubClient.h>

/* Logging modes:
 * 0 - no logs
 * 1 - Serial only
 * 2 - OLED only
 * 3 - OLED+Serial
 */
#define LOGGING_MODE 3

//Mqtt names
#define MQTT_CLIENT_NAME "Cool MQTT"
#define MQTT_CLIENT_INTOPIC "intopic"
#define MQTT_CLIENT_OUTTOPIC "outtopic"


//BT Defines
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

//BT Sig
#define BT_SIG_READY "ready"
#define BT_SIG_DONE "done"
#define BT_SIG_RECEIVED "rec"
#define BT_SIG_ERR "err"

//Pins
#define SHUTDOWN_PIN 23   //Pin to send shutdown sig from
#define DS18S20_Pin 17    //Temp sensor
#define SETUP_PIN 19    //if this pin is HIGH on startup, device will enter config mode

//Payload size
#define PAYLOAD_SIZE 4

//Storage defines
#define EEPROM_SSID "ssid"
#define EEPROM_PWD "pwd"

//Oled config
#if LOGGING_MODE >= 2
  #include <U8x8lib.h>
  
  //Logging options
  #define U8LOG_WIDTH 16
  #define U8LOG_HEIGHT 8   
  
  uint8_t u8log_buffer[U8LOG_WIDTH*U8LOG_HEIGHT];
  U8X8LOG u8x8log;
  U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);
#endif

//Wifi Fields
WiFiClient client;
char* ssid;
char* pwd;

//MQTT Fields
byte mac[]    = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED };
IPAddress ip(172, 16, 0, 100);
IPAddress server(172, 16, 0, 2);
PubSubClient client(client);

static uint8_t* data;   //Data to be sent

OneWire ds(DS18S20_Pin);  // Temp sensor
Preferences preferences;    //Prefs object

//Bluetooth config
BLECharacteristic* creds;
BLEServer* server;
BLEService* service;
BLEAdvertising* advert;

void done();

class Collector: public BLECharacteristicCallbacks {
  public:
    char** current;
    
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();

      if(current == 0) {
        current = &ssid;
      } else {
        current = &pwd;
      }

      *current = (char*) malloc(value.length());
      (*current)[value.length()] = 0;

      if (value.length() > 0) {
        for (int i = 0; i < value.length(); i++) {
          (*current)[i] = value[i];
        }

      }

      if(current == &pwd) {
        done();
      }
    }
};  

void setup(void) {
  pinMode(SETUP_PIN, INPUT);

  #if (LOGGING_MODE == 1) || LOGGING_MODE == 3
  Serial.begin(115200);
  #endif

  #if LOGGING_MODE >= 2
  u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  
  u8x8log.begin(u8x8, U8LOG_WIDTH, U8LOG_HEIGHT, u8log_buffer);
  u8x8log.setRedrawMode(0);    // 0: Update screen with newline, 1: Update screen for every char  
  #endif

  log("Starting");
  
  preferences.begin("wifi", false);

  if(digitalRead(SETUP_PIN) == HIGH) {
    log("Config start");
  
    BLEDevice::init("CoolBluetoothMan");
    server = BLEDevice::createServer();
  
    service = server->createService(SERVICE_UUID);
  
    creds = service->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  
    creds->setCallbacks(new Collector());
  
    creds->setValue(BT_SIG_READY);

    
    service->start();
  
    advert = server->getAdvertising();
    advert->start();
  } else {
    log("Regular start");
    data = (uint8_t*) calloc(PAYLOAD_SIZE, sizeof(uint8_t));    //Allocate the required number of bytes for the payload

    ssid = (char*) malloc(33);
    pwd = (char*) malloc(65);
    
    String ssidStr = preferences.getString(EEPROM_SSID);
    memcpy((void*) ssid, (const void*) ssidStr.c_str(), ssidStr.length() + 1);



    String pwdStr = preferences.getString(EEPROM_PWD);
    memcpy((void*) pwd, (const void*) pwdStr.c_str(), pwdStr.length() + 1);

    log(ssid);
    log(pwd);

    /*
     * PUT THE CODE TO SEND DATA TO SERVER HERE!!!
     */
    collectData();
    mqttSend();
    powerDown();

  }
}

boolean wifiConnect(char* ss, char* pw, long timeout) {
  WiFi.begin(ssid, pwd);

  bool connected = true;
  unsigned long start = millis();
  
  while (WiFi.status() != WL_CONNECTED) {
    unsigned long current = millis();

    if(current - start > 10000) {
      connected = false;
      break;
    }
    
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  WiFi.mode(WIFI_OFF);

  return connected;
}

/*
 * Send data via MQTT
 */
void mqttSend() {

  while (!client.connected()) {
    log("Attempting MQTT connection...");
    if (client.connect(MQTT_CLIENT_NAME)) {
      log("connected");
      
      client.publish(MQTT_CLIENT_OUTTOPIC, data);   //Publish data
      client.subscribe(MQTT_CLIENT_INTOPIC);
    } else {
      log("Failed to publish.  retrying");
      
      delay(5000);
    }
  }


void loop(void) {
  
}

void done() {
  log("SSID:");
  log(ssid);
  log("PWD:");
  log(pwd);

  creds->setValue(BT_SIG_RECEIVED);

  log("Connecting");
  advert->stop();
  bool connected = wifiConnect(ssid, pwd, 10000);
  advert->start();
  
  if(connected) {
    log("Connected");
    creds->setValue(BT_SIG_DONE);

    log("Writing");
    preferences.putString(EEPROM_SSID, ssid);
    preferences.putString(EEPROM_PWD, pwd);
    preferences.end();
    log("Done");
  } else {
    log("Failed to connect");
    creds->setValue(BT_SIG_ERR);
    preferences.end();
  }

  powerDown();
}

//Send power down signal
void powerDown() {
  log("Power Down");
  digitalWrite(SHUTDOWN_PIN, LOW);
  pinMode(SHUTDOWN_PIN, OUTPUT);
}

void collectData() {
  int temp = getTemp();

  int2bytes(getTemp(), (void*) (data));
}

/*
   Convert a floating point number to 4 bytes at the output location provided
   Uses memcopy, so don't have the output location be the same as the location of the input float
*/
void int2bytes(int input, void* outputLocation) {
  memcpy(outputLocation, (void*) &input, sizeof(int));
}

void log(char* string) {
  #if (LOGGING_MODE == 1) || (LOGGING_MODE == 3)
  Serial.println(string);
  #endif
  #if LOGGING_MODE >= 2
  u8x8log.println(string);
  #endif
}

//From sensor library example, am not touching
int getTemp() {
  //returns the temperature from one DS18S20 in DEG Celsius
  byte data[12];
  byte addr[8];

  if (!ds.search(addr)) {
    //no more sensors on chain, reset search
    ds.reset_search();
    return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad


  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = (int16_t) ((MSB << 8) | LSB);
  float TemperatureSum = tempRead / 16;
  
  return (int) (TemperatureSum * 1000);

}
