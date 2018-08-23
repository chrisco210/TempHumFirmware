#include <OneWire.h>
#define SHUTDOWN_PIN 23
#define DS18S20_Pin 17
#include <U8g2lib.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>


#define BATPIN A7     //Battery level pin
#define ACTIVEPIN 11   //Activity LED pin
#define DHTPIN 10    // what pin we're connected to

//# of bytes in the payload
#define PAYLOAD_SIZE 4


// This EUI must be in little-endian format, so least-signif  icant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]= { 0x94, 0x07, 0x01, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]= { 0x60, 0x30, 0x1A, 0x83, 0x96, 0xFD, 0x29, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from the TTN console can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0xD9, 0x8F, 0x39, 0x93, 0xDA, 0x77, 0x7D, 0x32, 0xF5, 0x29, 0x93, 0x9F, 0xD8, 0x3C, 0x1C, 0xB0 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

//Pointner to the data that is to be sent
static uint8_t* data;

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
// USE THIS TO CHANGE INTERVAL
const unsigned TX_INTERVAL = 300;   //5min

// Pin mapping
/*
//Feather M0
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 5, 6},
};
*/

//Heltec esp32
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32},
};

unsigned long startTime;
//Temperature chip i/o
OneWire ds(DS18S20_Pin);  // on digital pin 2
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

void setup(void) {
  //digitalWrite(SHUTDOWN_PIN, LOW);
  //pinMode(SHUTDOWN_PIN, OUTPUT);
  

  
  Serial.begin(115200);
    
    
    Serial.println("Starting");
    
    startTime = millis();
    u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.drawString(0, 0, "Starting");
            // put your setup code here, to r un once:
    
    data = (uint8_t*) calloc(PAYLOAD_SIZE, sizeof(uint8_t));    //Allocate the required number of bytes for the payload

    Serial.println("os_init");
    u8x8.drawString(0, 0, "os_init");
    // LMIC init
    os_init();
    Serial.println("LMIC_reset");
    u8x8.drawString(0, 0, "LMIC reset");
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    Serial.println("setLinkCheckMode");
    u8x8.drawString(0, 0, "setLinkCheckMode");
    LMIC_setLinkCheckMode(0);
    Serial.println("setDrTxpow");   
    u8x8.drawString(0, 0, "setDrTxpow");
    LMIC_setDrTxpow(DR_SF7,14);
    Serial.println("selectSubBand");
    u8x8.drawString(0, 0, "selectSubBand");
    LMIC_selectSubBand(1);

    Serial.println("do_send");
    u8x8.drawString(0, 0, "do_send");
    // Start job (sending automatically starts OTAA too)    
    do_send(&sendjob);
    
}

void loop(void) {
  os_runloop_once();
  
  
}

void onEvent (ev_t ev) {
    if(ev == EV_TXCOMPLETE) {
      u8x8.drawString(0, 0, "EV_TXCOMPLETE");
      powerDown();
    } else if(ev == EV_JOINED ) {
            Serial.println(F("EV_JOINED"));
            u8x8.drawString(0, 0, "EV_JOINED");
            u4_t netid = 0;
            devaddr_t devaddr = 0;
            u1_t nwkKey[16];
            u1_t artKey[16];
            LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
            // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
  }
}

void powerDown() {
  
  
  Serial.println(millis() - startTime);
  Serial.println("Finished");
  u8x8.drawString(0, 0, "Finished. Shutdown");
  
  digitalWrite(SHUTDOWN_PIN, LOW);
  pinMode(SHUTDOWN_PIN, OUTPUT);
}
/*
 * This function actually sends the data to the gateway
 */
void do_send(osjob_t* j){
// Check if there is not a current TX/RX job running
    Serial.println("Checking");
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
        u8x8.drawString(0, 0, "OP_TXRXPEND, no send");
    } else {
        Serial.println("Collecting data");
        u8x8.drawString(0, 0, "Collecting Data");
        // Prepare upstream data transmission at the next possible time.
        for(int i = 0; i < 30; i++) {
          collectData();
        }
        

          Serial.println("Sending");
          u8x8.drawString(0, 0, "Sending");
  LMIC_setTxData2(1, data, PAYLOAD_SIZE, 0);   //PAYLOAD_SIZE bytes being sent
        Serial.println(F("Packet queued"));
        u8x8.drawString(0, 0, "Packet Queued");
    }

  
  
}

/*
 * Collect data from sensors and battery and place it in the output pointer to be sent
 */
void collectData() {
  //int temp = (int) getTemp();
  //memcpy((void*) data, (void*) &temp, sizeof(int));
  
  
  float2bytes(getTemp(), (void*) (data));
}

/*
 * Convert a floating point number to 4 bytes at the output location provided
 * Uses memcopy, so don't have the output location be the same as the location of the input float
 */
void float2bytes(int input, void* outputLocation) {
  memcpy(outputLocation, (void*) &input, sizeof(int));
}

//From sensor library example, am not touching
int getTemp(){
  //returns the temperature from one DS18S20 in DEG Celsius
  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE); // Read Scratchpad

  
  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }
  
  ds.reset_search();
  
  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = (int16_t) ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  Serial.print("TEMP: ");
  Serial.println(TemperatureSum);
  
  return (int) TemperatureSum;
  
}