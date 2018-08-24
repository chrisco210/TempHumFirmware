#include <OneWire.h>
#include <U8g2lib.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

//Use these to set the shutdown output pin and the temp sensor pin
#define SHUTDOWN_PIN 23
#define DS18S20_Pin 17


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
const unsigned TX_INTERVAL = 300;   //5min

// Pin mapping
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

void setup(void) {
  data = (uint8_t*) calloc(PAYLOAD_SIZE, sizeof(uint8_t));    //Allocate the required number of bytes for the payload

  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  LMIC_setLinkCheckMode(0);
  LMIC_setDrTxpow(DR_SF7,14);
  LMIC_selectSubBand(1);

  do_send(&sendjob);  
}

void loop(void) {
  os_runloop_once();
}


//Event handler
void onEvent (ev_t ev) {
  //Transfer is complete, shutdown
  if(ev == EV_TXCOMPLETE) {
    powerDown();
  } else if(ev == EV_JOINED ) {   //Joined, get OTAA keys
    u4_t netid = 0;
    devaddr_t devaddr = 0;
    u1_t nwkKey[16];
    u1_t artKey[16];
    LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
    LMIC_setLinkCheckMode(0);
  }
}

//Send power down signal
void powerDown() {
  digitalWrite(SHUTDOWN_PIN, LOW);
  pinMode(SHUTDOWN_PIN, OUTPUT);
}
/*
 * This function actually sends the data to the gateway
 */
void do_send(osjob_t* j){
  if (!(LMIC.opmode & OP_TXRXPEND)) {
    
    //Collecting data 30 times, this ensures that the sensor is found and reads a proper value
    for(int i = 0; i < 30; i++) {
      collectData();
    }
    
    LMIC_setTxData2(1, data, PAYLOAD_SIZE, 0);
  }
}

/*
 * Collect data from sensors and battery and place it in the output pointer to be sent
 */
void collectData() {
  int temp = getTemp();
  
  int2bytes(getTemp(), (void*) (data));
}

/*
 * Convert a floating point number to 4 bytes at the output location provided
 * Uses memcopy, so don't have the output location be the same as the location of the input float
 */
void int2bytes(int input, void* outputLocation) {
  memcpy(outputLocation, (void*) &input, sizeof(int));
}

//From sensor library example, am not touching
int getTemp() {
  //returns the temperature from one DS18S20 in DEG Celsius
  byte data[12];
  byte addr[8];

  if(!ds.search(addr)) {
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

  
  for(int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }
  
  ds.reset_search();
  
  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = (int16_t) ((MSB << 8) | LSB);
  float TemperatureSum = tempRead / 16;

  Serial.print("TEMP: ");
  Serial.println(TemperatureSum);
  
  return (int) (TemperatureSum * 1000);
  
}
