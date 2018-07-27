//#include <Adafruit_SleepyDog.h>
#include <Mouse.h>

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <DHT.h>

#define BATPIN A7     //Battery level pin
#define ACTIVEPIN 11   //Activity LED pin
#define DHTPIN 10    // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)

//# of bytes in the payload
#define PAYLOAD_SIZE 12


DHT dht(DHTPIN, DHTTYPE);

// This EUI must be in little-endian format, so least-significant-byte
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
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 5, 6},
};
//Event handler based on https://github.com/matthijskooijman/arduino-lmic/blob/master/examples/ttn-abp/ttn-abp.ino
//Many events have been removed, you may want to re add them if you are having issues
void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
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
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
            //Watchdog.sleep(270000);   //4.5 min
            break;
            break;
         default:
            Serial.print(F("Event Received: "));
            Serial.println(ev);
            break;
    }
}


/*
 * This function actually sends the data to the gateway
 */
void do_send(osjob_t* j){
  collectData();    //Gather the data to be sent
  
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, data, PAYLOAD_SIZE, 0);   //PAYLOAD_SIZE bytes being sent
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

/*
 * Collect data from sensors and battery and place it in the output pointer to be sent
 */
void collectData() {
  float2bytes(dht.readHumidity(), (void*) data);
  float2bytes(dht.readTemperature(), (void*) (data + 4));

  float batV = analogRead(BATPIN);
  batV *= 2;    
  batV *= 3.3;
  batV /= 1024;
  
  float2bytes(batV, (void*) (data + 8));
}

/*
 * Convert a floating point number to 4 bytes at the output location provided
 * Uses memcopy, so don't have the output location be the same as the location of the input float
 */
void float2bytes(float input, void* outputLocation) {
  memcpy(outputLocation, (void*) &input, sizeof(float));
}

void setup() {
  // put your setup code here, to r un once:
  data = (uint8_t*) calloc(PAYLOAD_SIZE, sizeof(uint8_t));    //Allocate the required number of bytes for the payload
    delay(5000);
    while (! Serial)
        ;
    Serial.begin(9600);
    Serial.println(F("Starting"));
    dht.begin();

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    LMIC_setLinkCheckMode(0);
    LMIC_setDrTxpow(DR_SF7,14);
    LMIC_selectSubBand(1);
    
    // Start job (sending automatically starts OTAA too)    
    do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}
