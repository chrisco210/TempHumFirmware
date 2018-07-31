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
const unsigned TX_INTERVAL = 10;   //5min

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 5, 6},
};

//Event handler provided by https://github.com/matthijskooijman/arduino-lmic/blob/master/examples/ttn-abp/ttn-abp.ino
void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("artKey: ");
              for (int i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                Serial.print(artKey[i], HEX);
              }
              Serial.println("");
              Serial.print("nwkKey: ");
              for (int i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      Serial.print(nwkKey[i], HEX);
              }
              Serial.println("");
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
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
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
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

void setup() {
    data = (uint8_t*) calloc(PAYLOAD_SIZE, sizeof(uint8_t));    //Allocate the required number of bytes for the payload
    delay(5000);
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

void loop() {
    os_runloop_once();
}
