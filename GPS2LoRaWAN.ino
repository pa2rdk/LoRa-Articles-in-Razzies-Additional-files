/*******************************************************************************
   Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

   This example will send Temperature and Humidity
   using frequency and encryption settings matching those of
   the The Things Network. Application will 'sleep' 7x8 seconds (56 seconds)

   This uses OTAA (Over-the-air activation), where where a DevEUI and
   application key is configured, which are used in an over-the-air
   activation procedure where a DevAddr and session keys are
   assigned/generated for use with all further communication.

   Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
   g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
   violated by this sketch when left running for longer)!

   To use this sketch, first register your application and device with
   the things network, to set or generate an AppEUI, DevEUI and AppKey.
   Multiple devices can use the same AppEUI, but each device has its own
   DevEUI and AppKey.

   Do not forget to define the radio type correctly in config.h.

 *******************************************************************************/

#include <avr/wdt.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include <Arduino.h>

bool joined = false;
bool sleeping = false;
unsigned long time;

#define LedPin	13
#define sendPin	5
#define fixPin	3
#define GPSPowerPin 10
#define VBATPIN A9

int byteGPS=-1;
char linea[300] = "";
char buffer[15] = "";
char comandoGPR[7] = "$GPRMC";
int cont=0;
int bien=0;
int conta=0;
int indices[13];

#define gpsPort Serial1

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.

// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = { 0x60, 0x56, 0x43, 0xE4, 0xDB, 0x04, 0x8A, 0x44, 0xED, 0x9A, 0x5C, 0x2E, 0xDC, 0x5E, 0x60, 0xBD };

// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = { 0x2C, 0xA1, 0x2F, 0x71, 0x9D, 0xFD, 0x20, 0x9E, 0x71, 0x32, 0x0F, 0x2E, 0x33, 0xD1, 0x94, 0xCE };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = { 0x26011407 };

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static void initfunc (osjob_t*);

int32_t lat;
int32_t lon;
int32_t alt=1;
bool fix;

//// provide APPEUI (8 bytes, LSBF)
//void os_getArtEui (u1_t* buf) {
//  memcpy(buf, APPEUI, 8);
//}
//
//// provide DEVEUI (8 bytes, LSBF)
//void os_getDevEui (u1_t* buf) {
//  memcpy(buf, DEVEUI, 8);
//}
//
//// provide APPKEY key (16 bytes)
//void os_getDevKey (u1_t* buf) {
//  memcpy(buf, APPKEY, 16);
//}

static osjob_t sendjob;
static osjob_t initjob;

// Pin mapping is hardware specific.
// Pin mapping
const lmic_pinmap lmic_pins = {
		.nss = 8,
		.rxtx = LMIC_UNUSED_PIN,
		.rst = 4, // Needed on RFM92/RFM95? (probably not) D0/GPIO16
		.dio = {7, 6, LMIC_UNUSED_PIN}, // Specify pin numbers for DIO0, 1, 2
		// connected to D7, D6, -
};

void onEvent (ev_t ev) {
	if (ev == EV_TXCOMPLETE){
		digitalWrite(sendPin, HIGH);
		sleeping = true;
	}

	//  int i, j;
	//  switch (ev) {
	//  case EV_SCAN_TIMEOUT:
	//    Serial.println(F("EV_SCAN_TIMEOUT"));
	//    break;
	//  case EV_BEACON_FOUND:
	//    Serial.println(F("EV_BEACON_FOUND"));
	//    break;
	//  case EV_BEACON_MISSED:
	//    Serial.println(F("EV_BEACON_MISSED"));
	//    break;
	//  case EV_BEACON_TRACKED:
	//    Serial.println(F("EV_BEACON_TRACKED"));
	//    break;
	//  case EV_JOINING:
	//    Serial.println(F("EV_JOINING"));
	//    break;
	//  case EV_JOINED:
	//    Serial.println(F("EV_JOINED"));
	//    // Disable link check validation (automatically enabled
	//    // during join, but not supported by TTN at this time).
	//    LMIC_setLinkCheckMode(0);
	//    digitalWrite(LedPin, LOW);
	//    // after Joining a job with the values will be sent.
	//    joined = true;
	//    break;
	//  case EV_RFU1:
	//    Serial.println(F("EV_RFU1"));
	//    break;
	//  case EV_JOIN_FAILED:
	//    Serial.println(F("EV_JOIN_FAILED"));
	//    break;
	//  case EV_REJOIN_FAILED:
	//    Serial.println(F("EV_REJOIN_FAILED"));
	//    // Re-init
	//    // os_setCallback(&initjob, initfunc);
	//    break;
	//  case EV_TXCOMPLETE:
	//    sleeping = true;
	//    if (LMIC.dataLen) {
	//      // data received in rx slot after tx
	//      // if any data received, a LED will blink
	//      // this number of times, with a maximum of 10
	//      Serial.print(F("Data Received: "));
	//      Serial.println(LMIC.frame[LMIC.dataBeg], HEX);
	//      i = (LMIC.frame[LMIC.dataBeg]);
	//      // i (0..255) can be used as data for any other application
	//      // like controlling a relay, showing a display message etc.
	//      if (i > 10) {
	//        i = 10;   // maximum number of BLINKs
	//      }
	//      for (j = 0; j < i; j++)
	//      {
	//        digitalWrite(LedPin, HIGH);
	//        delay(200);
	//        digitalWrite(LedPin, LOW);
	//        delay(400);
	//      }
	//    }
	//    Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
	//    delay(50);  // delay to complete Serial Output before Sleeping
	//
	//    // Schedule next transmission
	//    // next transmission will take place after next wake-up cycle in main loop
	//    break;
	//  case EV_LOST_TSYNC:
	//    Serial.println(F("EV_LOST_TSYNC"));
	//    break;
	//  case EV_RESET:
	//    Serial.println(F("EV_RESET"));
	//    break;
	//  case EV_RXCOMPLETE:
	//    // data received in ping slot
	//    Serial.println(F("EV_RXCOMPLETE"));
	//    break;
	//  case EV_LINK_DEAD:
	//    Serial.println(F("EV_LINK_DEAD"));
	//    break;
	//  case EV_LINK_ALIVE:
	//    Serial.println(F("EV_LINK_ALIVE"));
	//    break;
	//  default:
	//    Serial.println(F("Unknown event"));
	//    break;
	//  }
}

void do_send(osjob_t* j) {
	byte buffer[22];
	int32_t i;
	int32_t h;
	float measuredvbat = analogRead(VBATPIN);

	i = 0;
	Serial.println(F("Beacon?"));
	if (fix) {
		buffer[i++] = 0x01;
		buffer[i++] = 0x88;
		h = lat / 1000L;
		buffer[i++] = (h >> 16) & 0xff;
		buffer[i++] = (h >> 8) & 0xff;
		buffer[i++] = h & 0xff;
		h = lon / 1000L;
		buffer[i++] = (h >> 16) & 0xff;
		buffer[i++] = (h >> 8) & 0xff;
		buffer[i++] = h & 0xff;
		buffer[i++] = (alt >> 16) & 0xff;
		buffer[i++] = (alt >> 8) & 0xff;
		buffer[i++] = alt & 0xff;

		measuredvbat *= 2;    // we divided by 2, so multiply back
		measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
		measuredvbat /= 1024; // convert to voltage
		h = measuredvbat * 100;
		Serial.print("VBat: " ); Serial.println(h);
		buffer[i++] = 0x02;
		buffer[i++] = 0x02;
		buffer[i++] = (h >> 8) & 0xff;
		buffer[i++] = h & 0xff;

		Serial.println(lat);
		Serial.println(lon);

		if (LMIC.opmode & OP_TXRXPEND) {
			Serial.println(F("OP_TXRXPEND, not sending"));
		} else {
			// Prepare upstream data transmission at the next possible time.
			LMIC_setTxData2(1, (uint8_t*) buffer, i , 0);
			Serial.println(F("Sending"));
		}
	} else {
		digitalWrite(sendPin, HIGH);
		sleeping = true;
	}
}

// initial job
static void initfunc (osjob_t* j) {
	// reset MAC state
	LMIC_reset();
	LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
	// start joining
	LMIC_startJoining();
	// init done - onEvent() callback will be invoked...
}

void setup()
{
	// Disable GPS power
	pinMode(GPSPowerPin, OUTPUT);
	digitalWrite(GPSPowerPin, HIGH);
	pinMode(sendPin, OUTPUT);
	digitalWrite(sendPin, LOW);
	pinMode(fixPin, OUTPUT);
	digitalWrite(fixPin, LOW);
	delay(10000);
	Serial.begin(115200);
	Serial.println(F("Starting"));
	delay(5000);


	// LMIC init
	os_init();

	// Reset the MAC state. Session and pending data transfers will be discarded.
	LMIC_reset();

	// Set static session parameters. Instead of dynamically establishing a session
	// by joining the network, precomputed session parameters are be provided.
	uint8_t appskey[sizeof(APPSKEY)];
	uint8_t nwkskey[sizeof(NWKSKEY)];
	memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
	memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
	LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);

	// Select frequencies range
	//LMIC_selectSubBand(0);

	// Disable link check validation
	LMIC_setLinkCheckMode(0);

	// TTN uses SF9 for its RX2 window.
	LMIC.dn2Dr = DR_SF9;

	// Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
	LMIC_setDrTxpow(DR_SF9,14);

	digitalWrite(GPSPowerPin, LOW);
	digitalWrite(sendPin, HIGH);
	digitalWrite(fixPin, HIGH);

	gpsPort.begin(9600);
	time = millis();

	// Start job
	//do_send(&sendjob);
}

void loop()
{
	smartDelay(2000);

	if (millis() - time >= 3000) {
		do_send(&sendjob);    // Send sensor values
		while (sleeping == false)
		{
			digitalWrite(sendPin, LOW);
			os_runloop_once();
		}
		sleeping = false;
		delay(10000);
		time = millis();
	}

	digitalWrite(LedPin, ((millis() / 100) % 2) && (joined == false));
}

static void smartDelay(unsigned long ms)
{
	unsigned long start = millis();
	do
	{
		char buffer[15] = "";
		byteGPS=Serial1.read();         // Read a byte of the serial port
		if (byteGPS == -1) {           // See if the port is empty yet
			delay(100);
		} else {
			// note: there is a potential buffer overflow here!
			linea[conta]=byteGPS;        // If there is serial port data, it is put in the buffer
			conta++;
			if (byteGPS==13){            // If the received byte is = to 13, end of transmission
				// note: the actual end of transmission is <CR><LF> (i.e. 0x13 0x10)

				cont=0;
				bien=0;
				// The following for loop starts at 1, because this code is clowny and the first byte is the <LF> (0x10) from the previous transmission.
				for (int i=1;i<7;i++){     // Verifies if the received command starts with $GPR
					if (linea[i]==comandoGPR[i-1]){
						bien++;
					}
				}
				if(bien==6){               // If yes, continue and process the data
					for (int i=0;i<300;i++){
						if (linea[i]==','){    // check for the position of the  "," separator
							// note: again, there is a potential buffer overflow here!
							indices[cont]=i;
							cont++;
						}
						if (linea[i]=='*'){    // ... and the "*"
							indices[12]=i;
							cont++;
						}
					}

					for (int i=0;i<12;i++){
						switch(i){
						//          case 0 :Serial.print("Time in UTC (HhMmSs): ");break;
						//          case 1 :Serial.print("Status (A=OK,V=KO): ");break;
						//          case 2 :Serial.print("Latitude: ");break;
						//          case 3 :Serial.print("Direction (N/S): ");break;
						//          case 4 :Serial.print("Longitude: ");break;
						//          case 5 :Serial.print("Direction (E/W): ");break;
						//          case 6 :Serial.print("Velocity in knots: ");break;
						//          case 7 :Serial.print("Heading in degrees: ");break;
						//          case 8 :Serial.print("Date UTC (DdMmAa): ");break;
						//          case 9 :Serial.print("Magnetic degrees: ");break;
						//          case 10 :Serial.print("(E/W): ");break;
						//          case 11 :Serial.print("Mode: ");break;
						//          case 12 :Serial.print("Checksum: ");break;
						}
						int x = 0;
						buffer[x] = 0;
						for (int j=indices[i];j<(indices[i+1]-1);j++){
							buffer[x]=linea[j+1];
							x++;
						}
						buffer[x]=0;
						if (i==1){
							if (buffer[0]=='A'){
								fix = true;
								digitalWrite(fixPin, LOW);
								Serial.println(F("Fix!"));
							}else{
								fix = false;
								digitalWrite(fixPin, HIGH);
								Serial.println(F("No fix"));
							}
						}
						if (i==2){
							lat=atof(buffer)*100000;
							lat=decimalDegrees(lat);
						}
						if (i==4){
							lon=atof(buffer)*100000;
							lon=decimalDegrees(lon);
						}
					}
				}
				conta=0;                    // Reset the buffer
				for (int i=0;i<300;i++){    //
					linea[i]=' ';
				}
			}
		}
	} while (millis() - start < ms);
}

uint32_t decimalDegrees(uint32_t nmeaCoord) {
	uint32_t wholeDegrees = (nmeaCoord/10000000)*10000000;
	uint32_t restDegrees = nmeaCoord-wholeDegrees;
	restDegrees = restDegrees/0.6;
	return wholeDegrees + restDegrees;
}
