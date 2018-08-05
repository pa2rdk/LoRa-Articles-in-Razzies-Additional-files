/*******************************************************************************
   Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
   Changes made R.J. de Kok - (c) 2018

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
#include <TinyGPS++.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "LowPower.h"
#include <Arduino.h>

bool joined = false;
bool txComplete = false;
bool ledHIGH = HIGH;
bool ledLOW = LOW;

#define LedPin	13	//should be 3 for tracker and 13 for internal led (BMW)
#define sendPin	5
#define powerPin 12
#define VBATPIN A9
#define EXTVOLTAGE A0

#define isRDK

const int wakeUpPin = 1;
const int maxLoop = 75;
bool moveDetected = false;

int32_t latitude= 0;
int32_t longitude = 0;
int32_t height = 0;
int32_t speed = 0;

TinyGPSPlus gps;

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.

#ifdef isRDK
// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = { 0x60, 0x56, 0x43, 0xE4, 0xDB, 0x04, 0x8A, 0x44, 0xED, 0x9A, 0x5C, 0x2E, 0xDC, 0x5E, 0x60, 0xBD };

// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = { 0x2C, 0xA1, 0x2F, 0x71, 0x9D, 0xFD, 0x20, 0x9E, 0x71, 0x32, 0x0F, 0x2E, 0x33, 0xD1, 0x94, 0xCE };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = { 0x26011407 };

#else
// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = { 0x3B, 0xFF, 0xBD, 0x08, 0xF2, 0xE5, 0x38, 0xB7, 0xAC, 0x87, 0x9F, 0xFF, 0x98, 0x19, 0x3B, 0xBD };

// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = { 0xD5, 0x4F, 0x28, 0x25, 0xDF, 0xA5, 0x95, 0xB0, 0x64, 0xA1, 0x50, 0xD7, 0xE8, 0x04, 0x5D, 0xBB };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = { 0x260114BA };
#endif

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static void initfunc (osjob_t*);

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

void wakeUp()
{
	// Just a handler for the pin interrupt.
	moveDetected = true;
}

void onEvent (ev_t ev) {
	if (ev == EV_TXCOMPLETE){
		Serial.println(F("TX Complete"));
		digitalWrite(sendPin, ledLOW);
		txComplete = true;
		delay(50);
	}
}

void do_send(osjob_t* j) {
	byte buffer[22];
	int32_t i;
	int32_t h;
	float measuredvbat = analogRead(VBATPIN);
	int extVoltage = analogRead(EXTVOLTAGE);

	i = 0;
	buffer[i++] = 0x01;
	buffer[i++] = 0x88;
	buffer[i++] = latitude >> 16;
	buffer[i++] = latitude >> 8;
	buffer[i++] = latitude;
	buffer[i++] = longitude >> 16;
	buffer[i++] = longitude >> 8;
	buffer[i++] = longitude;
	buffer[i++] = height;
	buffer[i++] = height >> 8;
	buffer[i++] = gps.hdop.value();

	measuredvbat *= 2;    // we divided by 2, so multiply back
	measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
	measuredvbat /= 1024; // convert to voltage
	h = measuredvbat * 100;
	Serial.print("VBat: " ); Serial.println(h);
	buffer[i++] = 0x02;
	buffer[i++] = 0x02;
	buffer[i++] = (h >> 8) & 0xff;
	buffer[i++] = h & 0xff;

	buffer[i++] = 0x03;
	buffer[i++] = 0x02;
	buffer[i++] = (speed >> 8) & 0xff;
	buffer[i++] = speed & 0xff;

	buffer[i++] = 0x04;
	buffer[i++] = 0x02;
	buffer[i++] = (extVoltage >> 8) & 0xff;
	buffer[i++] = extVoltage & 0xff;

	if (LMIC.opmode & OP_TXRXPEND) {
		Serial.println(F("OP_TXRXPEND, not sending"));
	} else {
		// Prepare upstream data transmission at the next possible time.
		LMIC_setTxData2(1, (uint8_t*) buffer, i , 0);
		Serial.println(F("Sending"));
	}
}

// initial job
static void initfunc (osjob_t* j) {
	// reset MAC state
	LMIC_reset();
	LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
	// start joining
	// LMIC_startJoining();
	// init done - onEvent() callback will be invoked...
}

void forceTxSingleChannelDr(int channel) {
	for(int i=0; i<9; i++) { // For EU; for US use i<71
		if(i != channel) {
			LMIC_disableChannel(i);
		}
	}
	// Set data rate (SF) and transmit power for uplink
	LMIC_setDrTxpow(DR_SF9, 14);
}


void setup()
{
	// Disable GPS power
	pinMode(powerPin, OUTPUT);
	digitalWrite(powerPin, LOW);
	pinMode(sendPin, OUTPUT);
	digitalWrite(sendPin, ledHIGH);
	pinMode(LedPin, OUTPUT);
	digitalWrite(LedPin, ledHIGH);
	pinMode(wakeUpPin, INPUT);
	delay(5000);
	Serial.begin(115200);
	Serial.println(F("Starting"));
	delay(2000);

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
	//LMIC_setDrTxpow(DR_SF9,14);
	forceTxSingleChannelDr(1);

	Serial1.begin(9600);
	Serial1.setTimeout(1000);
	//Serial1.setTimeout(2);
	digitalWrite(sendPin, ledLOW);
	digitalWrite(LedPin, ledLOW);
}

void loop()
{
	//	digitalWrite(sendPin, LOW);
	//	delay(60);
	//	digitalWrite(sendPin, HIGH);
	unsigned long start;
	int extVoltage = analogRead(EXTVOLTAGE);
	Serial.println(extVoltage);

	digitalWrite(powerPin, HIGH);
	digitalWrite(LedPin, ledHIGH);

	bool validGPS = false;
	Serial.print(F("Wait for fix"));
	while (!validGPS){
		start = millis();
		do
		{
			while (Serial1.available()) Serial.write(Serial1.read());
		} while (millis() - start < 2000);
		start = millis();
		do
		{
			while (Serial1.available()) gps.encode(Serial1.read());
		} while (millis() - start < 2000);
		validGPS=(gps.location.isValid()==true && gps.location.isUpdated()==true);
		Serial.print(".");
	}
	latitude= gps.location.lat()*10000;
	longitude = gps.location.lng()*10000;
	height = gps.altitude.meters();
	speed = gps.speed.kmph()*100;
	Serial.println();
	Serial.println(latitude);
	Serial.println(longitude);
	Serial.println(height);
	Serial.println(speed);

	int waitLoop = maxLoop;
	if (extVoltage>700) waitLoop = 4;
	if (speed>100) waitLoop = 4;
	if (speed>500) waitLoop = 0;

	start=millis();
	if (validGPS==true) {
		digitalWrite(sendPin, ledHIGH);
		do_send(&sendjob);    // Send sensor values
		start = millis();
		while (txComplete==false){
			os_runloop_once();
			if (millis() - start > 60000) {
				Serial.println(F("Waited 60 secs for TX complete"));
				start=millis();
			}
		}
		txComplete=false;
		delay(50);
	}


	if (waitLoop>0){
		digitalWrite(powerPin, LOW);
		digitalWrite(LedPin, ledLOW);
		moveDetected = false;
		attachInterrupt(0, wakeUp, HIGH);
		for(int x=0;x<waitLoop;x++){
			LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
			digitalWrite(LedPin, ledHIGH);
			LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF);
			digitalWrite(LedPin, ledLOW);
			if (moveDetected==true){
				x = waitLoop;
				Serial.println(F("Move detected"));
			}
		}
		//LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
		detachInterrupt(0);
	}
	delay(1000);
}


