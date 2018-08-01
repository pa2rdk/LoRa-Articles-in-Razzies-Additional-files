#include <SPI.h>
#include <LoRa.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <RDKAPRS.h>
#include <TimerOne.h>

#define bfPin   5   //afsk output must be PD5

int counter = 0;
bool validGPS = false;
static char conv_buf[16];

SoftwareSerial softSerial(3,4);
TinyGPSPlus gps;

void setup() {
	Serial.begin(9600);
	softSerial.begin(9600);
	while (!Serial);

	Serial.println("LoRa Sender");

	if (!LoRa.begin(433E6)) {
		Serial.println("Starting LoRa failed!");
		while (1);
	}
	  LoRa.setSignalBandwidth(125E3);
	  LoRa.setSpreadingFactor(9);
	//LoRa.setSyncWord(0xF3);
	//LoRa.dumpRegisters(Serial);
	LoRa.enableCrc();
	Timer1.initialize(76);
	InitBeacon();
}

void loop() {
	float flat, flon;

	smartDelay(2000);

	if ((!gps.location.isValid()) || (gps.location.age() > 3000))	{
		Serial.println(F(" Invalid position"));
		validGPS = false;
	} else {
		displayInfo();
		validGPS = true;
		flat = gps.location.lat();
		flon = gps.location.lng();
	}

	if (validGPS==true) {
		Beacon.setLat(deg_to_nmea(flat,true));
		Beacon.setLon(deg_to_nmea(flon,false));
		char dataIn[72];
		byte x= Beacon.getMessage(dataIn);
		Serial.println(dataIn);

		Serial.print("Sending packet: ");
		Serial.println(counter);

		LoRa.beginPacket();
		LoRa.print(dataIn);
		LoRa.endPacket();
		counter++;
	}
}

void displayInfo() {
	Serial.print(F("Location: "));
	if (gps.location.isValid())
	{
		Serial.print(gps.location.lat(), 6);
		Serial.print(F(","));
		Serial.print(gps.location.lng(), 6);
	}
	else
	{
		Serial.print(F("INVALID"));
	}

	Serial.print(F("  Date/Time: "));
	if (gps.date.isValid())
	{
		Serial.print(gps.date.month());
		Serial.print(F("/"));
		Serial.print(gps.date.day());
		Serial.print(F("/"));
		Serial.print(gps.date.year());
	}
	else
	{
		Serial.print(F("INVALID"));
	}
	Serial.print(F(" "));
	if (gps.time.isValid())
	{
		if (gps.time.hour() < 10) Serial.print(F("0"));
		Serial.print(gps.time.hour());
		Serial.print(F(":"));
		if (gps.time.minute() < 10) Serial.print(F("0"));
		Serial.print(gps.time.minute());
		Serial.print(F(":"));
		if (gps.time.second() < 10) Serial.print(F("0"));
		Serial.print(gps.time.second());
		Serial.print(F("."));
		if (gps.time.centisecond() < 10) Serial.print(F("0"));
		Serial.print(gps.time.centisecond());
	}
	else
	{
		Serial.print(F("INVALID"));
	}
	Serial.println();
}

void InitBeacon() {
	Beacon.begin(bfPin, 1200, 2200, 350);   //analog pin, led pin, freq1, freq2, shift freq
	Beacon.setCallsign("PA2RDK",6);
	Beacon.setDestination("APZRAZ",0);
	Beacon.setSymbol('>');
	Beacon.setComment("LoRa APRS Beacon");
	Beacon.setPath1("WIDE1",1);
	Beacon.setPath2("WIDE2",2);
	Beacon.setPower(1);
	Beacon.setHeight(0);
	Beacon.setGain(0);
	Beacon.setDirectivity(0);
	Beacon.setPreamble(140);
	Beacon.setTail(20);
	Beacon.printSettings();
}

char* deg_to_nmea(float fdeg, boolean is_lat) {
	long deg = fdeg*1000000;
	bool is_negative=0;
	if (deg < 0) is_negative=1;

	// Use the absolute number for calculation and update the buffer at the end
	deg = labs(deg);

	unsigned long b = (deg % 1000000UL) * 60UL;
	unsigned long a = (deg / 1000000UL) * 100UL + b / 1000000UL;
	b = (b % 1000000UL) / 10000UL;

	conv_buf[0] = '0';
	// in case latitude is a 3 digit number (degrees in long format)
	if( a > 9999) { snprintf(conv_buf , 6, "%04u", a);} else snprintf(conv_buf + 1, 5, "%04u", a);

	conv_buf[5] = '.';
	snprintf(conv_buf + 6, 3, "%02u", b);
	conv_buf[9] = '\0';
	if (is_lat) {
		if (is_negative) {conv_buf[8]='S';}
		else conv_buf[8]='N';
		return conv_buf+1;
		// conv_buf +1 because we want to omit the leading zero
	}
	else {
		if (is_negative) {conv_buf[8]='W';}
		else conv_buf[8]='E';
		return conv_buf;
	}
}

static void smartDelay(unsigned long ms)
{
	unsigned long start = millis();
	do
	{
		while (softSerial.available() > 0) gps.encode(softSerial.read());
	} while (millis() - start < ms);
}

void txing()
{
	delay(400);                                       //delay before sending data
	TCCR0B = (TCCR0B & 0b11111000) | 1;               //switch to 62500 HZ PWM frequency
	byte save_TIMSK0 = TIMSK0;                             //save Timer 0 register
	TIMSK0 = 0;                                       //disable Timer 0
	byte save_PCICR = PCICR;                               //save external pin interrupt register
	PCICR = 0;                                        //disable external pin interrupt
	Timer1.attachInterrupt(sinus_irq);                //warp interrupt in library
	Beacon.sendMessage();             				 //Beacon.sendPacket(track2, 72);
	Timer1.detachInterrupt();                         //disable timer1 interrupt
	analogWrite(bfPin, 0);                            //PWM at 0
	TCCR0B = (TCCR0B & 0b11111000) | 3;                 //register return to normal
	TIMSK0 = save_TIMSK0;
	PCICR = save_PCICR;
	Beacon.ptrStartNmea = 0;
	Serial.println("");
}

void sinus_irq()    //warp timer1 irq into DRAPRS lib
{
	Beacon.sinus();
}
