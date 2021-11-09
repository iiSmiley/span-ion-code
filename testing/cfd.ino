#include <SPI.h> // include the SPI library
/* --------------------------------- */
/* --- Pin Mappings (Teensy 3.6) --- */
/* --------------------------------- */
// Analog scan chain
const int pin_scan_inb 			= A15;	// ASC input data
const int pin_scan_outb 		= A18;	// ASC output data
const int pin_scan_clk 			= A17;	// ASC clock
const int pin_scan_loadb 		= A16;	// ASC load

// Analog output signal voltages
const int pin_dac_small 		= A0; 	// Resistive DAC for small chain
const int pin_dac_main 			= A0;	// Resistive DAC for main chain
const int pin_bandgap			= A0;	// Test structure bandgap voltage source
const int pin_pk_out 			= A0; 	// Peak detector output voltage

// On-chip supply voltages
const int pin_vddaon 			= A0;	// Always-on LDO output
const int pin_vddmain			= A0;	// Main signal chain LDO output
const int pin_vddsmall 			= A0; 	// Small signal chain LDO output

const int pin_pk_rst			= A14;	// Reset for test structure peak detector
const int pin_tdc_rstb 			= A3;	// Inverted reset ramp for TDC

// TDC SPI and other connections
// TODO Don't worry about how these are all currently A0
// What matters is the names on the left side
const int pin_spi_main_csb 		= A0;	// CSb for the main signal chain TDC
const int pin_spi_main_din		= A0; 	// Data-in pin for the main signal chain TDC
const int pin_spi_main_dout 	= A0;	// Data-out pin for the main signal chain TDC
const int pin_spi_main_clk		= A0;	// SPI clock for the main signal chain TDC
const int pin_spi_main_intrpt	= A0;	// Interrupt pin for the TDC for the main signal chain

const int pin_spi_small_csb 	= A0;	// CSb for the small signal chain TDC
const int pin_spi_small_din		= A0; 	// Data-in pin for the small signal chain TDC
const int pin_spi_small_dout 	= A0;	// Data-out pin for the small signal chain TDC
const int pin_spi_small_clk		= A0;	// SPI clock for the small signal chain TDC
const int pin_spi_small_intrpt	= A0;	// Interrupt pin for the TDC for the small signal chain

// Constants
const int N_SCAN 				= 44; 	// Number of scan bits
const int N_TDC_CONFIG 			= 2; 	// TODO: Number of TDC config bits
const int B_ADC 				= 16;	// Number of bits for Teensy analogRead
const int CHAIN_MAIN			= 0;	// Used to indicate the main signal chain
const int CHAIN_SMALL			= 1;	// Used to indicate the small signal chain

/* ----------------------------- */
/* --- Runs once at power-on --- */
/* ----------------------------- */
// Variables for command interpreter
String inputString = "";
boolean stringComplete = false;

void setup() {
	// Open USB serial port
	Serial.begin(9600);

	// Reserve 200 bytes for the inputString
	inputString.reserve(200);

	// Set up pins
	// - Analog Scan Chain
	pinMode(pin_scan_inb, 			OUTPUT);
	pinMode(pin_scan_outb, 			INPUT);
	pinMode(pin_scan_clk, 			OUTPUT);
	pinMode(pin_scan_loadb, 		OUTPUT);

	digitalWrite(pin_scan_inb, 		LOW);
	digitalWrite(pin_scan_clk, 		LOW);
	digitalWrite(pin_scan_loadb, 	HIGH);

	// - Test Structures
	pinMode(pin_rst_test, 			OUTPUT);

	digitalWrite(pin_rst_test, 		LOW);

	// - TODO for Lydia: set up/initialize the rest of the pins

    pinMode(pin_dac_small, 			OUTPUT);
    pinMode(pin_dac_main, 			OUTPUT);
    pinMode(pin_bandgap, 			OUTPUT);
    pinMode(pin_pk_out, 			INPUT);

    digitalWrite(pin_dac_main, 		);
	digitalWrite(pin_dac_small, 	);
	digitalWrite(pin_bandgap, 	    );

    // - On-chip supply voltages setup
    pinMode(pin_vddaon, 			INPUT);
    pinMode(pin_vddmain, 			INPUT);
    pinMode(pin_vddsmall, 			INPUT);

    pinMode(pin_pk_rst, 			OUTPUT);
    pinMode(pin_tdc_rstb, 			INPUT);
    digitalWrite(pin_pk_rst, 	    LOW);

    pinMode(pin_spi_main_csb, 		OUTPUT);
    pinMode(pin_spi_main_din, 		OUTPUT);
    pinMode(pin_spi_main_dout, 		INPUT);
    pinMode(pin_spi_main_clk, 		OUTPUT);
    pinMode(pin_spi_main_intrpt, 	INPUT);
    digitalWrite(pin_spi_main_csb,  HIGH);
    digitalWrite(pin_spi_main_din, 	);
    digitalWrite(pin_spi_main_clk, 	);

    pinMode(pin_spi_small_csb, 		OUTPUT);
    pinMode(pin_spi_small_din, 		OUTPUT);
    pinMode(pin_spi_small_dout, 	INPUT);
    pinMode(pin_spi_small_clk, 		OUTPUT);
    pinMode(pin_spi_small_intrpt, 	INPUT);
    digitalWrite(pin_spi_small_csb, HIGH);
    digitalWrite(pin_spi_small_din, );
    digitalWrite(pin_spi_small_clk, );

	analogReadResolution(16);	// 16B -> 13ENOB

	// Initialize SPI
	SPI.begin();

	// TODO for Lydia: enable 16MHz clock for TDC reference
    analogWriteFrequency(pin_spi_main_clk, 16000000);
    analogWriteFrequency(pin_spi_small_clk, 16000000);
}

/* ----------------------- */
/* --- Runs repeatedly --- */
/* ----------------------- */
void loop() {
    analogWrite(pin_spi_small_clk, 128);
    analogWrite(pin_spi_main_clk, 128);
	if (stringComplete){
		if (inputString == 'ascwrite\n'){
			asc_write();
		}
		else if (inputString == 'ascread\n'){
			asc_read();
		}
		else if (inputString == 'ascload\n'){
			asc_load();
		}
		else if (inputString == 'tdcmainread\n') {
			tdc_read(CHAIN_MAIN);
		}
		else if (inputString == 'tdcsmallread\n') {
			tdc_read(CHAIN_SMALL);
		}
		else if (inputString == 'tdcconfig\n') {
			tdc_write();
		}
		else if (inputString == 'peakreset\n') {
			peak_reset();
		}

		// Reset to listen for a new '\n' terminated string over serial
		inputString = "";
		stringComplete = false;
	}
}

/* ------------------------- */
/* --- Analog Scan Chain --- */
/* ------------------------- */
void atick() {
/*
	10kHz clock, 50% duty cycle.
*/
	digitalWrite(pin_scan_clk, LOW);
	delayMicroseconds(50);

	// Read
	digitalWrite(pin_scan_clk, HIGH);
	delayMicroseconds(50);
}

void asc_write() {
/*
	Writes the data into the on-chip shift register.
	NB: The inversion is handled internally! That is, if  bits=000
	or bitsb=111, the input should be 000 and code here will feed in 111.
*/
	Serial.println("Executing ASC write");
	int count = 0;
	char scanbits[N_SCAN];

	// loop until all scan bits are received over serial
	while (count != N_SCAN) {
		
		// read one bit at a time over serial
		if (Serial.available()) {
			scanbits[count] = Serial.read();
			count ++;
		}
	}

	// Once all bits are received, bitbang to ASC input
	for (int x=0; x<N_SCAN; x++) {
		if (scanbits[x] == '1') {
			digitalWrite(pin_scan_inb, LOW);
		}
		else if (scanbits[x] == '0') {
			digitalWrite(pin_scan_inb, HIGH);
		}
		else {
			Serial.println("Error in ASC write");
		}

		// Pulse clock
		atick();
	}
	Serial.println("ASC write complete");
}

void asc_load() {
/*
	Latches the loaded data to the outputs of the scan chain.
	You must call asc_write() before calling this!
*/
	Serial.println("Executing ASC load");
	digitalWrite(pin_scan_loadb, LOW);
	digitalWrite(pin_scan_loadb, HIGH);
}

void asc_read() {
/*
	Reads data from the single pin on the output of the scan chain. Sends
	the data (not datab) back through to the serial.
*/
	String st = "";
	int valb = 1;

	// First bit should be available
	valb = digitalRead(pin_scan_outb);
	if (valb) {
		st = String(st + "0");
	}
	else {
		st = String(st + "1");
	}

	// Send the first bit back
	Serial.print(st);
	st = "";

	// Rest of the bits
	for (int i=1; i<N_bits; i=i+1) {
		atick();
		valb = digitalRead(pin_scan_outb);
		if (valb) {
			st = String(st + "0");
		}
		else {
			st = String(st + "1");
		}
	}

	// Send the rest of the bits
	Serial.print(st);
	st = "";

	// Terminator
	Serial.println();
}

/* ----------------------------------- */
/* --- TDC Control & Communication --- */
/* ----------------------------------- */
void tdc_write(int chain) {
/*
	Uses the Teensy' SPI library to program the TDC using
	the TDC's SPI interface.
	SPI library documentation:
		https://www.pjrc.com/teensy/td_libs_SPI.html
	TI TDC device documentation:
		https://www.ti.com/lit/ds/symlink/tdc7200.pdf?ts=1636125541217 

	Inputs:
		chain: CHAIN_MAIN or CHAIN_SMALL. The former indicates that
			we'd like to program the TDC associated with the 
			main signal chain; the latter indicates we'd like to 
			program the TDC associated with the small signal
			chain.
*/
	Serial.println("Executing TDC config");
	int count = 0;
	char configbits[N_TDC_CONFIG];// 2 byte for a single transaction

	// TODO tell the SPI software which pins to interact with
	// based on if we're dealing with the small signal chain or the main
	// signal chain
	if(chain){
	    digitalWrite(pin_spi_small_csb, LOW);
	    digitalWrite(pin_spi_main_csb, HIGH);
	}
	else{
	    digitalWrite(pin_spi_small_csb, HIGH);
	    digitalWrite(pin_spi_main_csb, LOW);
	}
	// loop until all bits are received over serial from the computer
	while (count != N_TDC_CONFIG) {
		
		// read one bit at a time over serial
		if (Serial.available()) {
			configbits[count] = Serial.read();
			count ++;
		}
	}

	// TODO once all bits are received, forward them to the TDC
	// over SPI
	count = 0;
	SPISettings settings_main(16000000, MSBFIRST, SPI_MODE1);// not sure with mode
    SPISettings settings_small(16000000, MSBFIRST, SPI_MODE1);
    while (count != N_TDC_CONFIG) {

		// transfer one byte at a time over SPI
		SPI.transfer(configbits[count]);
		count ++;
	}
	digitalWrite(pin_spi_main_csb, HIGH);
	digitalWrite(pin_spi_small_csb, HIGH);
    SPI.endTransaction();

	Serial.println("TDC config complete");
}

void tdc_read(int chain) {
/*
	Uses the Teensy's SPI library to read from the TDC
	using the TDC's SPI interface.

	Inputs:
		chain: CHAIN_MAIN or CHAIN_SMALL to indicate which
			TDC to communicate with. CHAIN_MAIN (the variable name)
			indicates that we'd like to read from the TDC
			that's connected to the main signal chain. CHAIN_SMALL
			indicates that we'd like to read from the TDC that's 
			connected to the small signal chain.
*/
	// TODO tell the SPI software which pins to interact with
	// based on if we're dealing with the small signal chain or the main
	// signal chain
    if(chain){
	    digitalWrite(pin_spi_small_csb, LOW);
	    digitalWrite(pin_spi_main_csb, HIGH);
	}
	else{
	    digitalWrite(pin_spi_small_csb, HIGH);
	    digitalWrite(pin_spi_main_csb, LOW);
	}
	// TODO retrieve the bits from the TDC
	int count = 0;
    char val[N_TDC_CONFIG];
    while (count != N_TDC_CONFIG) {

		// transfer one byte at a time over SPI
		val[count] = SPI.transfer(0);
		count ++;
		// TODO check if the TDC has indicated overflow
		if(chain){
	        if(pin_spi_small_intrpt){
	            Serial.println("TDC interuption");
                return;
	        }
	    }
	    else{
	        if(pin_spi_small_intrpt){
	            Serial.println("TDC interuption");
	            return;
	        }
	    }
	}
    digitalWrite(pin_spi_main_csb, HIGH);
	digitalWrite(pin_spi_small_csb, HIGH);
    SPI.endTransaction();
	// TODO forward the bits to the computer over serial
	count = 0;
    while (count != ) {

		// read one bit at a time over serial
		if (Serial.available()) {
			Serial.write(val(count));
			count ++;
		}
	}
	// Terminator
	Serial.println("TDC read complete")
}


/* ------------------------------------------ */
/* --- On-Chip Test Structure Interaction --- */
/* ------------------------------------------ */
void peak_reset() {
/* 
	Resets the test structure peak detector
*/
	digitalWrite(pin_rst_test, HIGH);
	delayMicroseconds(50);
	digitalWrite(pin_rst_time, LOW);
}


/* ------------- */
/* --- Other --- */
/* ------------- */
void serialEvent() {
/* 
	SerialEvent occurs whenever new data comes in the hardware serial RX. This
	routine is run between each time loop() runs, so using delay inside loop
	can delay response. Multiple bytes of data may be available.
*/
	if (Serial.available()) {
		char inChar = (char)Serial.read();
		inputString += inChar;

		// String terminates with \n
		if (inChar == '\n') {
			stringComplete = true;
		}
	}
}