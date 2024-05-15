/*
 * Functions for interfacing with the analog scan chain on chip.
*/
#include <Arduino.h>

// Constants
const int N_SCAN = 207;     // Number of scan bits
const int CLK_PERIOD = 200; // Clock period in micro-second

void atick(int pin_scan_clk) {
/*
 * 200 us -> 5kHz
 * 50% duty cycle.
 * Note:  The other CLK_PERIOD / 4 is included the code body (Line 68).
 *        It is added there to prevent set-up/hold time violations.
*/
  digitalWrite(pin_scan_clk, LOW);
  delayMicroseconds(CLK_PERIOD / 2);

  // Read
  digitalWrite(pin_scan_clk, HIGH);
  delayMicroseconds(CLK_PERIOD / 4);
} // end atick()

void asc_write(int pin_scan_clk, int pin_scan_inb) {
/*
 * Writes the data into the on-chip shift register.
 * NB: The inversion is handled internally! That is, if bits=000
 * or bitsb=111, the input should be 000 and code here will feed in 111.
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

  // Initilize the clock
  /* Note:  Clocking occurs on rising edges
   * 
   * clk: ____|‾‾|__|‾‾|__|‾‾|__|‾‾
   * din: __|‾‾|__|‾‾|__|‾‾|__|‾‾|_
   */
  
  digitalWrite(pin_scan_clk, HIGH);
  delayMicroseconds(100);

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
    
    // To prevent set-up/hold time violations
    delayMicroseconds(CLK_PERIOD / 4);

    // Pulse clock
    atick(pin_scan_clk);
  }
  Serial.println("ASC write complete");
} // end asc_write()

void asc_load(int pin_scan_loadb) {
/*
 * Latches the loaded data to the outputs of the scan chain.
 * You must call asc_write() before calling this!
*/
  Serial.println("Executing ASC load");
  digitalWrite(pin_scan_loadb, LOW);
  digitalWrite(pin_scan_loadb, HIGH);
} // end asc_write()

void asc_read(int pin_scan_clk, int pin_scan_outb) {
/*
 * Reads data from the single pin on the output of the scan chain. Sends
 * the data (not datab) back through to the serial.
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
  for (int i=1; i<N_SCAN; i=i+1) {
    atick(pin_scan_clk);
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
} // end asc_read()
