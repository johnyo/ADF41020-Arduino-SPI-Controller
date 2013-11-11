
/* Purpose of this program is to create templates & code that can later be merged with the other sweep/radio control */
/* Author: Daniel Arnitz */

/* Implemented: Unified SPI interface, SPI device interfaces for PLL, DAC, and switch controller, Serial command interface. */

// Libraries
#include <SPI.h>
#include <SoftwareSerial.h>

/* SPI TIMING AND INTERFACE PARAMETERS */
#define SPI_BITORDER_DEFAULT MSBFIRST // everything assumes MSB first; do not change without modifying the code!
/*    clock modess */
#define SPI_MODE_DEFAULT SPI_MODE0 // default: clock in at rising edge; clock is low when idle (for AD41020, MCP4811)
#define SPI_MODE_TLE723X SPI_MODE1 // clock in at falling edge; clock is low when idle (for TLE723x)
/*    clock and timing parameters */
#define SPI_CLOCK_DEFAULT SPI_CLOCK_DIV2 // default: 8 MHz (for AD41020, MCP4811)
#define SPI_CLOCK_TLE723X SPI_CLOCK_DIV4 // 4 MHz for TLE723x
/*    binary masks for chip enable - PORTD (7:PLL, 5:DAC, 4:switch-driver-chain) */
#define SPI_CE_MASK_PLL 0b10000000 // PLL controller
#define SPI_CE_MASK_DAC 0b00100000 // sync signal DAC
#define SPI_CE_MASK_DRV 0b00010000 // switch driver chain

/* OTHER BINARY MASKS */
#define SIG_SS_MASK 0b01000000 // start sweep signal

/* DAC sync line values (10 bit = 1024 max) */
#define SYNC_SIGNAL_STARTPANEL 1024 // start of new panel
#define SYNC_SIGNAL_STARTSWEEP 512 // start of sweep
#define SYNC_SIGNAL_STARTSTEP 256 // start of step
#define SYNC_SIGNAL_RESET 0 // default

/* SERIAL COMMAND INTERFACE */
#define SERIAL_EOL '\n' // end-of-line terminator for serial interface
#define SERIAL_SEP_C2V ' ' // separator between command and value(s)
#define SERIAL_SEP_V2V ',' // separator between two values
#define SERIAL_INBUF_SIZE 1001 // bytes serial input buffer

/* SWEEP  */
/*    basics (other PLL config parameters can be found in pll_adf40120_freq2regval()) */
#define MAX_NUM_FREQ 201 // maximum number of frequencies
#define PLL_RF_INPUT_FREQ 100 // RF Input Frequency in MHz
/*    timing parameters (approximate; plus execution time) */
#define PULSE_DURATION 100 // sync signaling pulse duration in microseconds (must be smaller than DWELL_TIME)
#define DWELL_TIME 1000 // dwell time at each step in microseconds
#define SWEEP_PAUSE 2500  // pause between sweeps in microseconds
#define LONG_PAUSE 200 // pause in milliseconds (if needed)

/* HANDSHAKE / ERRORS */
#define HALT_IF_ERROR false // stop execution in case of errors
#define SERIAL_HANDSHAKE_OK "OK -- " // prefix for "OK"
#define SERIAL_HANDSHAKE_ERR "ERROR -- " // prefix for "error"

/* GLOBAL VARIABLES */
/*    frequency sweep / PLL control */
long PFDFreq = 1250; // PFD Frequency in MHz ("channel spacing")
int  num_freq = 0; // actual number of frequencies
byte F2 = 0; // PLL 24-bit function register, [F2,F1,F0]
byte F1 = 0; // PLL 24-bit function register, [F2,F1,F0]
byte F0 = 0; // PLL 24-bit function register, [F2,F1,F0]
byte R2 = 0b10010001; // PLL 24-bit R counter, [R2,R1,R0] (fixed/reserved)
byte R1[MAX_NUM_FREQ] = {}; // PLL 14-bit R counter, [R2,R1,R0]
byte R0[MAX_NUM_FREQ] = {}; // PLL 14-bit R counter, [R2,R1,R0]
byte N2[MAX_NUM_FREQ] = {}; // PLL 19-bit N counter, [N2,N1,N0]
byte N1[MAX_NUM_FREQ] = {}; // PLL 19-bit N counter, [N2,N1,N0]
byte N0[MAX_NUM_FREQ] = {}; // PLL 19-bit N counter, [N2,N1,N0]
/*    serial command interace */
String  serial_cmd = ""; //
int     serial_val = 0; // 
String  serial_inbuf = "";  // serial input buffer (CMD VAL1,VAL2,...)
boolean serial_inbuf_complete = false; // command complete (EOL reached)?



//############################################################
// MAIN SETUP
//############################################################
void setup() {
  
  // Initialize serial port (debugging and control)
  Serial.begin(9600);
  // Reserve memory for serial input string buffer
  serial_inbuf.reserve(SERIAL_INBUF_SIZE);
  // Reset command interface
  serial_reset_cmd_interface();
  
  // Initialize digital outputs
  //    Define outputs on port D
  DDRD = DDRD | SPI_CE_MASK_PLL | SPI_CE_MASK_DAC | SPI_CE_MASK_DRV | SIG_SS_MASK;
  //    Initialize outputs
  PORTD = 0b00000000;
  
  // Initialize SPI interface
  SPI.begin();
  //   Set default bit order
  SPI.setBitOrder(SPI_BITORDER_DEFAULT); 
  //   Set default data mode
  SPI.setDataMode(SPI_MODE_DEFAULT);
  //   Set the speed we want to transmit at. This divides
  //   the 16 MHz system clock. Default is divide by 4.
  SPI.setClockDivider(SPI_CLOCK_DEFAULT);
  
  // Initialize PLL
  //Serial.println("--------------------------------------------------------");
  //pll_set_linear_sweep(10415, 45, 13070);
}


//############################################################
// MAIN LOOP
//############################################################
void loop() {
  
  /* SERIAL CONTROL INTERFACE */
  //    get serial data
  if (Serial.available() > 0){
    serial_input();
  }
  //    if command is complete: parse
  if (serial_inbuf_complete){
    // decode command and reset interface
    serial_command_decode();
  }
  
  /* SWITCH CONTROL */
  /*
  // initial state of all switches in the chain
  byte switch_states[] = {0, 0, 0}; 
  // set initial state
  spi_drv_tle723x_set(switch_states, sizeof(switch_states));
  spi_dac_mcp4811(switch_states[0] << 4); // also output state on DAC
  delay(LONG_PAUSE);
  // loop through switch states 1...6
  for(byte i = 0; i < 6; i++) {
    // define patterns for each switch
    switch_states[0] = 0b00000001 << i; // counts up
    switch_states[1] = 0b00100000 >> i; // counts down
    switch_states[2] = switch_states[2] | switch_states[0]; // fills from zero
    // change states
    spi_drv_tle723x_set(switch_states, sizeof(switch_states));
    spi_dac_mcp4811(switch_states[0] << 4); // also output state on DAC
    // wait [ms]
    delay(LONG_PAUSE);
  }
  */
  
  /* PLL CONTROL, 101-point sweep */
  /*
  for(int i = 0; i < 101; i++) {
    // program PLL
    spi_pll_adf40120(0, 0, (byte)i); // ### REPLACE DUMMY VALUES BY REAL DATA
    // sync signal
    if(i == 0)
      pulse_sync_signal(SYNC_SIGNAL_STARTSWEEP); // new sweep
    else
      pulse_sync_signal(SYNC_SIGNAL_STARTSTEP); // new step
    // wait for step dwell time
    delayMicroseconds(DWELL_TIME - PULSE_DURATION);
  }
  // pause after sweep
  delayMicroseconds(SWEEP_PAUSE);
  */
  
  
  /* DAC TEST */
  /*
  for(int i = 0; i < 1024; i++) {
    spi_dac_mcp4811(i);
    delay(1);
  }
  spi_dac_mcp4811(0);
  delay(1000);
  */
}



//############################################################
// FREQUENCY SWEEP FUNCTIONS
//############################################################
// populate frequency vector; linear sweep
void pll_set_linear_sweep(int fstart, int fstep, int fstop) {
  // reset number of frequency values
  num_freq = 0;
  // populate frequency vector (PLL register values)
  for (int curr_freq = fstart; curr_freq <= fstop; curr_freq += fstep) {
    pll_adf40120_addfreq(curr_freq, num_freq);
    num_freq++;
  } 
}
// add another frequency step to the sweep
void pll_add_sweep_frequency(int freq) {
  pll_adf40120_addfreq(freq, num_freq);
  num_freq++;
}
// reset sweep
void pll_reset_sweep() {
  num_freq = 0;
}


//############################################################
// SERIAL CONTROL INTERFACE
//############################################################
// parse string and return the next token (command string - whitespace - comma separated list of values)
//    returns true if a new token was found; false if not
void serial_command_decode() {
  // get command from buffer
  serial_parse_next_token();
  serial_cmd.toUpperCase();
  // decode command
  //     update PFD frequency
  if (serial_cmd == "PLL:PFD") { 
    pll_reset_sweep(); // reset sweep (register values need to be recalculated)
    serial_parse_next_token(); // get value from buffer
    PFDFreq = serial_val; // update PFD frequency
    // send feedback
    Serial.print(SERIAL_HANDSHAKE_OK + serial_cmd);
    Serial.print(" "); Serial.println(PFDFreq);
  }
  //     linear sweep [min:step:max]
  else if (serial_cmd == "PLL:SWEEP:LIN") { 
    pll_reset_sweep(); // reset sweep (register values need to be recalculated)
    // get start, step, stop values from buffer
    int fstart = 0;
    int fstep = 0;
    int fstop = 0;
    if (serial_parse_next_token()) {
      fstart = serial_val;
    } else {
      Serial.println("Start value for PLL:SWEEP:LIN missing. Need PLL:SWEEP:LIST <start>,<step>,<stop>");
    }
    if (serial_parse_next_token()) {
      fstep = serial_val;
    } else {
      Serial.println("Step value for PLL:SWEEP:LIN missing. Need PLL:SWEEP:LIST <start>,<step>,<stop>");
    }
    if (serial_parse_next_token()) {
      fstop = serial_val;
    } else {
      Serial.println("Stop value for PLL:SWEEP:LIN missing. Need PLL:SWEEP:LIST <start>,<step>,<stop>");
    }
    // send feedback
    Serial.print(SERIAL_HANDSHAKE_OK + serial_cmd);
    Serial.print(" "); Serial.print(fstart); 
    Serial.print(":"); Serial.print(fstep); 
    Serial.print(":"); Serial.println(fstop);
    // program sweep
    pll_set_linear_sweep(fstart, fstep, fstop);
  }
  //     program new list sweep
  else if (serial_cmd == "PLL:SWEEP:LIST") { 
    pll_reset_sweep(); // reset old sweep
    // send feedback
    Serial.println(SERIAL_HANDSHAKE_OK + serial_cmd);
    // update sweep
    while (serial_parse_next_token()) { // while there are new values in the buffer ...
      pll_add_sweep_frequency(serial_val); // ... add them to the sweep
    }
  }
  //    get number of frequencies in sweep
  else if (serial_cmd == "PLL:GET:NUM_FREQ") {
    Serial.println(num_freq);
  } 
  //    get PLL PFD frequency value
  else if (serial_cmd == "PLL:GET:PFD") {
    Serial.println(PFDFreq);
  } 
  //    get PLL register values
  else if (serial_cmd == "PLL:GET:REGVALS") {
    for(int i = 0; i < num_freq; i++) {
      Serial.print("|");
      Serial.print(((long)F2)    << 16 | ((long)F1)    << 8 | (long)F0, HEX);
      Serial.print("|");
      Serial.print(((long)R2)    << 16 | ((long)R1[i]) << 8 | (long)R0[i], HEX);
      Serial.print("|");
      Serial.print(((long)N2[i]) << 16 | ((long)N1[i]) << 8 | (long)N0[i], HEX);
      Serial.println(" ");
    }  
  } 
  //    unrecognized command; throw an error
  else {
    Serial.print(SERIAL_HANDSHAKE_ERR);
    Serial.println("Unrecognized command \"" + serial_cmd + "\". Resetting command buffer.");
  }
  // reset command buffer
  serial_reset_cmd_interface();
}
//############################################################
// parse string and return the next token (command string - whitespace - comma separated list of values)
//    returns true if a new token was found; false if not
boolean serial_parse_next_token() {
  // split index in string
  int ind_sep = 0;
  // check if buffer is already empty
  if (serial_inbuf.length() == 0){
    return false;
  }
  //Serial.println("------------------------------------");
  //Serial.println("BUFFER: " + serial_inbuf);
  // next token is a command
  if (serial_cmd.length() == 0) {
    ind_sep = serial_inbuf.indexOf(SERIAL_SEP_C2V); // find command-value separator
    serial_cmd = serial_inbuf.substring(0,ind_sep); // -> command string
    //Serial.println("COMMAND: " + serial_cmd);
  }
  // next token is a value (command already decoded)
  else {
    ind_sep = serial_inbuf.indexOf(SERIAL_SEP_V2V); // find value-value separator
    if(ind_sep < 0){ // last value might not be terminated with SERIAL_SEP_V2V
      ind_sep = serial_inbuf.length();
    }
    serial_val = (serial_inbuf.substring(0,ind_sep)).toInt(); // -> integer
    //Serial.print("VALUE: "); Serial.println(serial_val);
  }
  // remove decoded token plus separator; get rid of any whitespaces
  serial_inbuf = serial_inbuf.substring(ind_sep+1);
  serial_inbuf.trim();
  //Serial.println("REMAINING BUFFER: " + serial_inbuf);
  return true;
}
//############################################################
// write serial input to buffer (copied from SerialEvent example)
void serial_input() {
  while (Serial.available()) {
    // get the new character
    char inChar = (char)Serial.read(); 
    // if the incoming character is the EOL character...
    if (inChar == SERIAL_EOL) {
      // make sure we don't have any trailing/leading whitespaces
      serial_inbuf.trim();
      // and set the completion flag
      serial_inbuf_complete = true;
    }
    else
    {
      serial_inbuf += inChar; // otherwise, add the new character to the input buffer
    } 
  }
}
//############################################################
// reset command interface
void serial_reset_cmd_interface() {
 serial_cmd   = "";
 serial_val   = 0;
 serial_inbuf = "";
 serial_inbuf_complete = false;
}


//############################################################
// SPECIALIZED DEVICE CONTROL FUNCTIONS
//############################################################
// sends a pulse signal on the sync/handshake line (DAC)
void pulse_sync_signal(int signal) {
  // change the state to SIGNAL
  spi_dac_mcp4811(signal);
  // wait for pre-defined pulse duration
  delayMicroseconds(PULSE_DURATION);
  // reset the state
  spi_dac_mcp4811(SYNC_SIGNAL_RESET);
}  
// calculate function register value F (global), R-, and N-counter values (returned [R, N]) for ADF41020 PLL
// (copied and modified from ADF41020 host software)
// TODO: FIND OUT WHY SOME SETTINGS ARE UNUSED
void pll_adf40120_addfreq(int RFout, int i) {
  // device configuration; please refer to ADF41020 datasheet
  int  Prescaler = 1; // prescaler value (00: 8/9, 16/17, 32/33, 11:64/65)
  byte CPsetting1 = 3; // charge pump 1 setting (see manual)
  byte CPsetting2 = 3; // charge pump 2 setting (see manual)
  byte CPGain = 0; // charge pump gain (see manual)
  byte CP3state = 0; // charge pump three-state (0: normal, 1: three-state)
  byte Fastlock = 0; // fastlock setting (see manual)
  byte Timeout = 0; // timeout (PFD cycles; 3,7,11,13 ...)
  byte PDPolarity = 0; // phase detector polarity (0:neg, 1:pos)
  byte CounterReset = 0; // 0: normal, 1: R,A,B held in reset
  //byte LDP = 0; // lock detect precision (see manual)
  byte Powerdown = 0; // power down (see manual)
  //byte ABPW = 0;
  //byte Sync = 0;
  //byte Delay = 0; // 
  byte Muxout = 0; // multiplexer output (see manual)
  //byte Testmodes = 1;
  
  // calculate P, R, N, B, & A values for calculating register 
  // (typecast to long for multiplication / division; multiplication first to avoid rounding errors)   
  int  P = (1 << Prescaler) << 3; // 1 << x = 2^x, x << 3 = x * 8
  int  R = (int)( ((long)PLL_RF_INPUT_FREQ * 1000) / PFDFreq ); // kHz -> Hz
  int  N = (int)( ((long)RFout * 250) / PFDFreq ); // kHz -> Hz plus /4 for channel spacing
  int  B = N / P;
  byte A = (byte)( N - (B * P) );
 
  // modify fastlock and powerdown settings
  if (Fastlock==2) Fastlock++;
  if (Powerdown==2) Powerdown++;
  
  // calculate register values
  //    R-counter (23-16: fixed,  15-2: R-Counter,  1-0: 00)
  R1[i] = (byte)(R >> 6);
  R0[i] = (byte)(R << 2) | 0x0;
  //    N-counter (23-22: 00, 21: CP gain,  20-8: B-Counter,  7-2: A-Counter,  1-0: 01)
  N2[i] = 0b00111111 & (CPGain << 5 | (byte)(B >> 8));
  N1[i] = (byte)B;
  N0[i] = ((A&0x3F) << 2) | 0x1;
  //    Function latch
  F2 = (byte)( Prescaler << 6 | (Powerdown & 0x2) << 5 | CPsetting1 << 2 | CPsetting2 >> 1 );
  F1 = (byte)( CPsetting2 << 7 | Timeout << 3 | Fastlock << 1 | CP3state);
  F0 = (byte)( PDPolarity << 7 | Muxout << 4 | (Powerdown & 0x1) << 3 | CounterReset << 2) | 0x2;
  
  // Debug output
  /*
  Serial.print(RFout);
  Serial.print("|");
  Serial.print(((long)F2)    << 16 | ((long)F1)    << 8 | (long)F0, HEX);
  Serial.print("|");
  Serial.print(((long)R2)    << 16 | ((long)R1[i]) << 8 | (long)R0[i], HEX);
  Serial.print("|");
  Serial.print(((long)N2[i]) << 16 | ((long)N1[i]) << 8 | (long)N0[i], HEX);
  Serial.println(" ");
  */
}



//############################################################
// DEVICE-SPECIFIC SPI WRITE FUNCTIONS
//############################################################
// MCP4811 10-bit DAC (DEFAULT SPI MODES)
void spi_dac_mcp4811(int value) {
  // prepare data
  //   bit 15: 0
  //   bit 14: 0 (don't care)
  //   bit 13: 1 = 0 to 2V,  0 = 0 to 4 V
  //   bit 12: 1 = normal operation, 0 = shutdown
  //   bit 11-2: 10-bit value
  //   bit 1-0: ignored
  value = ((value << 2) & 0b0001111111111100) | 0b0001000000000000;
  // set active low chip enable
  PORTD = PORTD & (~SPI_CE_MASK_DAC);
  // transfer data
  spi_write_int(value);
  // reset active low chip enable
  PORTD = PORTD | SPI_CE_MASK_DAC;
}
//############################################################
// ADF41020 PLL (DEFAULT SPI MODES)
void spi_pll_adf40120(byte freg, byte rreg, byte nreg) {
  // set active low chip enable
  PORTD = PORTD & (~SPI_CE_MASK_PLL);
  // transfer data
  SPI.transfer(freg);
  SPI.transfer(rreg);
  SPI.transfer(nreg);
  // reset active low chip enable
  PORTD = PORTD | SPI_CE_MASK_PLL;
}
//############################################################
// TLE723x switch driver chain, 16 bits/device (USES NON-DEFAULT SPI MODES)
//   bit 15-14: command (00: diagnostics, 01: read, 10: reset, 11: write)
//   bit 13-11: 0
//   bit 10- 8: address (CTL: 111)
//   bit  7- 0: data / state 
void spi_drv_tle723x_set(byte *state, size_t num_dev) {
  // prepare command
  byte cmd = 0b11000111; // write, CTL
  // set active low chip enable
  PORTD = PORTD & (~SPI_CE_MASK_DRV);
  // change SPI timing to what the TLE723x expects (this takes about 2-3us)
  SPI.setDataMode(SPI_MODE_TLE723X);
  SPI.setClockDivider(SPI_CLOCK_TLE723X);
  // transfer data for entire chain
  for(int i = 0; i < num_dev; i++) {
    SPI.transfer(cmd);
    SPI.transfer(state[i]);
  }
  // revert SPI timing to defaults (this takes about 2-3us)
  SPI.setDataMode(SPI_MODE_DEFAULT);
  SPI.setClockDivider(SPI_CLOCK_DEFAULT);
  // reset active low chip enable
  PORTD = PORTD | SPI_CE_MASK_DRV;
}


//############################################################
// GENERIC SPI WRITE FUNCTIONS (MSB FIRST)
//############################################################
// 1 byte
void spi_write_8(byte b7to0) {
  SPI.transfer(b7to0);
}
//############################################################
// 2 bytes
void spi_write_16(byte b15to8, byte b7to0) {
  // transfer data (MSB first)
  SPI.transfer(b15to8);
  SPI.transfer(b7to0);
}
//############################################################
// 24 bit (3 bytes)
void spi_write_16(byte b23to16, byte b15to8, byte b7to0) {
  // transfer data (MSB first)
  SPI.transfer(b23to16);
  SPI.transfer(b15to8);
  SPI.transfer(b7to0);
}
//############################################################
// integer (2 bytes)
void spi_write_int(int data) {
  byte b15to8 = highByte(data);
  byte b7to0  =  lowByte(data);
  spi_write_16(b15to8, b7to0);
}
