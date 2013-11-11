
/* Purpose of this program is to create templates & code that can later be merged with the other sweep/radio control */
/* Author: Daniel Arnitz */

/* Implemented: Unified SPI interface, SPI device interfaces for PLL, DAC, and switch controller. */


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

/* SWEEP  */
/*    basics (other PLL config parameters can be found in pll_adf40120_freq2regval()) */
#define MAX_NUM_FREQ 201 // maximum number of frequencies
#define PLL_RF_INPUT_FREQ 100 // RF Input Frequency in MHz
/*    timing parameters (approximate; plus execution time) */
#define PULSE_DURATION 100 // sync signaling pulse duration in microseconds (must be smaller than DWELL_TIME)
#define DWELL_TIME 1000 // dwell time at each step in microseconds
#define SWEEP_PAUSE 2500  // pause between sweeps in microseconds
#define LONG_PAUSE 200 // pause in milliseconds (if needed)

/* SERIAL INTERFACE */
#define SERIAL_INBUFSIZE 101 // bytes serial input buffer 

/* GLOBAL VARIABLES */
/*    frequency sweep / PLL control */
int PFDFrequency 1250 // PFD Frequency in MHz ("channel spacing")
int num_freq = 0; // actual number of frequencies
byte F2 = 0; // PLL 24-bit function register, [F2,F1,F0]
byte F1 = 0; // PLL 24-bit function register, [F2,F1,F0]
byte F0 = 0; // PLL 24-bit function register, [F2,F1,F0]
byte R2 = 0b10010001; // PLL 24-bit R counter, [R2,R1,R0] (fixed/reserved)
byte R1[MAX_NUM_FREQ] = {}; // PLL 14-bit R counter, [R2,R1,R0]
byte R0[MAX_NUM_FREQ] = {}; // PLL 14-bit R counter, [R2,R1,R0]
byte N2[MAX_NUM_FREQ] = {}; // PLL 19-bit N counter, [N2,N1,N0]
byte N1[MAX_NUM_FREQ] = {}; // PLL 19-bit N counter, [N2,N1,N0]
byte N0[MAX_NUM_FREQ] = {}; // PLL 19-bit N counter, [N2,N1,N0]
/*    serial interace */
int serial_inputbuffer[SERIAL_INBUFSIZE] = {};




//############################################################
// MAIN SETUP
//############################################################
void setup() {
  
  // Initialize serial port (debugging and control)
  Serial.begin(9600);
  
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
 
}


//############################################################
// MAIN LOOP
//############################################################
void loop() {
  
  /* SWITCH CONTROL */
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
void set_linear_sweep(int fstart, int fstep, int fstop) {
  // reset number of frequency values
  num_freq = 0;
  // populate frequency vector (PLL register values)
  for (int curr_freq = fstart; curr_freq <= fstop; curr_freq += fstep) {
    pll_adf40120_addfrequency(curr_freq, num_freq);
    num_freq++;
  } 
}
// add another frequency step to the sweep
void add_sweep_frequency(int freq) {
  pll_adf40120_addfrequency(freq, num_freq);
  num_freq++;
}
// reset sweep
void reset_sweep() {
  num_freq = 0;
}


//############################################################
// SERIAL CONTROL INTERFACE
//############################################################



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
// TODO: IMPLEMENT
void pll_adf40120_freq2regval(int RFout, int i) {

  // device configuration 
  int  Prescaler = 1; // prescaler setting (16/17)
  byte CPsetting1 = 3; // charge pump 1 setting
  byte CPsetting2 = 3; // charge pump 2 setting
  byte CPGain = 0; // charge pump gain
  byte CP3state = 0;
  byte Fastlock = 0;
  byte Timeout = 0;
  byte PDPolarity = 0;
  byte CounterReset = 0;
  byte LDP = 0; // lock detect precision
  byte Powerdown = 0;
  byte ABPW = 0;
  byte Sync = 0;
  byte Delay = 0;
  byte Muxout = 0;
  byte Testmodes = 1;

  // calculate P, R, N, B, & A values for calculating register
  int P = pow2(Prescaler) << 3;
  int R = REFin * 1000 / PFDFreq); // kHz -> Hz
  int N = RFout *  250 / PFDFreq); // kHz -> Hz plus /4 for channel spacing
  int B = N / P;
  int A = N - (B * P);
 
  // modify fastlock and powerdown settings
  if (Fastlock==2) Fastlock++;
  if (Powerdown==2) Powerdown++;
  
  // calculate register values
  //    R-counter (23-16: fixed,  15-2: R-Counter,  1-0: 00)
  //    N-counter (23-22: 00, 21: CP gain,  20-8: B-Counter,  7-2: A-Counter,  1-0: 01)
  //    Function latch
  
   
  // Calculate the register values
  Reg[0] = ( pow2(23) + pow2(20) + Testmodes*pow2(16) + (R & 0x3FFF) * pow2(2) );
  Reg[1] = ( CPGain*pow2(21) + (B&0x1FFF)*pow2(8) + (A&0x3F)*pow2(2) + 1);
  Reg[2] = ( Prescaler*pow2(22) + CPsetting2*pow2(18) + CPsetting1*pow2(15) + 
             Timeout*pow2(11) + Fastlock*pow2(9) + CP3state*pow2(8) + 
             PDPolarity*pow2(7) + Muxout * pow2(4) + Powerdown*pow2(3) + CounterReset*pow2(2) + 2);
  
  // N0 is the low byte
  N0[i] = (byte)lowByte((long)Reg[1]);
    
  // Shift the value returned by calcRegisters by 8 bits to get the N1, middle byte
  N1[i] = (byte)lowByte((long)Reg[1] >> 8);
  
  // If this is the first iteration, the RCounter and Function Latch should be set
  if( i == 0){
     R0 = (byte)(lowByte((long)Reg[0])); // The LSB of the R Latch
     R1 = (byte)(lowByte((long)Reg[0] >> 8)); // The middle byte of the R Latch
     R2 = (byte)(lowByte((long)Reg[0] >> 16)); // The MSB of the R Latch

     F0 = (byte)(lowByte((long)Reg[2])); // The LSB of the Function Latch
     F1 = (byte)(lowByte((long)Reg[2] >> 8)); // The middle byte of the Function Latch
     F2 = (byte)(lowByte((long)Reg[2] >> 16)); // The MSB of the Function Latch
  }
  
  // R0A is the low byte of the R Latch
  R0A[i] = (byte)(lowByte((long)Reg[0]));
  
  // R1A is the middle byte of the R Latch
  R1A[i] = (byte)(lowByte((long)Reg[0] >> 8));
  
  // R1A is the high byte of the R Latch
  R2A[i] = (byte)(lowByte((long)Reg[0] >> 16));
    
}


// fast implementation for "power of two"
int pow2(byte value){
  return (int 1 << value)
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
