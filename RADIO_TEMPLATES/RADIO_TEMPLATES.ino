
/* Purpose of this program is to create templates & code that can later be merged with the other sweep/radio control */
/*    Pinout designed for Arduino UNO + Shields */
/* Author: Daniel Arnitz */

/* Implemented: Unified SPI interface, SPI device interfaces for PLL, DAC, and switch controller, Serial command interface. */
/* Testing/Incomplete: Ethernet (Not enough SRAM) */
/* TODO: allow dwell times lo */


/* *************************************/
/* IMPORTANT -- IMPORTANT -- IMPORTANT */
/* *************************************
  Due to the limited SRAM size, string constants - in particular in Serial.println("YOUR STRING") - should be enclosed 
  by an F() function. This function will keep the string from getting copied into SRAM:
  http://learn.adafruit.com/memories-of-an-arduino/optimizing-sram
  
  If the code fails with unexplicable and seemingly random errors, check for SRAM problems! In this code, for example,
  serial_inbuf is one of the prime candidates for corruption due to this problem.
*/


// Libraries
#include <SPI.h>
#include <SoftwareSerial.h>
//#include <Ethernet.h>

/* SPI TIMING AND INTERFACE PARAMETERS */
#define SPI_BITORDER_DEFAULT MSBFIRST // everything assumes MSB first; do not change without modifying the code!
/*    clock modess */
#define SPI_MODE_DEFAULT SPI_MODE0 // default: clock in at rising edge; clock is low when idle (for AD41020, MCP4811)
#define SPI_MODE_TLE723X SPI_MODE1 // clock in at falling edge; clock is low when idle (for TLE723x)
/*    clock and timing parameters */
#define SPI_CLOCK_DEFAULT SPI_CLOCK_DIV2 // default: 8 MHz (for AD41020, MCP4811)
#define SPI_CLOCK_TLE723X SPI_CLOCK_DIV4 // 4 MHz for TLE723x

///* DIGITAL PORTS - ARDUINO UNO, v3a * /
///*    masks for chip enable */
//#define SPI_CE_DDR DDRD // data direction register
//#define SPI_CE_PORT PORTD // port
//#define SPI_CE_MASK_PLL 0b10000000 // PLL controller
//#define SPI_CE_MASK_SWI 0b00001000 // switch driver chain
//#define SPI_CE_MASK_DAC 0b00100000 // sync signal DAC
///*    masks for other digital signals */
//#define DIG_SIGNAL_DDR DDRD // data direction register
//#define DIG_SIGNAL_PORT PORTD // port
//#define DIG_SIGNAL_RSWI 0b00000100 // reset switch chain
//#define DIG_SIGNAL_TRIG 0b01000000 // sweep trigger signal for sampling

/* DIGITAL PORTS - ARDUINO UNO v3b (COMPATIBLE WITH MEGA 2560)  */
/*    masks for chip enable */
#define SPI_CE_DDR DDRC // data direction register
#define SPI_CE_PORT PORTC // port
//                        76543210
#define SPI_CE_MASK_PLL 0b00000010 // PLL controller
#define SPI_CE_MASK_DAC 0b00000100 // sync signal DAC
#define SPI_CE_MASK_SWI 0b00001000 // switch driver chain
/*    masks for other digital signals */
#define DIG_SIGNAL_DDR DDRC // data direction register
#define DIG_SIGNAL_PORT PORTC // port
//                        76543210
#define DIG_SIGNAL_MRST 0b00000001 // master reset (wired into the reset circuit)
#define DIG_SIGNAL_RSWI 0b00010000 // reset switch chain
#define DIG_SIGNAL_TRIG 0b00100000 // sweep trigger signal for sampling

///* DIGITAL PORTS - ARDUINO MEGA v3b  */
///*    masks for chip enable */
//#define SPI_CE_DDR DDRF // data direction register
//#define SPI_CE_PORT PORTF // port
////                        76543210
//#define SPI_CE_MASK_PLL 0b00000010 // PLL controller
//#define SPI_CE_MASK_DAC 0b00000100 // sync signal DAC
//#define SPI_CE_MASK_SWI 0b00001000 // switch driver chain
///*    masks for other digital signals */
//#define DIG_SIGNAL_DDR DDRF // data direction register
//#define DIG_SIGNAL_PORT PORTF // port
////                        76543210
//#define DIG_SIGNAL_MRST 0b00000001 // master reset (wired into the reset circuit)
//#define DIG_SIGNAL_RSWI 0b00010000 // reset switch chain
//#define DIG_SIGNAL_TRIG 0b00100000 // sweep trigger signal for sampling


/* DAC sync line values (10 bit = 1023 max \approx 4V) */
#define SYNC_SIGNAL_STARTMEAS 1023 // start of entire measurement/loop
#define SYNC_SIGNAL_STARTSWEEP 767 // start of sweep
#define SYNC_SIGNAL_STARTSTEP 512 // start of step
#define SYNC_SIGNAL_RESET 0 // default

/* SWITCHES */
#define SWITCH_NUM_PORTS 6 // number of ports per switch (default)
#define SWITCH_NUM_SWITCHES 12 // MAXIMUM number of switches
#define SWITCH_STATE_OFF 0b01000000 // "off" state; needed for Agilent switches - port 7 wired to "reset"

/* SWEEP / PLL  */
/*    basics (other PLL config parameters can be found in pll_adf40120_addfreq) */
#define MAX_NUM_FREQ 101 // maximum number of frequencies
#define PLL_RF_INPUT_FREQ 100 // RF Input Frequency in MHz
/*    timing parameters (approximate; plus execution time) */
#define DWELL_TIME 1000 // default dwell time in microseconds
#define DWELL_TIME_EXEC_CORR 33 // microseconds correction for dwell time to account for program delay
#define SWEEP_PAUSE 15000  // default pause between sweeps in microseconds (e.g, for switching time)
#define TRIG_PAUSE 500 // pause after sending the trigger signal and before starting a sweep in microseconds
#define LONG_PAUSE 200 // pause in milliseconds (if needed)
/*    default sweep (linear) */
#define SWEEP_DEFAULT_PFDFREQ 1250 // kHz default PFD Frequency
#define SWEEP_DEFAULT_FSTART 10415 // MHz start frequency
#define SWEEP_DEFAULT_FSTEP     45 // MHz frequency step
#define SWEEP_DEFAULT_FSTOP  13070 // MHz stop frequency

/* SERIAL COMMAND INTERFACE */
#define SERIAL_BAUD_RATE 19200 // bit/sec on serial interface (4800,9600,14400,19200,28800,38400,57600,115200)
#define SERIAL_INBUF_SIZE MAX_NUM_FREQ*6 + 24 // bytes serial input buffer (6 char per value plus two dozen for commands)
#define SERIAL_EOL '\n' // end-of-line terminator for serial interface
#define SERIAL_SEP_C2V ' ' // separator between command and value(s)
#define SERIAL_SEP_V2V ',' // separator between two values
#define SERIAL_HANDSHAKE_OK "OK -- " // prefix for "OK"
#define SERIAL_HANDSHAKE_ERR "ERROR -- " // prefix for "error"

/* ETHERNET */
/*
byte mac[] = {0x71,0x06,0x52,0x59,0x12,0xBA}; // random MAC
IPAddress ip(192,168,1,177); // reserved on Reynoldslab network
unsigned int localPort = 5025; // standard SCPI port is 5025
//    initialize the Ethernet server
EthernetServer eth_server(localPort);
*/

/* GLOBAL VARIABLES */
/*    frequency sweep and PLL control */
unsigned long PFDFreq = SWEEP_DEFAULT_PFDFREQ; // PFD Frequency in MHz ("channel spacing")
unsigned int  num_freq = 0; // actual number of frequencies
byte F2 = 0; // PLL 24-bit function register, [F2,F1,F0]
byte F1 = 0; // PLL 24-bit function register, [F2,F1,F0]
byte F0 = 0; // PLL 24-bit function register, [F2,F1,F0]
byte R2 = 0b10010001; // PLL 24-bit R counter, [R2,R1,R0] (fixed/reserved)
byte R1 = 0b00000000; // PLL 14-bit R counter, [R2,R1,R0] (only changes w PFDFreq or PLL_RF_INPUT_FREQ => constant during sweep)
byte R0 = 0b00000000; // PLL 14-bit R counter, [R2,R1,R0] (only changes w PFDFreq or PLL_RF_INPUT_FREQ => constant during sweep)
byte N2[MAX_NUM_FREQ] = {}; // PLL 19-bit N counter, [N2,N1,N0]
byte N1[MAX_NUM_FREQ] = {}; // PLL 19-bit N counter, [N2,N1,N0]
byte N0[MAX_NUM_FREQ] = {}; // PLL 19-bit N counter, [N2,N1,N0]
/*    sweep timing */
boolean continuous_run_panels = false; // continuous run, all panels (including switches)
boolean continuous_run_frequency = false; // continuous frequency sweep (current switch only)
int sweep_dwell_time = DWELL_TIME - DWELL_TIME_EXEC_CORR; // dwell time at each step in microseconds
int sweep_pulse_time = DWELL_TIME / 2; // sync signaling pulse duration in microseconds
int sweep_pause_time = SWEEP_PAUSE; // pause time between sweeps in microseconds
/*    serial command interace */
String  serial_cmd = ""; //
int     serial_val = 0; // 
String  serial_inbuf = "";  // serial input buffer (CMD VAL1,VAL2,...)
boolean serial_inbuf_complete = false; // command complete (EOL reached)?
/*    switches */
byte switch_num_switches = SWITCH_NUM_SWITCHES; // number of switches
byte switch_num_ports = SWITCH_NUM_PORTS; // number of ports per switch
byte switch_states[SWITCH_NUM_SWITCHES] = {}; // state of all switches in chain (binary)
byte switch_port_active = 0; // currently active port in switch chain (0: all off, 1 ... #switches * #ports)
byte switch_port_mask = (1 << (switch_num_ports + 1)) - 1; // port mask (ones up to switch_num_ports, zeros otherwise)



//############################################################
// MAIN SETUP
//############################################################
void setup() {

  // Initialize serial port (debugging and control)
  Serial.begin(SERIAL_BAUD_RATE);
  // Reserve memory for serial input string buffer
  serial_inbuf.reserve(SERIAL_INBUF_SIZE);
  // Reset command interface
  serial_reset_cmd_interface();

  // Initialize digital outputs (init first; then define as outputs to avoid glitches)
  //    Chip enable signals (active low)
  SPI_CE_PORT = 0x0 | SPI_CE_MASK_PLL | SPI_CE_MASK_DAC | SPI_CE_MASK_SWI; // initialize
  SPI_CE_DDR = SPI_CE_DDR | SPI_CE_MASK_PLL | SPI_CE_MASK_DAC | SPI_CE_MASK_SWI; // set as outputs
  //    Other digital signals (master reset is active low; others are active high)
  DIG_SIGNAL_PORT = 0x0 | DIG_SIGNAL_MRST | DIG_SIGNAL_RSWI; // resets are active low; others active high
  DIG_SIGNAL_DDR = DIG_SIGNAL_DDR | DIG_SIGNAL_TRIG | DIG_SIGNAL_MRST | DIG_SIGNAL_RSWI;
  
  // Initialize SPI interface
  SPI.begin();
  //   Set default bit order
  SPI.setBitOrder(SPI_BITORDER_DEFAULT); 
  //   Set default data mode
  SPI.setDataMode(SPI_MODE_DEFAULT);
  //   Set the speed we want to transmit at. This divides
  //   the 16 MHz system clock. Default is divide by 4.
  SPI.setClockDivider(SPI_CLOCK_DEFAULT);
  
  // Start the Ethernet server
  /*
  Ethernet.begin(mac, ip);
  eth_server.begin();
  */
  
  // Initialize signaling DAC
  pulse_sync_signal(SYNC_SIGNAL_RESET);
  
  // Initialize PLL
  //     check sweep timings
  check_sweep_timing();
  //     program the standard linear sweep
  pll_set_linear_sweep(SWEEP_DEFAULT_FSTART, SWEEP_DEFAULT_FSTEP,  SWEEP_DEFAULT_FSTOP);
  //    and initialize the PLL
  pll_init();
  
  // Initialize switch chain (all off)
  switch_chain_reset();
  switch_chain_apply();
  
}



//############################################################
// MAIN LOOP
//############################################################
void loop() {

  /* SWITCHES + PLL + CONTROL SIGNALS + SERIAL CONTROL INTERFACE */
  // serial command interface
  //    if command is complete: parse, decode, and reset interface
  if (serial_inbuf_complete) {
    serial_command_decode();
  //    otherwise: check if new data is available, process if that's the case
  } else {
    if (Serial.available() > 0){
      serial_input();
    }
  }
  // continuous run modes
  if (continuous_run_panels) {
    continuous_run_frequency = false; // prevent additional sweep for last panel
    run_panel_sweep();
  }
  if (continuous_run_frequency) {
    continuous_run_panels = false; // prevent switching
    run_frequency_sweep();
    delayMicroseconds(sweep_pause_time - sweep_pulse_time - sweep_dwell_time); // inter-sweep wait
  }
}



//############################################################
// SWEEP / MEASUREMENT MASTER FUNCTIONS
//############################################################
// sweep: run panel sweep (measurement) / cycle through switches
void run_panel_sweep() {
  // cycle through all ports
  for(unsigned int i = 0; i < switch_num_switches * switch_num_ports; i++) {
    // next switch port
    if (i == 0) {
      switch_chain_init();
    } else {
      switch_chain_next();
    }    
    // wait for switches to connect
    delayMicroseconds(sweep_pause_time);
    // signal "start of measurement"
    if (i == 0) {
      pulse_sync_signal(SYNC_SIGNAL_STARTMEAS);
      delayMicroseconds(sweep_pulse_time);
    }
    // run sweep
    run_frequency_sweep();
  }
  // reset switch chain
  switch_chain_reset();
  switch_chain_apply();
  // wait for switches to (dis)connect
  delayMicroseconds(sweep_pause_time - sweep_pulse_time);
}
//############################################################
// sweep: cycle through frequencies 
void run_frequency_sweep() {
  // run sweep 
  for(unsigned int i = 0; i < num_freq; i++) {
    // program PLL
    spi_pll_adf40120(N2[i], N1[i], N0[i]);
    // sync signal
    if(i == 0)
      pulse_sync_signal(SYNC_SIGNAL_STARTSWEEP); // new sweep
    else
      pulse_sync_signal(SYNC_SIGNAL_STARTSTEP); // new step
    // wait for step dwell time
    delayMicroseconds(sweep_dwell_time - sweep_pulse_time);
   }
   // reset to first frequency and terminate last step
   spi_pll_adf40120(N2[0], N1[0], N0[0]);
   pulse_sync_signal(SYNC_SIGNAL_STARTSTEP);
   delayMicroseconds(sweep_dwell_time - sweep_pulse_time);
}



//############################################################
// FREQUENCY SWEEP FUNCTIONS
//############################################################
// populate frequency vector; linear sweep
void pll_set_linear_sweep(unsigned int fstart, unsigned int fstep, unsigned int fstop) {
  // reset number of frequencies
  num_freq = 0;
  // do a quick sanity check
  if ((fstop < fstart) || (fstep == 0) || ((fstop-fstart)/fstep + 1 > MAX_NUM_FREQ)) {
    Serial.print(SERIAL_HANDSHAKE_ERR);
    Serial.print(F("Incorrect linear sweep parameters. Ignoring."));
    return;
  }
  // populate frequency vector (PLL register values)
  for (unsigned int curr_freq = fstart; curr_freq <= fstop; curr_freq += fstep) {
    pll_adf40120_addfreq(curr_freq, num_freq);
    num_freq++;
  } 
}
//############################################################
// add another frequency step to the sweep
void pll_add_sweep_frequency(unsigned int freq) {
  pll_adf40120_addfreq(freq, num_freq);
  num_freq++;
}
//############################################################
// reset sweep
void pll_reset_sweep() {
  num_freq = 0;
}
//############################################################
// initialize new sweep
void pll_init() {
  pll_adf40120_init();
}
//############################################################
// check sweep timings
void check_sweep_timing() {
  // pulse duration and dwell time
  if (sweep_dwell_time < 0) {
    sweep_dwell_time = 0;
  }
  if (sweep_pulse_time >= sweep_dwell_time) {
    sweep_pulse_time = sweep_dwell_time >> 1;
    Serial.print(SERIAL_HANDSHAKE_ERR);
    Serial.println(F("Pulse time cannot be larger or equal to dwell time. Setting PULSE = DWELL/2."));
  }
}



//############################################################
// SERIAL CONTROL INTERFACE
//############################################################
// command decoder
void serial_command_decode() {
  // get command from buffer
  serial_parse_next_token(true);
  serial_cmd.toUpperCase();
  // decode command
  //    run frequency sweep for all panels (single)
  if (serial_cmd == "RUN:PANELS") {
    continuous_run_panels = false; // switch off continuous modes
    continuous_run_frequency = false;
    DIG_SIGNAL_PORT = DIG_SIGNAL_PORT | DIG_SIGNAL_TRIG; // set trigger signal (active high)
    delayMicroseconds(TRIG_PAUSE);
    run_panel_sweep(); // run
    DIG_SIGNAL_PORT = DIG_SIGNAL_PORT & (~DIG_SIGNAL_TRIG); // reset trigger signal (active high)
    Serial.println(SERIAL_HANDSHAKE_OK + serial_cmd); // send feedback
  }
  //    continuously run frequency sweeps for all panels
  else if (serial_cmd == "CONT:PANELS") { 
    continuous_run_panels = true; // switch on continuous mode
    Serial.println(SERIAL_HANDSHAKE_OK + serial_cmd); // send feedback
  }
  //    run frequency sweep (single)
  else if (serial_cmd == "RUN:FREQ") {
    continuous_run_panels = false; // switch off continuous modes
    continuous_run_frequency = false;
    DIG_SIGNAL_PORT = DIG_SIGNAL_PORT | DIG_SIGNAL_TRIG; // set trigger signal (active high)
    delayMicroseconds(TRIG_PAUSE);
    run_frequency_sweep(); // run
    DIG_SIGNAL_PORT = DIG_SIGNAL_PORT & (~DIG_SIGNAL_TRIG); // reset trigger signal (active high)
    Serial.println(SERIAL_HANDSHAKE_OK + serial_cmd); // send feedback
  }
  //     continuously run frequency sweeps
  else if (serial_cmd == "CONT:FREQ") { 
    continuous_run_frequency = true; // switch on continuous mode
    Serial.println(SERIAL_HANDSHAKE_OK + serial_cmd); // send feedback
  }
  //     stop continuous runs
  else if ((serial_cmd == "SINGLE") || (serial_cmd == "STOP"))  { 
    continuous_run_panels = false;
    continuous_run_frequency = false;
    Serial.println(SERIAL_HANDSHAKE_OK + serial_cmd); // send feedback
  }
  //     PFD frequency (set and query)
  else if (serial_cmd == "PLL:PFD") { 
    pll_reset_sweep(); // reset sweep (register values need to be recalculated)
    serial_parse_next_token(false); // get value from buffer
    PFDFreq = serial_val; // update PFD frequency
    Serial.print(SERIAL_HANDSHAKE_OK + serial_cmd + " "); // send feedback
    Serial.println(PFDFreq);
  }
  else if (serial_cmd == "PLL:PFD?") {
    Serial.println(PFDFreq);
  } 
  //     dwell time (set and query)
  else if (serial_cmd == "TIME:DWELL") {
    serial_parse_next_token(false); // get value from buffer
    sweep_dwell_time = serial_val - DWELL_TIME_EXEC_CORR; // update time
    check_sweep_timing(); // check new timing
    Serial.print(SERIAL_HANDSHAKE_OK + serial_cmd + " "); // send feedback
    Serial.println(sweep_dwell_time + DWELL_TIME_EXEC_CORR); // EXEC_CORR is internal; report true sweep time
  } 
  else if (serial_cmd == "TIME:DWELL?") {
    Serial.println(sweep_dwell_time + DWELL_TIME_EXEC_CORR); // EXEC_CORR is internal; report true sweep time
  } 
  //     update pause time between sweeps
  else if (serial_cmd == "TIME:PAUSE") {
    serial_parse_next_token(false); // get value from buffer
    sweep_pause_time = serial_val; // update time
    check_sweep_timing(); // check new timing
    Serial.print(SERIAL_HANDSHAKE_OK + serial_cmd + " "); // send feedback
    Serial.println(sweep_pause_time);
  } 
  else if (serial_cmd == "TIME:PAUSE?") {
    Serial.println(sweep_pause_time);
  }
  //     update sync pulse duration
  else if (serial_cmd == "TIME:PULSE") {
    serial_parse_next_token(false); // get value from buffer
    sweep_pulse_time = serial_val; // update time
    check_sweep_timing(); // check new timing
    Serial.print(SERIAL_HANDSHAKE_OK + serial_cmd + " "); // send feedback
    Serial.println(sweep_pulse_time);
  } 
  else if (serial_cmd == "TIME:PULSE?") {
    Serial.println(sweep_pulse_time);
  } 
  //     program standard sweep
  else if (serial_cmd == "SWEEP:DEFAULT") {
    pll_reset_sweep(); // reset old sweep
    pll_set_linear_sweep(SWEEP_DEFAULT_FSTART, SWEEP_DEFAULT_FSTEP,  SWEEP_DEFAULT_FSTOP); // set default
    pll_init(); /// program
    Serial.println(SERIAL_HANDSHAKE_OK + serial_cmd); // send feedback
  }
  //     program linear sweep [min:step:max]
  else if (serial_cmd == "SWEEP:LIN") { 
    pll_reset_sweep(); // reset sweep (register values need to be recalculated)
    // get start, step, stop values from buffer
    int fstart = 0;
    int fstep = 0;
    int fstop = 0;
    if (serial_parse_next_token(false)) {
      fstart = serial_val;
    } 
    else {
      Serial.println(F("Start value for PLL:SWEEP:LIN missing. Need PLL:SWEEP:LIST <start>,<step>,<stop>"));
    }
    if (serial_parse_next_token(false)) {
      fstep = serial_val;
    } 
    else {
      Serial.println(F("Step value for PLL:SWEEP:LIN missing. Need PLL:SWEEP:LIST <start>,<step>,<stop>"));
    }
    if (serial_parse_next_token(false)) {
      fstop = serial_val;
    } 
    else {
      Serial.println(F("Stop value for PLL:SWEEP:LIN missing. Need PLL:SWEEP:LIST <start>,<step>,<stop>"));
    }
    // program sweep
    pll_set_linear_sweep(fstart, fstep, fstop);
    pll_init();
    // send feedback
    Serial.print(SERIAL_HANDSHAKE_OK + serial_cmd);
    Serial.print(F(" ")); 
    Serial.print(fstart); 
    Serial.print(F(":")); 
    Serial.print(fstep); 
    Serial.print(F(":")); 
    Serial.println(fstop);
  }
  //     program new list sweep
  else if (serial_cmd == "SWEEP:LIST") { 
    pll_reset_sweep(); // reset old sweep
    // update sweep
    while (serial_parse_next_token(false)) { // while there are new values in the buffer ...
      pll_add_sweep_frequency(serial_val); // ... add them to the sweep
    }
    // initialize PLL
    pll_init();
    // send feedback
    Serial.println(SERIAL_HANDSHAKE_OK + serial_cmd);
  }
  //    set number of switches
  else if (serial_cmd == "SWITCH:NUM") {
    serial_parse_next_token(false); // get value from buffer
    if (serial_val <= SWITCH_NUM_SWITCHES) {
      switch_num_switches = serial_val; // update value
      Serial.print(SERIAL_HANDSHAKE_OK + serial_cmd + " "); // send feedback
      Serial.println(switch_num_switches);
    } else {
      Serial.print(SERIAL_HANDSHAKE_ERR); // send feedback
      Serial.print(F("Maximum number of switches is "));
      Serial.print(SWITCH_NUM_SWITCHES);
      Serial.println(F(". Increase SWITCH_NUM_SWITCHES if needed."));
    }
  } 
  else if (serial_cmd == "SWITCH:NUM?") {
    Serial.println(switch_num_switches);
  } 
  //    set number of ports per switch
  else if (serial_cmd == "SWITCH:PORTS") {
    serial_parse_next_token(false); // get value from buffer
    switch_num_ports = serial_val; // update value
    switch_port_mask = (1 << (switch_num_ports + 1)) - 1; // update port mask
    Serial.print(SERIAL_HANDSHAKE_OK + serial_cmd + " "); // send feedback
    Serial.println(switch_num_ports);
  } 
  else if (serial_cmd == "SWITCH:PORTS?") {
    Serial.println(switch_num_ports);
  } 
  //    set switch states directly
  else if (serial_cmd == "SWITCH:STATES") {
    // modify switch states
    byte i = 0;
    while (serial_parse_next_token(false) && (i < switch_num_switches)) {
      switch_states[i] = (byte)serial_val;
      i++;
    }
    // update switch chain / send feedback
    if (i == switch_num_switches) {
      switch_chain_apply();
      Serial.println(SERIAL_HANDSHAKE_OK + serial_cmd);
    } else {
      Serial.print(SERIAL_HANDSHAKE_ERR);
      Serial.println(F("Cannot update switch chain; not enough values."));
    }
  } 
  else if (serial_cmd == "SWITCH:STATES?") {
    for(byte i = 0; i < switch_num_switches - 1; i++) {
      Serial.print(switch_states[i]);
      Serial.print(F(","));
    }
    Serial.println(switch_states[switch_num_switches-1]);
  } 
  //    select specific switch port (ports # start at 1)
  else if (serial_cmd == "SWITCH:SELECT") {
    serial_parse_next_token(false); // get value from buffer
    if ((serial_val >= 0) && (serial_val <= switch_num_switches * switch_num_ports)) {
      switch_chain_selectfeed(serial_val); // switch on this feed
      Serial.print(SERIAL_HANDSHAKE_OK + serial_cmd + " "); // send feedback
      Serial.println(serial_val);
    } else {
      Serial.print(SERIAL_HANDSHAKE_ERR + serial_cmd + " ");
      Serial.print(serial_val);
      Serial.println(F(": Feed does not exist."));
    }
  } 
  //    get number of frequencies in sweep
  else if (serial_cmd == "SWEEP:POINTS?") {
    Serial.println(num_freq);
  } 
  //    get PLL register values
  else if (serial_cmd == "PLL:REGVALS?") {
    for(int i = 0; i < num_freq; i++) {
      Serial.print(((long)F2)    << 16 | ((long)F1)    << 8 | (long)F0, HEX);
      Serial.print(F("|"));
      Serial.print(((long)R2)    << 16 | ((long)R1)    << 8 | (long)R0, HEX);
      Serial.print(F("|"));
      Serial.println(((long)N2[i]) << 16 | ((long)N1[i]) << 8 | (long)N0[i], HEX);
    }  
  } 
  //    master reset
  else if (serial_cmd == "RESET") {
    Serial.println(SERIAL_HANDSHAKE_OK + serial_cmd); // send feedback
    DIG_SIGNAL_PORT = DIG_SIGNAL_PORT & (~DIG_SIGNAL_RSWI); // reset all bus devices
    delay(100); // allow for enough time to transfer the OK signal
    DIG_SIGNAL_PORT = DIG_SIGNAL_PORT & (~DIG_SIGNAL_MRST); // pull the Arduino's reset port to ground as well -> full reset
  } 
  //    unrecognized command; throw an error
  else {
    Serial.print(SERIAL_HANDSHAKE_ERR);
    Serial.print(F("Unrecognized command: "));
    Serial.println(serial_cmd);
  }
  // reset command buffer
  serial_reset_cmd_interface();
}
//############################################################
// parse string and return the next token (command string - whitespace - comma separated list of values)
//    returns true if a new token was found; false if not
boolean serial_parse_next_token(boolean iscmd) {
  //Serial.print(F("BUFFER (")); Serial.print(serial_inbuf.length()); Serial.print(F("): ")); Serial.println(serial_inbuf);
  // split index in string
  int ind_sep = 0;
  // check if buffer is already empty
  if (serial_inbuf.length() == 0){
    return false;
  }
  // next token is a command
  if (iscmd) {
    ind_sep = serial_inbuf.indexOf(SERIAL_SEP_C2V); // find command-value separator
    if (ind_sep < 0) { // no separator: select entire string (likely command only)
      ind_sep = serial_inbuf.length();
    }
    serial_cmd = serial_inbuf.substring(0,ind_sep); // -> command string
    //Serial.print(F("SEPARATOR POSITION: ")); Serial.println(ind_sep);
    //Serial.print(F("COMMAND: ")); Serial.println(serial_cmd);
  }
  // next token is an integer value
  else {
    ind_sep = serial_inbuf.indexOf(SERIAL_SEP_V2V); // find value-value separator
    if(ind_sep < 0){ // last value might not be terminated with SERIAL_SEP_V2V
      ind_sep = serial_inbuf.length();
    }
    serial_val = (serial_inbuf.substring(0,ind_sep)).toInt(); // -> long
    //Serial.print(F("SEPARATOR POSITION: ")); Serial.println(ind_sep);
    //Serial.print(F("VALUE: ")); Serial.println(serial_val);
  }
  // remove decoded token plus separator
  serial_inbuf_shift(ind_sep+1);
  //serial_inbuf = serial_inbuf.substring(ind_sep+1); // TODO: fails for long vectors (memory?)
  //Serial.print(F("REMAINING BUFFER (")); Serial.print(serial_inbuf.length()); Serial.print(F("): ")); Serial.println(serial_inbuf);
  return true;
}
//############################################################
// write serial input to buffer (copied from SerialEvent example)
void serial_input() {
  while (Serial.available()) {
    // prevent buffer overruns
    if (serial_inbuf.length() >= SERIAL_INBUF_SIZE) {
      Serial.print(SERIAL_HANDSHAKE_ERR);
      Serial.println(F("Input buffer overrun. Resetting."));
      serial_reset_cmd_interface();
    }
    // get the new character
    char inChar = (char)Serial.read();
    // if the incoming character is the EOL character...
    if (inChar == SERIAL_EOL) {
      // make sure we don't have any trailing/leading whitespaces
      serial_inbuf.trim();
      // and set the completion flag
      serial_inbuf_complete = true;
      //Serial.print(F("FINAL INBUF (")); Serial.print(serial_inbuf.length()); Serial.print(F("): ")); Serial.println(serial_inbuf);
    }
    else
    {
      // otherwise: check if the new character ist printable and
      //            add it to the input buffer
      if (ascii_isprintable(inChar)) {
        serial_inbuf += inChar;
      } else {
        // do nothing for now; could add an error / buffer reset later
      }
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
// is an ASCII character printable (decimal 32-126)
char ascii_isprintable(char c) {
  return (c > 31) && (c < 127);
}
//############################################################
// "delete" leading n characters from input buffer (in place to save memory)
// TODO: is there a better way to do this?
void serial_inbuf_shift(unsigned int n) {
  // delete string completely?
  if (n >= serial_inbuf.length()) {
    serial_inbuf = "";
  }
  // delete n leading characters by replacing them with whitespaces...
  for(unsigned int i = 0; i < n; i++) {
    serial_inbuf.setCharAt(i, ' ');
  }
  // ... which are then trimmed away
  serial_inbuf.trim();
}


//############################################################
// PLL / SWEEP CONTROL FUNCTIONS
//############################################################
// sends a pulse signal on the sync/handshake line (DAC)
void pulse_sync_signal(int signal) {
  // change the state to SIGNAL
  spi_dac_mcp4811(signal);
  // wait for pre-defined pulse duration
  delayMicroseconds(sweep_pulse_time);
  // reset the state
  spi_dac_mcp4811(SYNC_SIGNAL_RESET);
}  
//############################################################
// calculate function register value F (global), R-, and N-counter values (returned [R, N]) for ADF41020 PLL
// (copied and modified from ADF41020 host software)
// ... this function takes about 100us for the first point (i==0) and 64us for all others 
void pll_adf40120_addfreq(int RFout, int i) {
 
  // prevent adding too many frequencies
  if (i >= MAX_NUM_FREQ - 1) {
    Serial.print(SERIAL_HANDSHAKE_ERR);
    Serial.print(F("Reached maximum number of frequencies ("));
    Serial.print(MAX_NUM_FREQ);
    Serial.println(F(")."));
    return;
  }
  
  // basic device configuration; please refer to ADF41020 datasheet
  int  Prescaler = 1; // prescaler value (00: 8/9, 16/17, 32/33, 11:64/65)
  byte CPsetting1 = 3; // charge pump 1 setting (see manual)
  byte CPsetting2 = 3; // charge pump 2 setting (see manual)
  byte CPGain = 0; // charge pump gain (see manual)
  byte CP3state = 0; // charge pump three-state (0: normal, 1: three-state)
  byte Fastlock = 0; // fastlock setting (see manual)
  byte Timeout = 0; // timeout (PFD cycles; 3,7,11,13 ...)
  byte PDPolarity = 0; // phase detector polarity (0:neg, 1:pos)
  byte CounterReset = 0; // 0: normal, 1: R,A,B held in reset
  byte Powerdown = 0; // power down (see manual)
  byte Muxout = 0; // multiplexer output (see manual)

  // calculate P, R, N, B, & A values for calculating register 
  // (typecast to long for multiplication and division; multiplication first to avoid rounding errors)   
  unsigned int  P = (1 << Prescaler) << 3; // 1 << x = 2^x, x << 3 = x * 8
  unsigned int  R = (unsigned int)( ((unsigned long)PLL_RF_INPUT_FREQ * 1000) / PFDFreq ); // MHz -> kHz (like PFDFreq)
  unsigned int  N = (unsigned int)( ((unsigned long)RFout * 250) / PFDFreq ); // MHz -> kHz plus /4 for channel spacing
  unsigned int  B = N / P;
  byte A = (byte)( N - (B * P) );

  // modify fastlock and powerdown settings
  if (Fastlock==2) Fastlock++;
  if (Powerdown==2) Powerdown++;

  // calculate register values
  if (i == 0) {
    //    Function latch
    F2 = (byte)( Prescaler << 6 | (Powerdown & 0x2) << 5 | CPsetting1 << 2 | CPsetting2 >> 1 );
    F1 = (byte)( CPsetting2 << 7 | Timeout << 3 | Fastlock << 1 | CP3state);
    F0 = (byte)( PDPolarity << 7 | Muxout << 4 | (Powerdown & 0x1) << 3 | CounterReset << 2) | 0x2;
    //    R-counter (23-16: fixed,  15-2: R-Counter,  1-0: 00)
    R1 = (byte)(R >> 6);
    R0 = (byte)(R << 2) | 0x0;
  }
  //    N-counter (23-22: 00, 21: CP gain,  20-8: B-Counter,  7-2: A-Counter,  1-0: 01)
  N2[i] = 0b00111111 & (CPGain << 5 | (byte)(B >> 8));
  N1[i] = (byte)B;
  N0[i] = ((A&0x3F) << 2) | 0x1;
  
  // Debug output
  /*
  Serial.print(RFout);
   Serial.print(F("|"));
   Serial.print(((long)F2)    << 16 | ((long)F1)    << 8 | (long)F0, HEX);
   Serial.print(F("|"));
   Serial.print(((long)R2)    << 16 | ((long)R1)    << 8 | (long)R0, HEX);
   Serial.print(F("|"));
   Serial.print(((long)N2[i]) << 16 | ((long)N1[i]) << 8 | (long)N0[i], HEX);
   Serial.println(F(" "));
   */
}  
//############################################################
// initialize ADF40120 PLL (write F, R, and N)
void pll_adf40120_init() {
  spi_pll_adf40120(F2   , F1  ,  F0   );
  spi_pll_adf40120(R2   , R1  ,  R0   );
  spi_pll_adf40120(N2[0], N1[0], N0[0]);
}



//############################################################
// SWITCH CHAIN CONTROL FUNCTIONS
//############################################################
// reset switch chain
void switch_chain_reset() {
  for(byte i = 0; i < switch_num_switches; i++) {
    switch_states[i] = SWITCH_STATE_OFF;
  }
}
//############################################################
// initialize switch chain (set first switch)
void switch_chain_init() {
  // select first port and program
  switch_port_active = 1;
  switch_chain_selectfeed(switch_port_active);
}
//############################################################
// next switch
void switch_chain_next() {
  // select next port (circular)
  if (switch_port_active < switch_num_switches * switch_num_ports) {
    switch_port_active++;
  } else {
    switch_port_active = 1;
  }
  // program
  switch_chain_selectfeed(switch_port_active);
}
//############################################################
// select a specific port/feed (numbering starts at 1)
void switch_chain_selectfeed(int feed) {
  // set states to inactive (reset)
  switch_chain_reset();
  // calculate switch and port number for given feed (feed=0 -> reset)
  if ((feed > 0) && (feed <= switch_num_switches * switch_num_ports)) {
    feed = feed - 1; // feed # starts at 1
    byte sw = feed / switch_num_ports; // integer division takes care of floor()
    byte port = feed - sw * switch_num_ports;
    // change state of selected switch
    switch_states[sw] = 1 << port;
  }
  // program (note: program is needed outside if to make feed = 0 reset
  switch_chain_apply();
}
//############################################################
// apply current state to switch chain
void switch_chain_apply() {
  spi_drv_tle723x_set(switch_states);
}
//############################################################
// output switch states (binary)
void switch_chain_output_states() {
   for(byte i = 0; i < switch_num_switches - 1; i++) {
     Serial.print(switch_states[i], BIN);
     Serial.print(F(" "));
   }
   Serial.println(switch_states[switch_num_switches - 1], BIN);
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
  SPI_CE_PORT = SPI_CE_PORT & (~SPI_CE_MASK_DAC);
  // transfer data
  spi_write_int(value);
  // reset active low chip enable
  SPI_CE_PORT = SPI_CE_PORT | SPI_CE_MASK_DAC;
}
//############################################################
// ADF41020 PLL (DEFAULT SPI MODES)
void spi_pll_adf40120(byte b23to16, byte b15to8, byte b7to0) {
  // set active low chip enable
  SPI_CE_PORT = SPI_CE_PORT & (~SPI_CE_MASK_PLL);
  // transfer data
  SPI.transfer(b23to16);
  SPI.transfer(b15to8);
  SPI.transfer(b7to0);
  // reset active low chip enable
  SPI_CE_PORT = SPI_CE_PORT | SPI_CE_MASK_PLL;
}
//############################################################
// TLE723x switch driver chain, 16 bits/device (USES NON-DEFAULT SPI MODES)
//   bit 15-14: command (00: diagnostics, 01: read, 10: reset, 11: write)
//   bit 13-11: 0
//   bit 10- 8: address (CTL: 111)
//   bit  7- 0: data / state 
void spi_drv_tle723x_set(byte *state) {
  // prepare command
  byte cmd = 0b11000111; // write, CTL
  // set active low chip enable
  SPI_CE_PORT = SPI_CE_PORT & (~SPI_CE_MASK_SWI);
  // change SPI timing to what the TLE723x expects (this takes about 2-3us)
  SPI.setDataMode(SPI_MODE_TLE723X);
  SPI.setClockDivider(SPI_CLOCK_TLE723X);
  // transfer data for entire chain
  for(int i = 0; i < switch_num_switches; i++) {
    SPI.transfer(cmd);
    SPI.transfer(state[i]);
  }
  // revert SPI timing to defaults (this takes about 2-3us)
  SPI.setDataMode(SPI_MODE_DEFAULT);
  SPI.setClockDivider(SPI_CLOCK_DEFAULT);
  // reset active low chip enable
  SPI_CE_PORT = SPI_CE_PORT | SPI_CE_MASK_SWI;
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





//########################################################################################################################
//########################################################################################################################
// OLD AND RUSTY
//########################################################################################################################
//########################################################################################################################


//############################################################
/*
void serial_inbuf_shift(unsigned int n) {
  // delete string completely?
  if (n >= serial_inbuf.length()) {
    serial_inbuf = "";
  }
  // copy characters in string
  for(unsigned int i = 0; i < serial_inbuf.length(); i++) {
    serial_inbuf.setCharAt(i, serial_inbuf[i+n]);
  }
  // fill the rest of the string with whitespaces and trim
  //   TODO: this is a workaround because settings length-n to \0 does not work;
  //         why not?
  for(unsigned int i = serial_inbuf.length()-n; i < serial_inbuf.length(); i++) {
    serial_inbuf.setCharAt(i, ' ');
  }
  serial_inbuf.trim();
}
/*


//############################################################
// MAIN LOOP

  /* SERIAL CONTROL INTERFACE */
  /*
  //    get serial data
  if (Serial.available() > 0){
    serial_input();
  }
  //    if command is complete: parse
  if (serial_inbuf_complete){
    // decode command and reset interface
    serial_command_decode();
  }
  */
  
  /* SWITCH CONTROL: RUNNING BIT */
  /*
  // initialize switch chain (first switch active)
  switch_chain_init();
  // output currently active switch port on DAC output
  spi_dac_mcp4811(switch_port_active);
  delay(LONG_PAUSE);
  // cycle through all ports
  for(byte i = 0; i < switch_num_switches * switch_num_ports + 1; i++) {
    // next switch port
    switch_chain_next();
    // output currently active switch port on DAC output
    spi_dac_mcp4811(switch_port_active);
    // wait [ms]
    delay(LONG_PAUSE);
  }
  */

  /* SWITCH CONTROL: ARBITRARY PATTERNS */
  /*
  // initial state of all switches in the chain
  switch_states = {0, 0, 0}; 
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
   delayMicroseconds(sweep_dwell_time - sweep_pulse_time);
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
