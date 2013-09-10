//############################################################
// SPI Serial Interface for the ADF41020
//############################################################
/*
We have a 14 Bit R Counter, 19 Bit N Counter, and a
21 Bit Function Latch. All are sent to a 24 bit shift register,
the 2 LSBs dictate the final destination. The serial order of
data is Function, R (send once), then N repeating. There is
no SS or MISO, only CLK, MOSI, LE.

The 3 signals that must connect to the ADF41020:
SPI MOSI: Pin 11
SPI CLK: PIN 13
LE: Pin 7

Note there is no SS or MISO. I also added another signal:
SI: Pin 6

This is an indicator that goes high before eqch frequency
sweep. It is not needed by the ADF41020 and is only used
for external syncing purposes.
*/

// For SPI Communications
#include <SPI.h>

// How long we keep LE high for in microseconds
#define LE_DURATION 500 // Must be smaller than 16383
// Delay between 24 bit serial data being sent
// in microseconds
// Must be larger than LE_DURATION
#define TRANSMIT_DELAY 1000//Must be smaller than 16383
// Delay between frequency sweeps
// in microseconds
#define SWEEP_DELAY 0 // Must be smaller than 16383
// Integer values can be maxed out if large enough
// This value is used to indicate to the delayer function
// if the pause is too large.
#define DELAY_IS_MICROSECONDS false 

/* Here is how the delays work together:
D0 (3 bytes) sent over spi
LE then goes high for LE_DURATION
Then we wait for TRANSMIT_DELAY
D1 (3 bytes) sent over spi
LE then goes high for LE_DURATION
Then we wait for TRANSMIT_DELAY
.
.
.
Dend (3 bytes) sent over spi
LE then goes high for LE_DURATION
Then we wait for TRANSMIT_DELAY
Then we also wait for SWEEP_DELAY
Then it starts again from D0
*/

//############################################################
// PRECALCULATED SPI DATA //
//############################################################

// R Counter Latch is static at 0x910140
byte R2 = 0x91; // MSBs
byte R1 = 0x01;
byte R0 = 0x40; // LSBs

// Function Latch is static at 0x4D8002
byte F2 = 0x4D; // MSBs
byte F1 = 0x80;
byte F0 = 0x02; // LSBs

// Precalculated byte arrays for N Counter
byte N2 = 0x00; // MSBs

// Middle Byte of N Counter
byte N1 [] = {
0x83,
0x84,
0x85,
0x87,
0x88,
0x89,
0x8A,
0x8C,
0x8D,
0x8E,
0x8F,
0x91,
0x92,
0x93,
0x94,
0x96,
0x97,
0x98,
0x99,
0x9B,
0x9C,
0x9D,
0x9E,
0xA0,
0xA1,
0xA2,
0x83,
0x84,
0x85,
};

// Least significant byte of N Counter
byte N0 [] = {
0x11,
0x21,
0x31,
0x01,
0x11,
0x21,
0x31,
0x01,
0x11,
0x21,
0x31,
0x01,
0x11,
0x21,
0x31,
0x01,
0x11,
0x21,
0x31,
0x01,
0x11,
0x21,
0x31,
0x01,
0x11,
0x21,
0x11,
0x21,
0x31,
};

//############################################################
// SETUP //
//############################################################
// Setup function, only run at the beginning
void setup() {
  
  // Set pints 6 and 7 as outputs without changing the values
  // of any other pins.
  // This is a C command so I can drive the pins
  // faster than using the standard Arduino synatx.
  DDRD = DDRD | B11000000;
  
  // Initialize LE and SI to low
  PORTD = B00000000;
  
  // Start the SPI library
  SPI.begin();
  // Set the bit order
  // The AD41020 expects the MSB to come first
  SPI.setBitOrder(MSBFIRST);
  // Set the data mode. Data is sent on the rising edge
  // and the clock is idle when low. This is Mode 0
  SPI.setDataMode(SPI_MODE0);
  // Set the speed we want to transmit at. This divides
  // the 16 MHz system clock. Default is divide by 4.
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  
  // Wait just a bit
  delayer(SWEEP_DELAY, DELAY_IS_MICROSECONDS);
  
  // Send 3 bytes of Function Latch Data. This is the same each time
  // So we only send it once during setup
  SPI.transfer(F2);
  SPI.transfer(F1);
  SPI.transfer(F0);
  
  // Now we must set LE high for a bit to push the data out of the shift register
  PORTD = B10000000;
  delayMicroseconds(LE_DURATION);
  PORTD = B00000000;
 
  delayer(TRANSMIT_DELAY, DELAY_IS_MICROSECONDS);
 
  // Send 3 bytes of R Latch Data. This is the same each time
  // So we only send it once during setup
  SPI.transfer(R2);
  SPI.transfer(R1);
  SPI.transfer(R0);
  
  // Now we must set LE high for a bit to push the data out of the shift register
  PORTD = B10000000;
  delayMicroseconds(LE_DURATION);
  PORTD = B00000000;
 
  delayer(TRANSMIT_DELAY, DELAY_IS_MICROSECONDS);
}

//############################################################
// POPULATE THE REGISTER ARRAY
//############################################################

void calcRegisters(String RFOutFreqBox, String RefFreqBox, String PFDFreqBox){
  
  // Initialize parameters for the sweep
  PrescalerBoxIndex = 1;
  ChargePumpSetting1BoxSelectedIndex = 3;
  ChargePumpSetting2BoxSelectedIndex = 3;
  ChargePumpGainBoxSelectedIndex = 0;
  ChargePump3StateBoxSelectedIndex = 0;
  FastLockBoxSelectedIndex = 0;
  TimeoutBoxSelectedIndex = 0;
  PhaseDetectorPolarityBoxSelectedIndex = 1;
  CounterResetBoxSelectedIndex = 0;
  LockDetectPrecisionBoxSelectedIndex = 0;
  PowerDownBoxSelectedIndex = 0;
  ABPWBoxSelectedIndex=0;
  SyncBoxSelectedIndex=0;
  DelayBoxSelectedIndex=0;
  MuxoutBoxSelectedIndex=0;
  TestmodesBoxSelectedIndex=0;
  PhaseDetectoPolarityBoxSelectedIndex=0;
  // end params
  
  float RFout = atof(RFOutFreqBox);
  float REFin = atof(RefFreqBox);
  float PFDFreq = atof(PFDFreqBox);
  
  RFout /= 4;
  
  P = (int)pow(2,8*PrescalerBoxIndex);
  R = (int)(REFin*1000/PFDFreq);
  N = (int)(RFout*1000/PFDFreq);
  B = (int)(N/P);
  A = (int)(N-(B*P));
  
  BString=(String)B;
  PString=(String)P;
  AString=(String)A;
  PFDString=PFDFreqBox;
  RFoutString=(String)(((B*P+A)*PFDFreq) / 1000);
  Prescaler = (byte)PrescalerBoxIndex;
  CPsetting1 = (byte)ChargePumpSetting1SelectedIndex;
  CPsetting2 = (byte)ChargePumpSetting2SelectedIndex;
  CPGain = (byte)ChargePumpGainBoxSelectedIndex;
  CP3state = (byte)ChargePump3StateBoxSelectedIndex;
  Fastlock = (byte)FastLockBoxSelectedIndex;
  
  if (Fastlock==2) FastLock++;
  
  Timeout = (byte)TimeoutBoxSelectedIndex;
  PDPolarity = (byte)PhaseDetectorPolarityBoxSelectedIndex;
  CounterReset = (byte)CounterResetBoxSelectedIndex;
  LDP = (byte)LockDetectPrecisionBoxSelectedIndex;
  Powerdown = (byte)PowerDownBoxSelectedIndex;
  
  if (Powerdown==2) Powerdown++;
  
  ABPW = (byte)ABPWBoxSelectedIndex;
  Sync = (byte)SyncBoxSelectedIndex;
  Delay = (byte)DelayBoxSelectedIndex;
  Muxout = (byte)MuxoutBoxSelectedIndex;
  
  Testmodes = (byte)TestmodesBoxSelectedIndex;
  
  Reg[0] = (int)( pow(2,23)+pow(2, 20) + Testmodes*pow(2,16) + (R & 0x3FFF) * pow(2,2) );
  Reg[1] = (int)( CPGain * pow(2,21)+(B&0x1FFF)*pow(2,8)+(A& 0x3F)*pow(2,2)+1};
  Reg[2] = (int)( Prescaler * pow(2,22)+CPSetting2*pow(2,18)+CPsetting1*pow(2,15)+Timeout*pow(2,11)+Fastloc*pow(2,9)+CP#state*pow(2,8)+PDPolarity*pow(2,7)+Muxout*pow(2,4)+Powerdown*pow(2,3)+CounterReset*pow(2,2)+2);
  }
  
  RCounterLatchBox = "{0:X}"+(String)Reg[0];
  NCounterLatchBox = "{0:X}"+(String)Reg[1];
  FunctionLatchBox = "{0:X}"+(String)Reg[2];
  }
//############################################################
// DELAY FUNCTION TO AVOID INT MAX
//############################################################

void delayer(int delval, boolean delayType){
  
  if(DELAY_IS_MICROSECONDS)
    delayMicroseconds(delval);
  else
    delay(delval);
    
}

//############################################################
// WRITE 3 BYTES OVER SPI //
//############################################################
void SPIwrite24bitRegister(byte b23to16, byte b15to8, byte b7to0, boolean NewFrame) {
  
  // Transfer the first byte
  // This also serves as a timer for how long we keep
  // LE and SI high
  SPI.transfer(b23to16);
  SPI.transfer(b15to8);
  SPI.transfer(b7to0);
  
  // If NewFrame is true, we are at the beginning of a new set of
  // data, and both SI and LE must be high
  if(NewFrame)
    PORTD = B11000000;
  // Otherwise, we are mid-data transmit, and only LE must be high
  else
    PORTD = B10000000;
    
  delayMicroseconds(LE_DURATION);
  // After one byte transmission, set both LE and SI back low
  PORTD = B00000000;
  
delayer(TRANSMIT_DELAY, DELAY_IS_MICROSECONDS);
}

//############################################################
// MAIN LOOP //
//############################################################
void loop() {

  // Loop for the number of values in our frequency sweep
  for(int i = 0; i < sizeof(N0); i++) {
   
    if(i==0)
        SPIwrite24bitRegister(N2, N1[i], N0[i],true);
    else
        SPIwrite24bitRegister(N2, N1[i], N0[i],false);
  }
  
  delayer(SWEEP_DELAY, DELAY_IS_MICROSECONDS);
}
