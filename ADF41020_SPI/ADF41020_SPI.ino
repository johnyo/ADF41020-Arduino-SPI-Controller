//############################################################
//           SPI Serial Interface for the ADF41020
//############################################################
/* 
We have a 14 Bit R Counter, 19 Bit N Counter, and a
21 Bit Function Latch. All are sent to a 24 bit shift register, 
the 2 LSBs dictate the  final destination. The serial order of 
data is Function, R, N 
*/

// For SPI Communications
#include <SPI.h>
// For adjusting the system clock register
#include <io.h>

//############################################################
//                 PRECALCULATED SPI DATA                   //
//############################################################

// R Counter Latch is static at 0x910084
byte R2 = 0x91; // MSBs
byte R1 = 0x00;
byte R0 = 0x84; // LSBs

// Function Latch is static at 0x4D8002
byte F2 = 0x4D; // MSBs
byte F1 = 0x80;
byte F0 = 0x02; // LSBs

// Precalculated byte arrays for N Counter
byte N2 = 0x00; // MSBs

// Middle Byte of N Counter
byte N1 [] = {
0x37,
0x37,
0x37,
0x37,
0x37,
0x37,
0x37,
0x38,
0x38,
0x38,
0x38,
0x38,
0x38,
0x38,
0x38,
0x39,
0x39,
0x39,
0x39,
0x39,
0x39,
0x39,
0x39,
0x3A,
0x3A,
0x3A,
0x3A,
0x3A,
0x3A,
0x3A,
0x3A,
0x3B,
0x3B,
0x3B,
0x3B,
0x3B,
0x3B,
0x3B,
0x3B,
0x3C,
0x3C,
0x3C,
0x3C,
0x3C,
0x3C,
0x3C,
0x3C,
0x3D,
0x3D,
0x3D,
0x3D,
0x3D,
0x3D,
0x3D,
0x3D,
0x3E,
0x3E,
0x3E,
0x3E,
0x3E,
0x3E,
0x3E,
0x3E,
0x3F,
0x3F,
0x3F,
0x3F,
0x3F,
0x3F,
0x3F,
0x3F,
0x40,
0x40,
0x40,
0x40,
0x40,
0x40,
0x40,
0x40,
0x41,
0x41,
0x41,
0x41,
0x41,
0x41,
0x41,
0x41,
0x42,
0x42,
0x42,
0x42,
0x42,
0x42,
0x42,
0x42,
0x43,
0x43,
0x43,
0x43,
0x43,
0x43 };

// Least significant byte of N Counter
byte N0 [] = {
0x0D,
0x15,
0x1D,
0x25,
0x2D,
0x35,
0x3D,
0x05,
0x0D,
0x15,
0x1D,
0x25,
0x2D,
0x35,
0x3D,
0x05,
0x0D,
0x15,
0x1D,
0x25,
0x2D,
0x35,
0x3D,
0x05,
0x0D,
0x15,
0x1D,
0x25,
0x2D,
0x35,
0x3D,
0x05,
0x0D,
0x15,
0x1D,
0x25,
0x2D,
0x35,
0x3D,
0x05,
0x0D,
0x15,
0x1D,
0x25,
0x2D,
0x35,
0x3D,
0x05,
0x0D,
0x15,
0x1D,
0x25,
0x2D,
0x35,
0x3D,
0x05,
0x0D,
0x15,
0x1D,
0x25,
0x2D,
0x35,
0x3D,
0x05,
0x0D,
0x15,
0x1D,
0x25,
0x2D,
0x35,
0x3D,
0x05,
0x0D,
0x15,
0x1D,
0x25,
0x2D,
0x35,
0x3D,
0x05,
0x0D,
0x15,
0x1D,
0x25,
0x2D,
0x35,
0x3D,
0x05,
0x0D,
0x15,
0x1D,
0x25,
0x2D,
0x35,
0x3D,
0x05,
0x0D,
0x15,
0x1D,
0x25,
0x2D
};

//############################################################
//                          SETUP                           //
//############################################################
// Setup function, only run at the beginning
void setup() {
  
  // Divide system clock
  // Tell the AtMega we want to change the system clock
  CLKPR = 0x80;    
  // 0x08 is a 1/256 prescaler = 60KHz for a 16MHz crystal
  // 0x03 gets us to around 1ms per 24 bit frame
  CLKPR = 0x03;    

  // Pin 7 is Load Enable (LE). 
  // It goes high after every 24 bit frame
  // Pin 6 is Start Indicator (SI). 
  // It goes high only at the beginning of every series
  
  // Set pints 6 and 7 as outputs without changing the values
  // of any other pins. 
  // This is a C command so I can drive the pins
  // faster than using the standard Arduino synatx.
  DDRD = DDRD | B11000000;
  
  // Initialize LE and SI to low
  PORTD = B00000000;
  
  // declare a slave select pin for SPI
  pinMode(SS, OUTPUT);
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
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  
  // Send 3 bytes of Function Latch Data. This is the same each time
  SPIwrite24bitRegister(F2, F1, F0, false);

  // Send 3 bytes of R Counter Data. This is the same each time
  SPIwrite24bitRegister(R2, R1, R0, false);
 
}

//############################################################
//                    WRITE 3 BYTES OVER SPI                //
//############################################################
void SPIwrite24bitRegister(byte b23to16, byte b15to8, byte b7to0, boolean NewFrame) {


    
  // Pull down SS
  digitalWrite(SS, LOW);
  
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
    
  delay(1);
 
  // After one byte transmission, set both LE and SI back low
  PORTD = B00000000;
  
  // Return SS to high
  digitalWrite(SS, HIGH);
  
  delay(10);
}

//############################################################
//                        MAIN LOOP                         //
//############################################################
void loop() {

  // Loop for the number of values in our frequency sweep
  for(int i = 0; i < sizeof(N0); i++) {
   
    if(i==0)
        SPIwrite24bitRegister(N2, N1[i], N0[i],true);
    else
        SPIwrite24bitRegister(N2, N1[i], N0[i],false);   
  }
  
  delay(100);
}

