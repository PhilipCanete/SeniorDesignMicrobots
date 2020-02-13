// biosaw_lite.ino
// Biolite Firmware

// Written by: Kyle O'Rourke & Jackson Atwater
// 5/14/2019
// UCSC SDP

#include <SPI.h>
#include <string.h>
#include <stdlib.h>

// DDS VALUES:
#define ref_clk 25000000          // 25Mhz external reference clock.
#define sys_clk 500000000         // 500Mhz internal frequency from PLL.
#define FTW_CFB 0.11641532182     // Constant used for finding FTW: 500MHz/(2^32)
#define FTW_CBF 8.589934592       // Constant used for finding FTW: (2^32)/500MHz
#define frqAdjust 0               // Value added to output frequency for calibration.

// DDS REGISTERS:
const byte CSR  = 0x00; // Channel Select Register
const byte FR1  = 0x01; // Function Register 1: VCO, PLL
const byte FR2  = 0x02; // Function Register
const byte CFR  = 0x03; // Channel Function Register
const byte CFTW = 0x04; // Channel Frequency Tunning Word

// DDS REGISTER VALUES:
// Sets the value to be sent to each register.
// FIXED VALUES:
const byte          CSR_V = 0x82;       // Set Ch.3 EN, I/O mode 3-wire is 01 (1000 0010)
const unsigned long FR1_V = 0x00D00000; // Set VCO ON and PLL to 20x. 1101 0000
const unsigned long FR2_V = 0x00;       // Unused.
const unsigned long CFR_V = 0x00000302; // Set DAC FS current, Clear Phase accumulator.
// VARIABLE VALUES:
unsigned long CFTW_V = 0;  // Frequency tuning word. 32-bits.

// ADC VARIABLES:
#define pwrSlope 0.0199 // mV/dB 
#define ext_vref 1.939  // Voltage on external voltage ref.
const float voltMul = (ext_vref)/(1024); // Value of (ext_vref)/(2^10) Shoul be: 0.0018935546875

// SWEEP VALUES
const int sweepDelay = 1;           // Delay per frequency step in ms. (ADC SPS = 10kHz)
unsigned long startFreq = 40000000; // Start frequency Hz
unsigned long endFreq = 60000000;   // End frequency Hz


// PIN DEFINITIONS:1
const int rst_dds = 2;   // DDS reset pin.
const int io_update = 4; // DDS I/O update toggle.
const int P0 = 8;        // Sweep control pin
const int CS = 10;       // Chip select
//const int MOSI = 11;   // SPI MOSI
//const int MISO = 12;   // SPI MISO
//const int SCK = 13;    // SPI SCK
const int red_led = 3;   // RGB LED
const int green_led = 5; // RGB LED
const int blue_led = 6;  // RGB LED
const int ADC_1 = A0;    // ADC A0 Ref
const int ADC_2 = A2;    // ADC A2 Test

// SPI SETTINGS:
SPISettings settings(20000000, MSBFIRST, SPI_MODE0); 

// SETUP:
void setup() {

  // SERIAL SETUP:
  Serial.begin(9600); // USER'S COMPUTER SERIAL COM SETUP (NOT SPI)
  Serial.print("BioLite Startup...\n");
  Serial.print("voltMul = "); Serial.print(voltMul, 9);
  Serial.print("  Should be: "); Serial.print("0.0018935546875\n");
  Serial.print("FTW_CFB = "); Serial.print(FTW_CFB, 9);
  Serial.print("  Should be: "); Serial.print("0.11641532182\n");
  Serial.print("FTW_CBF = "); Serial.print(FTW_CBF, 8);
  Serial.print("  Should be: "); Serial.print("8.589934592\n");

  // PINMODE DEFINITIONS:
  pinMode(CS,        OUTPUT);
  pinMode(rst_dds,   OUTPUT);
  pinMode(io_update, OUTPUT);
  pinMode(P0,        OUTPUT);
  pinMode(red_led,   OUTPUT);
  pinMode(green_led, OUTPUT);
  pinMode(blue_led,  OUTPUT);
  pinMode(ADC_1,      INPUT);
  pinMode(ADC_2,      INPUT);

  // SET PIN INITIAL VALUES:
  digitalWrite(P0, LOW); // Ensure profile pin 0 low.
  digitalWrite(CS, LOW); // Ensure slave select high to start.
  dleds(0,0,0);
  //startupLEDS();         // Little LED show for startup.

  // ADC REFERENCE SETUP:
  analogReference(EXTERNAL);
  
  // SPI STARTUP MUST OCCUR AFTER PIN DEFINITIONS & BEFORE DDS SETUP:
  SPI.begin();

  // DDS SET INITIAL REGISTER VALUES:
  dds_setup();

  // SETUP FINISHED:
  Serial.println("Startup Finished. Waiting for commands.\n");
}

// MAIN LOOP:
void loop() {
  // WAIT FOR SERIAL COMMANDS:
  serialControl();
  //sweep(1000000, 250000000, 40000);

}

// SWEEP FUNCTION:
// Runs frequency sweep and prints each new frequency's power value
// in -dB for the given frequency. Returns run time in miliseconds.
void sweep(unsigned long start, unsigned long end, int step){
  Serial.print("SWEEP RUN TIME: "); Serial.print((((end-start)/step)*sweepDelay)/100); Serial.println("s");
  Serial.println("SWEEP START");
  delay(1000);
  if(start > end){
    for(unsigned long i = start; i >= end; i = i-step){
      setFrequency(i);
      delay(sweepDelay);
      int adc1_in = analogRead(ADC_1); // ref
      int adc2_in = analogRead(ADC_2); // test
      int adc_dif = adc2_in - adc1_in;
      float dif_vol = convVol(adc_dif);
      float dB_in   = convPwr(dif_vol);
      Serial.print(dB_in, 5); Serial.print(", "); Serial.println(i);
    }
  }
  else{
    for(unsigned long i = start; i <= end; i = i+step){
      setFrequency(i);
      delay(sweepDelay);
      int adc1_in = analogRead(ADC_1); // ref
      int adc2_in = analogRead(ADC_2); // test
      int adc_dif = adc2_in - adc1_in;
      float dif_vol = convVol(adc_dif);
      float dB_in   = convPwr(dif_vol);
      Serial.print(dB_in, 5); Serial.print(", "); Serial.println(i);
    }
  }
  setFrequency(0);
  Serial.println("SWEEP END");
}

// CONVERT VOLTAGE READING TO OUTPUT POWER:
float convPwr(float voltage){
  return (voltage/pwrSlope);
}

// CONVERT ADC VALUE TO VOLTAGE:
// RETURNS VOLTAGE AS A FLOAT
float convVol(int adc_val){
  return float(adc_val*voltMul);
}

// SETS THE OUTPUT OF THE DDS TO THE GIVEN FREQUENCY
void setFrequency(float frequency){
  if(frequency < 40000000){
    dleds(1,0,0); // Current power detectors only go down to 40Mhz.
  }
  else{
    dleds(0,0,0);
  }
  CFTW_V = calc_FTW(frequency+frqAdjust);
  ddsWrite_32(CFTW, CFTW_V); // FTW
  pulse(io_update);
}

void setFreqMhz(float frequency){
  setFrequency(frequency*1000000);
}

void setFreqkHz(float frequency){
  setFrequency(frequency*1000);
}

// Calculates the needed FTW for a desired frequency output:
unsigned long calc_FTW(float frequency){
  // Formula for finding frequency f_out = (FTW*250MHz)/(2^32)
  // Changes to FTW = f_out * FTW_CBF, where FTW_CBF is precalculated 2^32/250MHz.
  return (frequency * FTW_CBF);
}

// DDS SETUP:
// Reset DDS and set initial control registers. 
// More information on AD9959 datasheet Pg.36 Table 28.
void dds_setup(){
  
  // Toggle DDS Reset Pin:
  ddsReset(); // AFTER RESET CSR SHOULD CONTAIN 0XF0

  // Channel Select Register: CSR (ADR: 0x00)
  ddsWrite_8(CSR, CSR_V);
  
  // Function Register 1: FR1 (ADR: 0x01)
  ddsWrite_24(FR1, FR1_V);

  // Channel Function Register: CFR (ADR: 0x03)
  //ddsWrite_24(CFR, CFR_V);

  pulse(io_update); // Make settings take effect.
  delay(10); // Wait for system to settle.
}

// MASTER DDS RESET:
void ddsReset(){
  pulse(rst_dds); // Pulse reset pin.
}

// Simple function for writing 8 bit values
void ddsWrite_8(byte address, byte val){
  spiBegin();
  SPI.transfer(address);
  SPI.transfer(val);
  spiEnd();
}

// Simple function for writing 16 bit values
void ddsWrite_16(byte address, unsigned int val){
  spiBegin();
  SPI.transfer(address);
  SPI.transfer(&val, 2);
  spiEnd();
}

// Simple function for writing 3 byte values (Untested)
void ddsWrite_24(byte address, unsigned long val){

  byte a,b,c;
  a = (val       & 0xFF); //extract first byte (LS)
  b = ((val>>8)  & 0xFF); //extract second byte
  c = ((val>>16) & 0xFF); //extract third byte (MS)
  
  spiBegin();
  SPI.transfer(address);
  SPI.transfer(c); // We want to transfer MSB first.
  SPI.transfer(b);
  SPI.transfer(a);
  spiEnd();
}

// Simple function for writing 4 byte values
void ddsWrite_32(byte address, unsigned long val){

  byte a,b,c,d;
  a = (val       & 0xFF); //extract first byte (LS)
  b = ((val>>8)  & 0xFF); //extract second byte
  c = ((val>>16) & 0xFF); //extract third byte
  d = ((val>>24) & 0xFF); //extract fourth byte
  
  spiBegin();
  SPI.transfer(address);
  SPI.transfer(d);  // We want to transfer MSB first.
  SPI.transfer(c);
  SPI.transfer(b);
  SPI.transfer(a);
  spiEnd();
}

void spiBegin(){
  SPI.beginTransaction(settings); // Define SPI settings before transfer.
  digitalWrite(CS, LOW);          // Enable slave select.
}

void spiEnd(){
  digitalWrite(CS, HIGH);  // Disable slave select.
  SPI.endTransaction();    // End SPI transfer.
}

// Pulses given pin.
void pulse(uint8_t pin){
  digitalWrite(pin, LOW);
  digitalWrite(pin, HIGH);
  digitalWrite(pin, LOW);
}

// Simple LED control function PWM.
void leds(int R, int G, int B) {
  analogWrite(red_led, 255-R);
  analogWrite(green_led, 255-G);
  analogWrite(blue_led, 255-B);
}

// Simple LED control function non-PWM.
void dleds(int R, int G, int B) {
  digitalWrite(red_led, 1-R);
  digitalWrite(green_led, 1-G);
  digitalWrite(blue_led, 1-B);
}

// Cute little LED start function:
void startupLEDS(){
  for(int i = 0; i < 255; i++){
    leds(0,0,i);
    delay(3);
  }
  for(int i = 0; i < 255; i++){
    leds(0,i,255-i);
    delay(2);
  }
  for(int i = 255; i > 0; i--){
    leds(0,i,0);
    delay(2);
  }
  dleds(0,0,0);
}

// Looks for incoming serial information to act on.
// ASSUMES SANATIZED INPUTS!!!
// COMMANDS:
// SWEEP: START, END, STEPSIZE
// SETF: X (Hz)
// 
void serialControl(){
  if(Serial.available() > 0){ // Check for a new message on serial port.
    String incoming =  getNewWord(); // Gets first word in serial string.

    //COMMANDS:
    if(incoming == "S"){  // SWEEP WITH THE NEXT THREE VALUE IN THE INCOMMING SERIAL STRING.
      dleds(0,0,1);
      String str_frq = getNewWord(); // grabs the starting frequency.
      unsigned long strFrq = str_frq.toDouble();
      Serial.print("Start Frequency: "); Serial.println(strFrq);

      String end_frq = getNewWord(); // grabs the starting frequency.
      unsigned long endFrq = end_frq.toDouble();
      Serial.print("End Frequency: "); Serial.println(endFrq);

      String stp_size = getNewWord(); // grabs the starting frequency.
      int stp = stp_size.toInt();
      Serial.print("Step Size: "); Serial.println(stp);

      if(str_frq == "<ERROR>" || end_frq == "<ERROR>" || stp_size == "<ERROR>"){
        errorMsg();
      }
      else{
        sweep(strFrq, endFrq, stp);
        clearSerial();
      }
    }
    else if(incoming == "F"){ // SET FREQUENCY WITH THE NEXT WORD.
      dleds(0,0,1);
      String frq = getNewWord(); // grabs the starting frequency.
      unsigned long Frq = frq.toDouble();
      if(frq == "<ERROR>"){
        errorMsg();
      }
      else{
        Serial.print("\nSet Frequency to: "); Serial.println(Frq);
        setFrequency(Frq);
        clearSerial();
      }
    }
    else if(incoming == "HELP" || incoming == "INFO"){
      Serial.println("\nBioLite Possible Commands:");
      Serial.println("Sweep: S STARTF ENDF STEPSIZE");
      Serial.println("Set Freq: F FREQ (Hz)");
      Serial.println("Example: S 40000000 60000000 10000");
      clearSerial();
    }
    else {
      errorMsg();
    }
  }
  //Fade blue while in Serial loop.
  for(int i = 0; i != 255; i++){
    leds(0,0,i);
    delay(4);
  }
  for(int i = 255; i != 0; i--){
    leds(0,0,i);
    delay(4);
  }
}

// Simple serial read helper function. Returns string.
// Very slow blocking algorithm, not recommended for timing critical cases.
String getNewWord(){
  unsigned long timer = millis();
  while(!(Serial.available() > 0)){ // While no new serial in buffer, wait.
    if(millis() - timer > 200){
      return "<ERROR>";
    }
  } 
  String incoming = Serial.readStringUntil(' '); //Read message as a string.
  incoming.toUpperCase(); // All commands should be in uppercase.
  return incoming;
}

void errorMsg(){
  Serial.println();
  Serial.println("Command not recognized! Send 'HELP' for more info.");
  clearSerial();
}

void clearSerial(){
  while(Serial.available() > 0){
    Serial.read();
  }
}
