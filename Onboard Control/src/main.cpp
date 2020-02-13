#include <Arduino.h>
#include <SPI.h>

/* Default Pins are as follows, from C:\Users\19168\.platformio\packages\framework-arduinonordicnrf5\variants\nRF52DK\variant.h
MOSI = 12
MISO = 11
SCLK = 13
*/


//Configured SPI Settings
const SPISettings settings(1000000, MSBFIRST, SPI_MODE0); //1MHz SPI speed

//DDS Settings
const uint8_t DDSCHIPSELECT = 10;
const uint32_t DDSCONTROLREGISTER = 0x0;
const uint32_t LEFTFTW = 0x22222222; //10Mhz @ 75Mhz in
const uint32_t STRAIGHTFTW = 0x33333333; //15Mhz @ 75Mhz in
const uint32_t RIGHTFTW = 0x44444444; //20Mhz @ 75Mhz in



void spiWrite(uint8_t address, uint32_t value, uint32_t chipSelect) {
  // Set specified chipSelect GPIO to 0, as SPI is active low
  digitalWrite(chipSelect, 0x0);         
  // Send in the address and value via SPI:
  SPI.transfer(address);      //SPI.transfer(param) will only transfer 8 bits
  SPI.transfer(&value, 32);   //SPI.transfer(dataPointer,sizeOfData) will transfer sizeOfData bits
  // Set specified chipSelect GPIO to 1, as SPI is active high
  digitalWrite(chipSelect, HIGH);
  // Release SPI bus
  SPI.endTransaction();   

}



void setup() {
  // set the ddsChipSelect as an output:
  pinMode (DDSCHIPSELECT, OUTPUT);
  // initialize SPI:
  SPI.beginTransaction(settings);
}

void loop() {
  //Send LOW frequency
  spiWrite(DDSCONTROLREGISTER, LEFTFTW, DDSCHIPSELECT);
  delay(1000);

  //Send STRAIGHT frequency
  spiWrite(DDSCONTROLREGISTER, STRAIGHTFTW, DDSCHIPSELECT);
  delay(1000);

  //Send HIGH frequency
  spiWrite(DDSCONTROLREGISTER, RIGHTFTW, DDSCHIPSELECT);
  delay(1000);

}