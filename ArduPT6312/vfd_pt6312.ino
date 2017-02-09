/**
 * PT6312/uPD16321/CS16321 VFD controller
 *  supports multiple display modes
 *    4 digits with 16 segments upto 11 digits with 11 segments
 *  scanned keyboard input of upto 24 keys
 *  4 switches input
 *  4 LEDs output
 *  SPI bus 
*/

#include <SPI.h>





#define CMD_DISPLAY_MODE        0x00
#define DISPLAY_MODE            0x04  // 8 digits, 14 segements
#define CMD_DATA_WRITE_INC      0x40  // increment address, after data has written
#define CMD_DATA_WRITE_FIX      0x44  // fixed address
#define CMD_SET_ADDR            0xC0
#define CMD_DISPLAY_CTRL        0x80

void setup() {
  // put your setup code here, to run once:
  SPI.begin();
  initVFD();
}

void initVFD() {
  clearRAM();
}

void sendData(unsigned int data) {
  SPI.beginTransaction(SPISettings(50000000, LSBFIRST, SPI_MODE3));
  SPI.transfer(data);
  SPI.endTransaction();
}

void clearRAM() {
  sendCommand(incrementAddress, true);
  setAddress(startAddress, false);
  for(int i = 0; i <= endAddress; i++)
  {
    sendData(0x00, false);
  }
  digitalWrite(STB, HIGH);
}

void loop() {


  delayMicroseconds(1);
}
