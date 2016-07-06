/*
 SPI slave opearation by software processing.
 by Takumi Kodama <http://arduino.alpaca-san.jp>
*/

#include <SoftwareSPISlave.h>

void setup() {
  Serial.begin(9600);

  // SCK and SS pin must be a PCINT pin.
  SPISlave.begin(MISO, MOSI, SCK, 8);
}

void loop() {
  if (SPISlave.available()) {
  	Serial.write(SPISlave.read());
  }
}
