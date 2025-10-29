#include <Arduino.h>
#include "Emulator.h"
#include <OneWire.h>

#define ONEWIRE_PIN 2

OneWireHub hub(ONEWIRE_PIN);
Emulator slaveEmu(0x01,0x02,0x03,0x04,0x05,0x06,0x07);

void setup() {
  Serial.begin(115200);
  Serial.println("Slave started");
  hub.attach(slaveEmu);
}

void loop() {

  hub.poll();

}
//wwewewefgitsdsdsdsdsd