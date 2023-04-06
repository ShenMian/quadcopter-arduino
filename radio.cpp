#include "radio.h"

RF24 radio(RF24_CE, RF24_CSN);

void setup_radio(uint8_t channel)
{
  while(!radio.begin());
  radio.setChannel(channel);
  radio.openReadingPipe(1, radio_address);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_1MBPS);
  radio.startListening();
}

void update_radio(EulerAngles& target_angles, Vector3& target_position, float& throttle)
{
  RadioPackage package;
  while (radio.available()) {
    radio.read(&package, sizeof(package));
    target_angles   = package.target_angles;
    target_position = package.target_position;
    throttle        = package.throttle;
  }
}